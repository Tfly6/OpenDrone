#!/usr/bin/env python3
"""Build the free/occupied terrain-map contract expected by Air-FAR.

``/registered_scan`` contains lidar returns in the map frame.  A return is an
obstacle *endpoint*, not a free-space sample.  Air-FAR consumes endpoints from
the scan topic and consumes intensity < ``terrain_free_z`` points from its
terrain topic as free space.  This node raycasts the scan returns to restore
that distinction.
"""

import time

import numpy as np
import rospy
import tf2_ros
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FREE_INTENSITY = 0.0
OCCUPIED_INTENSITY = 255.0


class AirfarTerrainMapper:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/registered_scan")
        self.output_topic = rospy.get_param("~output_topic", "/airfar/terrain_map_ext")
        self.map_frame = rospy.get_param("~map_frame", "map").strip("/")
        self.sensor_frame = rospy.get_param("~sensor_frame", "livox_link").strip("/")
        self.ray_step = float(rospy.get_param("~ray_step", 0.20))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.20))
        self.endpoint_clearance = float(rospy.get_param("~endpoint_clearance", 0.75))
        self.min_range = float(rospy.get_param("~min_range", 0.20))
        self.max_range = float(rospy.get_param("~max_range", 25.0))
        self.map_radius = float(rospy.get_param("~map_radius", 30.0))
        self.observation_ttl = float(rospy.get_param("~observation_ttl", 10.0))
        self.max_rays = int(rospy.get_param("~max_rays", 2500))
        self.max_map_voxels = int(rospy.get_param("~max_map_voxels", 120000))
        self.lookup_timeout = float(rospy.get_param("~lookup_timeout", 0.05))
        self.log_interval = float(rospy.get_param("~log_interval", 1.0))

        if min(self.ray_step, self.voxel_size, self.endpoint_clearance, self.max_range) <= 0.0:
            raise ValueError("ray_step, voxel_size, endpoint_clearance and max_range must be positive")
        if self.max_range <= self.min_range:
            raise ValueError("max_range must be greater than min_range")

        self._voxels = {}  # (ix, iy, iz) -> (last_observed_sec, intensity)
        self._received_clouds = 0
        self._published_clouds = 0
        self._published_since_log = 0
        self._tf_failures = 0
        self._frame_rejections = 0
        self._last_status_time = time.monotonic()
        self._buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self._subscriber = rospy.Subscriber(self.input_topic, PointCloud2, self._callback, queue_size=1)

    def _sensor_origin(self, stamp):
        transform = self._buffer.lookup_transform(
            self.map_frame, self.sensor_frame, stamp, rospy.Duration(self.lookup_timeout))
        translation = transform.transform.translation
        return np.array([translation.x, translation.y, translation.z], dtype=np.float32)

    def _callback(self, cloud):
        self._received_clouds += 1
        stamp = cloud.header.stamp if cloud.header.stamp != rospy.Time() else rospy.Time.now()
        try:
            origin = self._sensor_origin(stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._tf_failures += 1
            rospy.logwarn_throttle(1.0, "Air-FAR terrain mapper: TF unavailable: %s", exc)
            self._log_status()
            return

        if cloud.header.frame_id.strip("/") != self.map_frame:
            self._frame_rejections += 1
            rospy.logwarn_throttle(
                1.0, "Air-FAR terrain mapper expected frame '%s', received '%s'; skipping cloud",
                self.map_frame, cloud.header.frame_id)
            self._log_status()
            return

        endpoints = np.asarray(
            list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        if endpoints.size == 0:
            self._prune(stamp.to_sec(), origin)
            self._publish(stamp)
            return
        endpoints = endpoints.reshape((-1, 3))
        directions = endpoints - origin
        ranges = np.linalg.norm(directions, axis=1)
        valid = (ranges >= self.min_range) & (ranges <= self.max_range)
        endpoints, directions, ranges = endpoints[valid], directions[valid], ranges[valid]
        if endpoints.shape[0] > self.max_rays:
            indices = np.linspace(0, endpoints.shape[0] - 1, self.max_rays, dtype=np.int64)
            endpoints, directions, ranges = endpoints[indices], directions[indices], ranges[indices]
        if endpoints.shape[0] == 0:
            self._prune(stamp.to_sec(), origin)
            self._publish(stamp)
            return

        unit_directions = directions / ranges[:, None]
        free_limit = np.maximum(ranges - self.endpoint_clearance, 0.0)
        free_points = []
        for distance in np.arange(self.ray_step, self.max_range, self.ray_step, dtype=np.float32):
            mask = free_limit >= distance
            if np.any(mask):
                free_points.append(origin + unit_directions[mask] * distance)

        now = stamp.to_sec()
        if free_points:
            self._update_voxels(np.concatenate(free_points, axis=0), FREE_INTENSITY, now)
        # Endpoint updates come after free-space updates, so a real hit remains occupied.
        self._update_voxels(endpoints, OCCUPIED_INTENSITY, now)
        self._prune(now, origin)
        self._publish(stamp)

    def _update_voxels(self, points, intensity, stamp):
        keys = np.floor(points / self.voxel_size).astype(np.int32)
        keys = np.unique(keys, axis=0)
        for key in keys:
            self._voxels[(int(key[0]), int(key[1]), int(key[2]))] = (stamp, intensity)

    def _prune(self, stamp, origin):
        min_stamp = stamp - self.observation_ttl
        radius_sq = self.map_radius * self.map_radius
        # This function runs for every lidar frame.  A rolling map can contain
        # 120k voxels; doing one NumPy allocation and norm per dictionary item
        # made pruning alone take most of a scan period.  Snapshot once, then
        # evaluate expiry/range in a vectorized pass.
        items = list(self._voxels.items())
        if not items:
            return
        keys = np.asarray([key for key, _value in items], dtype=np.float32)
        last_seen = np.fromiter(
            (value[0] for _key, value in items), dtype=np.float64, count=len(items))
        centers = (keys + 0.5) * self.voxel_size
        delta = centers - origin
        stale_mask = ((last_seen < min_stamp) |
                      (np.einsum('ij,ij->i', delta, delta) > radius_sq))
        for index in np.flatnonzero(stale_mask):
            key = items[int(index)][0]
            del self._voxels[key]
        if len(self._voxels) > self.max_map_voxels:
            oldest = sorted(self._voxels, key=lambda key: self._voxels[key][0])
            for key in oldest[:len(self._voxels) - self.max_map_voxels]:
                del self._voxels[key]

    def _publish(self, stamp):
        header = Header(frame_id=self.map_frame, stamp=stamp)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        # sensor_msgs.point_cloud2.create_cloud packs every tuple through
        # Python's struct module.  Publishing 100k+ rolling-map voxels that
        # way makes the mapper lag behind the 5 Hz scan.  Build the standard
        # XYZI binary layout in NumPy instead; the wire format is identical.
        items = list(self._voxels.items())
        point_count = len(items)
        xyzi = np.empty((point_count, 4), dtype='<f4')
        if point_count:
            keys = np.asarray([key for key, _value in items], dtype=np.float32)
            xyzi[:, :3] = (keys + 0.5) * self.voxel_size
            xyzi[:, 3] = np.fromiter(
                (value[1] for _key, value in items), dtype=np.float32, count=point_count)
        cloud = PointCloud2(
            header=header,
            height=1,
            width=point_count,
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * point_count,
            data=xyzi.tobytes(),
            is_dense=True,
        )
        self._publisher.publish(cloud)
        self._published_clouds += 1
        self._published_since_log += 1
        self._log_status()

    def _log_status(self):
        """Emit mapper health and semantic-map composition at a stable rate."""
        now = time.monotonic()
        elapsed = now - self._last_status_time
        if elapsed < self.log_interval:
            return
        free_voxels = sum(1 for _stamp, intensity in self._voxels.values()
                          if intensity == FREE_INTENSITY)
        obstacle_voxels = len(self._voxels) - free_voxels
        publish_rate = self._published_since_log / elapsed
        rospy.loginfo(
            "Air-FAR terrain mapper: pub_rate=%.2fHz published=%d received=%d "
            "free_voxels=%d obstacle_voxels=%d tf_failures=%d frame_rejections=%d",
            publish_rate, self._published_clouds, self._received_clouds,
            free_voxels, obstacle_voxels, self._tf_failures, self._frame_rejections)
        self._published_since_log = 0
        self._last_status_time = now


if __name__ == "__main__":
    rospy.init_node("airfar_terrain_mapper")
    AirfarTerrainMapper()
    rospy.spin()

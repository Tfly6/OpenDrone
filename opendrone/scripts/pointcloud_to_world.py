#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudToWorld:
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'world')
        self.source_topic = rospy.get_param('~source_topic', '/livox/lidar')
        self.output_topic = rospy.get_param('~output_topic', '/livox/lidar_world')
        self.source_frame = rospy.get_param('~source_frame', 'livox_link')
        self.lookup_timeout = rospy.get_param('~lookup_timeout', 0.1)
        self.use_latest_on_extrapolation = rospy.get_param('~use_latest_on_extrapolation', True)
        self.filter_enable = rospy.get_param('~filter_enable', True)
        self.voxel_leaf_size = rospy.get_param('~voxel_leaf_size', 0.08)
        self.min_range = rospy.get_param('~min_range', 0.2)
        self.max_range = rospy.get_param('~max_range', 50.0)
        self.min_z = rospy.get_param('~min_z', None)
        self.max_z = rospy.get_param('~max_z', None)
        self.max_points = rospy.get_param('~max_points', 200000)
        self.cloud_timeout = rospy.get_param('~cloud_timeout', 0.5)
        self.log_interval = rospy.get_param('~log_interval', 2.0)
        self.filter_log = rospy.get_param('~filter_log', True)

        self.last_msg_time = None
        self.last_cloud_stamp = None
        self.tf_fail_count = 0
        self.tf_extrap_count = 0

        self.watchdog_timer = rospy.Timer(rospy.Duration(0.2), self._watchdog)

        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.source_topic, PointCloud2, self.callback, queue_size=1)

    def callback(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.last_cloud_stamp = msg.header.stamp
        frame_id = msg.header.frame_id.strip('/') if msg.header.frame_id else self.source_frame
        # 处理 Gazebo 命名空间（例如 "Mid360::livox_link" -> "livox_link"）
        if '::' in frame_id:
            frame_id = frame_id.split('::')[-1]
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time(0)

        try:
            transform = self.buffer.lookup_transform(
                self.target_frame,
                frame_id,
                stamp,
                rospy.Duration(self.lookup_timeout)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as exc:
            self.tf_fail_count += 1
            rospy.logwarn_throttle(
                self.log_interval,
                "TF lookup failed: %s (fail_count=%d, cloud_stamp=%.3f)",
                exc, self.tf_fail_count, stamp.to_sec())
            return
        except tf2_ros.ExtrapolationException as exc:
            if not self.use_latest_on_extrapolation:
                self.tf_fail_count += 1
                rospy.logwarn_throttle(
                    self.log_interval,
                    "TF lookup failed: %s (fail_count=%d, cloud_stamp=%.3f)",
                    exc, self.tf_fail_count, stamp.to_sec())
                return
            self.tf_extrap_count += 1
            rospy.logwarn_throttle(
                self.log_interval,
                "TF extrapolation: %s (extrap_count=%d, cloud_stamp=%.3f). Falling back to latest TF.",
                exc, self.tf_extrap_count, stamp.to_sec())
            try:
                transform = self.buffer.lookup_transform(
                    self.target_frame,
                    frame_id,
                    rospy.Time(0),
                    rospy.Duration(self.lookup_timeout)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc2:
                self.tf_fail_count += 1
                rospy.logwarn_throttle(
                    self.log_interval,
                    "TF lookup failed: %s (fail_count=%d)",
                    exc2, self.tf_fail_count)
                return

        cloud_out = do_transform_cloud(msg, transform)
        cloud_out.header.frame_id = self.target_frame
        if not self.filter_enable:
            self.pub.publish(cloud_out)
            return

        filtered_cloud = self._filter_cloud(cloud_out)
        self.pub.publish(filtered_cloud)

    def _watchdog(self, _event):
        if self.last_msg_time is None:
            return

        dt = (rospy.Time.now() - self.last_msg_time).to_sec()
        if dt > float(self.cloud_timeout):
            stamp = self.last_cloud_stamp.to_sec() if self.last_cloud_stamp else 0.0
            rospy.logwarn_throttle(
                self.log_interval,
                "No pointcloud received for %.3fs (last_cloud_stamp=%.3f, tf_fail=%d, tf_extrap=%d)",
                dt, stamp, self.tf_fail_count, self.tf_extrap_count)

    def _filter_cloud(self, cloud_msg):
        points = np.array(
            list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )
        initial_count = points.shape[0]

        if points.size == 0:
            if self.filter_log:
                rospy.logwarn_throttle(
                    self.log_interval,
                    "Pointcloud empty after NaN removal (frame=%s)",
                    cloud_msg.header.frame_id)
            header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
            return pc2.create_cloud_xyz32(header, [])

        range_count = initial_count
        if self.min_range is not None or self.max_range is not None:
            ranges = np.linalg.norm(points, axis=1)
            if self.min_range is not None:
                points = points[ranges >= float(self.min_range)]
            if self.max_range is not None:
                points = points[ranges <= float(self.max_range)]
            range_count = points.shape[0]

        if points.size == 0:
            if self.filter_log:
                rospy.logwarn_throttle(
                    self.log_interval,
                    "Pointcloud filtered out by range (initial=%d, min=%.2f, max=%.2f)",
                    initial_count,
                    float(self.min_range) if self.min_range is not None else -1.0,
                    float(self.max_range) if self.max_range is not None else -1.0)
            header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
            return pc2.create_cloud_xyz32(header, [])

        z_count = points.shape[0]
        if self.min_z is not None:
            points = points[points[:, 2] >= float(self.min_z)]
        if self.max_z is not None:
            points = points[points[:, 2] <= float(self.max_z)]
        z_count = points.shape[0]

        if points.size == 0:
            if self.filter_log:
                rospy.logwarn_throttle(
                    self.log_interval,
                    "Pointcloud filtered out by z (initial=%d, range=%d, min_z=%.2f, max_z=%.2f)",
                    initial_count,
                    range_count,
                    float(self.min_z) if self.min_z is not None else -1.0,
                    float(self.max_z) if self.max_z is not None else -1.0)
            header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
            return pc2.create_cloud_xyz32(header, [])

        sample_count = points.shape[0]
        if self.max_points and points.shape[0] > int(self.max_points):
            idx = np.random.choice(points.shape[0], int(self.max_points), replace=False)
            points = points[idx]
            sample_count = points.shape[0]

        voxel_count = points.shape[0]
        if self.voxel_leaf_size and float(self.voxel_leaf_size) > 0.0:
            leaf = float(self.voxel_leaf_size)
            voxel_idx = np.floor(points / leaf).astype(np.int32)
            _, unique_indices = np.unique(voxel_idx, axis=0, return_index=True)
            points = points[unique_indices]
            voxel_count = points.shape[0]

        if points.size == 0 and self.filter_log:
            rospy.logwarn_throttle(
                self.log_interval,
                "Pointcloud empty after filtering (initial=%d, range=%d, z=%d, sample=%d, voxel=%d, leaf=%.3f)",
                initial_count,
                range_count,
                z_count,
                sample_count,
                voxel_count,
                float(self.voxel_leaf_size) if self.voxel_leaf_size is not None else -1.0)

        header = Header(frame_id=self.target_frame, stamp=cloud_msg.header.stamp)
        return pc2.create_cloud_xyz32(header, points.tolist())


if __name__ == '__main__':
    rospy.init_node('pointcloud_to_world')
    PointCloudToWorld()
    rospy.spin()

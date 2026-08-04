/**
 * depth_camera_processor.cpp
 * Processes depth camera point cloud for Air-FAR planner:
 * 1. Subsamples the organized depth image before doing expensive work
 * 2. Removes invalid/out-of-range returns and adds Air-FAR obstacle intensity
 * 3. Transforms at the cloud acquisition timestamp into the world frame
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <eigen3/Eigen/Geometry>

class DepthCameraProcessor
{
public:
    DepthCameraProcessor()
        : tf_buffer_(), tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param("input_topic", input_topic_, std::string("/camera/depth/points"));
        pnh.param("output_topic", output_topic_, std::string("/scan_cloud"));
        pnh.param("target_frame", target_frame_, std::string("world"));
        pnh.param("source_frame", source_frame_, std::string("camera_link"));
        pnh.param("intensity_value", intensity_value_, 255.0f);
        pnh.param("max_points", max_points_, 20000);
        pnh.param("pixel_stride", pixel_stride_, 1);
        pnh.param("min_range", min_range_, 0.20);
        pnh.param("max_range", max_range_, 25.0);
        pnh.param("tf_timeout", tf_timeout_, 0.10);

        max_points_ = std::max(1, max_points_);
        pixel_stride_ = std::max(1, pixel_stride_);
        if (min_range_ < 0.0 || max_range_ <= min_range_)
        {
            ROS_FATAL("depth_camera_processor: require 0 <= min_range < max_range");
            ros::shutdown();
            return;
        }

        sub_ = nh.subscribe(
            input_topic_, 1, &DepthCameraProcessor::callback, this,
            ros::TransportHints().tcpNoDelay());
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

        ROS_INFO("depth_camera_processor: %s (expected source=%s) -> %s "
                 "(target=%s), max_points=%d, range=[%.2f, %.2f] m, intensity=%.1f",
                 input_topic_.c_str(), source_frame_.c_str(),
                 output_topic_.c_str(), target_frame_.c_str(), max_points_,
                 min_range_, max_range_, intensity_value_);
    }

private:
    static bool fieldOffset(const sensor_msgs::PointCloud2& cloud,
                            const std::string& name,
                            uint32_t* offset)
    {
        for (const sensor_msgs::PointField& field : cloud.fields)
        {
            if (field.name == name &&
                field.datatype == sensor_msgs::PointField::FLOAT32 &&
                field.count == 1)
            {
                *offset = field.offset;
                return true;
            }
        }
        return false;
    }

    static float readFloat(const uint8_t* data, uint32_t offset)
    {
        float value;
        std::memcpy(&value, data + offset, sizeof(value));
        return value;
    }

    sensor_msgs::PointCloud2 makeCloud(
        const std_msgs::Header& header,
        const std::vector<float>& xyzi) const
    {
        sensor_msgs::PointCloud2 output;
        output.header = header;
        output.header.frame_id = target_frame_;
        output.height = 1;
        output.width = static_cast<uint32_t>(xyzi.size() / 4);
        output.fields.resize(4);
        const char* names[] = {"x", "y", "z", "intensity"};
        for (uint32_t i = 0; i < 4; ++i)
        {
            output.fields[i].name = names[i];
            output.fields[i].offset = i * sizeof(float);
            output.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
            output.fields[i].count = 1;
        }
        output.is_bigendian = false;
        output.point_step = 4 * sizeof(float);
        output.row_step = output.point_step * output.width;
        output.is_dense = true;
        output.data.resize(xyzi.size() * sizeof(float));
        if (!xyzi.empty())
        {
            std::memcpy(output.data.data(), xyzi.data(), output.data.size());
        }
        return output;
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ROS_INFO_ONCE("DCBP: received %ux%u cloud in frame '%s'",
                      msg->width, msg->height, msg->header.frame_id.c_str());

        if (msg->is_bigendian)
        {
            ROS_ERROR_THROTTLE(1.0,
                               "depth_camera_processor: big-endian clouds are unsupported");
            return;
        }

        uint32_t x_offset = 0;
        uint32_t y_offset = 0;
        uint32_t z_offset = 0;
        if (!fieldOffset(*msg, "x", &x_offset) ||
            !fieldOffset(*msg, "y", &y_offset) ||
            !fieldOffset(*msg, "z", &z_offset))
        {
            ROS_ERROR_THROTTLE(
                1.0, "depth_camera_processor: cloud requires FLOAT32 x/y/z fields");
            return;
        }
        const uint32_t largest_offset =
            std::max(x_offset, std::max(y_offset, z_offset));
        const size_t required_data_size =
            static_cast<size_t>(msg->row_step) * msg->height;
        if (largest_offset + sizeof(float) > msg->point_step ||
            msg->row_step < msg->point_step * msg->width ||
            msg->data.size() < required_data_size)
        {
            ROS_ERROR_THROTTLE(
                1.0, "depth_camera_processor: malformed PointCloud2 layout");
            return;
        }

        const std::string source_frame =
            msg->header.frame_id.empty() ? source_frame_ : msg->header.frame_id;
        const ros::Time stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        if (source_frame != target_frame_)
        {
            try
            {
                const geometry_msgs::TransformStamped tf =
                    tf_buffer_.lookupTransform(
                        target_frame_, source_frame, stamp,
                        ros::Duration(tf_timeout_));
                const Eigen::Quaterniond rotation(
                    tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z);
                transform = Eigen::Translation3d(
                                tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z) *
                            rotation.normalized();
            }
            catch (const tf2::TransformException& exception)
            {
                // Publishing camera-frame coordinates under /scan_cloud would
                // silently corrupt both collision checking and terrain mapping.
                ROS_WARN_THROTTLE(
                    1.0, "depth_camera_processor: dropping cloud; TF %s -> %s "
                         "at %.6f unavailable: %s",
                    source_frame.c_str(), target_frame_.c_str(), stamp.toSec(),
                    exception.what());
                return;
            }
        }

        const size_t input_points =
            static_cast<size_t>(msg->width) * msg->height;
        int stride = pixel_stride_;
        if (input_points > static_cast<size_t>(max_points_))
        {
            const double ratio =
                static_cast<double>(input_points) / max_points_;
            stride = std::max(
                stride, static_cast<int>(std::ceil(std::sqrt(ratio))));
        }

        std::vector<float> xyzi;
        xyzi.reserve(static_cast<size_t>(max_points_) * 4);
        const double min_range_sq = min_range_ * min_range_;
        const double max_range_sq = max_range_ * max_range_;

        bool full = false;
        for (uint32_t row = 0; row < msg->height && !full;
             row += static_cast<uint32_t>(stride))
        {
            for (uint32_t col = 0; col < msg->width;
                 col += static_cast<uint32_t>(stride))
            {
                const size_t byte_offset =
                    static_cast<size_t>(row) * msg->row_step +
                    static_cast<size_t>(col) * msg->point_step;
                const uint8_t* point = msg->data.data() + byte_offset;
                const float x = readFloat(point, x_offset);
                const float y = readFloat(point, y_offset);
                const float z = readFloat(point, z_offset);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    continue;

                const double range_sq =
                    static_cast<double>(x) * x +
                    static_cast<double>(y) * y +
                    static_cast<double>(z) * z;
                if (range_sq < min_range_sq || range_sq > max_range_sq)
                    continue;

                const Eigen::Vector3d transformed =
                    transform * Eigen::Vector3d(x, y, z);
                xyzi.push_back(static_cast<float>(transformed.x()));
                xyzi.push_back(static_cast<float>(transformed.y()));
                xyzi.push_back(static_cast<float>(transformed.z()));
                xyzi.push_back(intensity_value_);
                if (xyzi.size() / 4 >= static_cast<size_t>(max_points_))
                {
                    full = true;
                    break;
                }
            }
        }

        std_msgs::Header header = msg->header;
        header.stamp = stamp;
        sensor_msgs::PointCloud2 output = makeCloud(header, xyzi);
        pub_.publish(output);
        ROS_INFO_THROTTLE(
            2.0, "depth_camera_processor: published %u/%zu points (stride=%d)",
            output.width, input_points, stride);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    std::string source_frame_;
    float intensity_value_;
    int max_points_;
    int pixel_stride_;
    double min_range_;
    double max_range_;
    double tf_timeout_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_camera_processor");
    DepthCameraProcessor processor;
    ros::spin();
    return 0;
}

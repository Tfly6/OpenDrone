/**
 * depth_camera_processor.cpp
 * Processes depth camera point cloud for Air-FAR planner:
 * 1. Adds intensity field (required by Air-FAR)
 * 2. Transforms from camera frame to world frame
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Geometry>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointXYZI PCLPoint;

class DepthCameraProcessor
{
public:
    DepthCameraProcessor()
        : tf_buffer_(), tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;

        nh.param("input_topic", input_topic_, std::string("/camera/depth/points"));
        nh.param("output_topic", output_topic_, std::string("/scan_cloud"));
        nh.param("target_frame", target_frame_, std::string("world"));
        nh.param("source_frame", source_frame_, std::string("camera_link"));
        nh.param("intensity_value", intensity_value_, 255.0f);

        sub_ = nh.subscribe(input_topic_, 1, &DepthCameraProcessor::callback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

        ROS_INFO("depth_camera_processor: %s (%s) -> %s (%s), intensity=%.1f",
                 input_topic_.c_str(), source_frame_.c_str(),
                 output_topic_.c_str(), target_frame_.c_str(), intensity_value_);

        ros::spin();
    }

private:
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // Step 1: Add intensity field if missing (replaces rgb to keep point_step=32)
        sensor_msgs::PointCloud2::Ptr cloud_with_intensity;
        if (!hasIntensityField(*msg))
        {
            cloud_with_intensity = addIntensity(*msg);
        }
        else
        {
            cloud_with_intensity = sensor_msgs::PointCloud2::Ptr(
                new sensor_msgs::PointCloud2(*msg));
            ROS_INFO("DCBP: copied original, point_step=%d",
                     cloud_with_intensity->point_step);
        }

        // Step 2: Transform to target frame
        if (cloud_with_intensity->header.frame_id == target_frame_)
        {
            cloud_with_intensity->header.stamp = ros::Time::now();
            pub_.publish(cloud_with_intensity);
            return;
        }

        try
        {
            // Use Time(0) to get the latest available transform instead of message timestamp
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame_, cloud_with_intensity->header.frame_id,
                ros::Time(0), ros::Duration(0.5));

            // Manual transform preserving all fields
            sensor_msgs::PointCloud2 output_msg = *cloud_with_intensity;
            output_msg.header.frame_id = target_frame_;
            output_msg.header.stamp = ros::Time::now();

            // Create Eigen transform matrix from quaternion + translation
            Eigen::Quaterniond q(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z);
            Eigen::Translation3d t(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
            Eigen::Affine3d eigen_transform = t * q;

            // Transform each point manually
            uint8_t* output_data = output_msg.data.data();
            const uint8_t* input_data = cloud_with_intensity->data.data();
            size_t num_points = cloud_with_intensity->width * cloud_with_intensity->height;

            for (size_t i = 0; i < num_points; ++i)
            {
                size_t offset = i * cloud_with_intensity->point_step;

                // Read XYZ
                float x = *reinterpret_cast<const float*>(&input_data[offset + 0]);
                float y = *reinterpret_cast<const float*>(&input_data[offset + 4]);
                float z = *reinterpret_cast<const float*>(&input_data[offset + 8]);

                // Transform using Eigen
                Eigen::Vector3d pt(x, y, z);
                Eigen::Vector3d pt_transformed = eigen_transform * pt;

                // Write transformed XYZ back
                *reinterpret_cast<float*>(&output_data[offset + 0]) = pt_transformed.x();
                *reinterpret_cast<float*>(&output_data[offset + 4]) = pt_transformed.y();
                *reinterpret_cast<float*>(&output_data[offset + 8]) = pt_transformed.z();
                // intensity at offset 16 stays unchanged
            }

            pub_.publish(output_msg);
        }
        catch (tf2::TransformException& e)
        {
            // If transform fails, still publish with intensity but in original frame
            cloud_with_intensity->header.stamp = ros::Time::now();
            pub_.publish(cloud_with_intensity);
            ROS_WARN_THROTTLE(1.0, "TF transform failed, publishing in original frame: %s", e.what());
        }
    }

    bool hasIntensityField(const sensor_msgs::PointCloud2& cloud)
    {
        for (const auto& field : cloud.fields)
        {
            if (field.name == "intensity")
                return true;
        }
        return false;
    }

    sensor_msgs::PointCloud2::Ptr addIntensity(const sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2());

        // Copy metadata
        output->header = cloud.header;
        output->height = cloud.height;
        output->width = cloud.width;
        output->is_bigendian = cloud.is_bigendian;
        output->is_dense = cloud.is_dense;

        // Replace rgb field with intensity at the same offset
        // This keeps point_step=32 and makes it compatible with PCL's PointXYZI
        output->fields.clear();
        for (const auto& f : cloud.fields)
        {
            if (f.name == "rgb")
            {
                // Replace rgb with intensity
                sensor_msgs::PointField intensity_field;
                intensity_field.name = "intensity";
                intensity_field.offset = f.offset;  // Same offset as rgb (16)
                intensity_field.datatype = sensor_msgs::PointField::FLOAT32;
                intensity_field.count = 1;
                output->fields.push_back(intensity_field);
            }
            else
            {
                output->fields.push_back(f);
            }
        }

        // point_step stays the same (32 bytes)
        output->point_step = cloud.point_step;
        output->row_step = output->point_step * output->width;

        // Copy data and set intensity value for all points
        const uint8_t* cloud_data = cloud.data.data();
        size_t num_points = cloud.width * cloud.height;
        std::vector<uint8_t> output_data(num_points * output->point_step);

        // Find the intensity field offset (should be where rgb was, offset 16)
        uint32_t intensity_offset = 16;  // rgb was at offset 16

        for (size_t i = 0; i < num_points; ++i)
        {
            // Copy existing point data
            memcpy(&output_data[i * output->point_step],
                   &cloud_data[i * cloud.point_step],
                   cloud.point_step);
            // Set intensity value (overwrite where rgb was)
            float intensity = intensity_value_;
            memcpy(&output_data[i * output->point_step + intensity_offset],
                   &intensity, 4);
        }

        output->data = output_data;
        return output;
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_camera_processor");
    DepthCameraProcessor processor;
    return 0;
}
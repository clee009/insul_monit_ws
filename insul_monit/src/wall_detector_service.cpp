#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_srvs/Trigger.h>

class WallDetectorService
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    std::string wall_x_param_;
    double tolerance_;
    double ransac_distance_threshold_;
    double max_angle_tolerance_;
    int min_points_wall_;

public:
    WallDetectorService() : nh_("~")
    {
        // Load parameters from launch file or set defaults
        nh_.param<std::string>("wall_x_param", wall_x_param_, "/detected_wall_x");
        nh_.param("tolerance", tolerance_, 0.05); // 5 cm default
        nh_.param("ransac_distance_threshold", ransac_distance_threshold_, 0.01); // 1 cm default
        nh_.param("max_angle_tolerance", max_angle_tolerance_, 10.0); // 10 degrees
        nh_.param("min_points_wall", min_points_wall_, 10); // At least 10 points

        service_ = nh_.advertiseService("detect_wall_x", &WallDetectorService::detectWall, this);
        ROS_INFO("Wall detection service ready");
    }

    bool detectWall(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("Service [detect_wall_x] called. Waiting for point cloud.");

        sensor_msgs::PointCloud2ConstPtr input = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/input", ros::Duration(3.0));
        if (!input)
        {
            res.success = false;
            res.message = "No point cloud received!";
            ROS_WARN("No point cloud received within timeout.");
            return false;
        }

        // Convert to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        if (cloud->empty())
        {
            res.success = false;
            res.message = "Empty point cloud!";
            ROS_WARN("Received an empty point cloud.");
            return false;
        }

        ROS_INFO("Received point cloud with %zu points", cloud->size());

        // Find max X points
        double max_x = std::numeric_limits<double>::lowest();
        pcl::PointCloud<pcl::PointXYZ>::Ptr max_x_points(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &point : cloud->points)
        {
            if (point.x > max_x)
                max_x = point.x;
        }

        ROS_INFO("Max X found: %.3f", max_x);

        for (const auto &point : cloud->points)
        {
            if (std::fabs(point.x - max_x) <= tolerance_)
            {
                max_x_points->points.push_back(point);
            }
        }

        ROS_INFO("Number of max X points within tolerance (%.3f): %zu", tolerance_, max_x_points->size());

        if (max_x_points->size() < min_points_wall_)
        {
            res.success = false;
            res.message = "Not enough points to detect a wall.";
            ROS_WARN("Not enough max X points to fit a plane.");
            return false;
        }

        // Fit a plane using RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        seg.setInputCloud(max_x_points);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            res.success = false;
            res.message = "No valid wall detected.";
            ROS_WARN("No valid plane found.");
            return false;
        }

        // Debugging: Print the RANSAC plane coefficients
        ROS_INFO("Plane coefficients: a=%.3f, b=%.3f, c=%.3f, d=%.3f",
                 coefficients->values[0], coefficients->values[1],
                 coefficients->values[2], coefficients->values[3]);

        // Plane equation: ax + by + cz + d = 0
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];

        // Compute angle between plane normal and X-axis
        double normal_magnitude = std::sqrt(a * a + b * b + c * c);
        double angle_with_x_axis = std::acos(std::fabs(a) / normal_magnitude) * (180.0 / M_PI);

        ROS_INFO("Angle between detected plane and X-axis: %.2f degrees", angle_with_x_axis);

        if (angle_with_x_axis > max_angle_tolerance_)
        {
            res.success = false;
            res.message = "Plane is not aligned with X-axis.";
            ROS_WARN("Detected plane is not aligned with X-axis (%.2f°).", angle_with_x_axis);
            return false;
        }

        // Store detected wall X in rosparam
        ros::param::set(wall_x_param_, max_x);
        ROS_INFO("Wall detected at X = %.3f, stored in ROS param: %s", max_x, wall_x_param_.c_str());

        res.success = true;
        res.message = "Wall detected at X = " + std::to_string(max_x);
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_detector_service");
    WallDetectorService detector;
    ros::spin();
    return 0;
}

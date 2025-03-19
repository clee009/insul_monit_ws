#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <limits>

class WallFilter
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    // Parameters
    double max_x_tolerance_;             // Tolerance to select points near the maximum x value
    double ransac_distance_threshold_;   // Threshold for RANSAC plane fitting
    double filtering_distance_threshold_; // Distance from plane below which points are removed
    double max_angle_tolerance_;         // Maximum angle (in degrees) allowed between the plane's normal and the X-axis
    int min_points_wall_;                // Minimum number of points required for a valid wall

public:
    WallFilter() : nh_("~")
    {
        nh_.param("max_x_tolerance", max_x_tolerance_, 0.05);
        nh_.param("ransac_distance_threshold", ransac_distance_threshold_, 0.01);
        nh_.param("filtering_distance_threshold", filtering_distance_threshold_, 0.05);
        nh_.param("max_angle_tolerance", max_angle_tolerance_, 10.0);
        nh_.param("min_points_wall", min_points_wall_, 10);

        cloud_sub_ = nh_.subscribe("/input", 1, &WallFilter::filterCloud, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output", 1);

        ROS_INFO("Wall filter node initialized with parameters:");
        ROS_INFO("  max_x_tolerance: %.3f", max_x_tolerance_);
        ROS_INFO("  ransac_distance_threshold: %.3f", ransac_distance_threshold_);
        ROS_INFO("  filtering_distance_threshold: %.3f", filtering_distance_threshold_);
        ROS_INFO("  max_angle_tolerance: %.3f", max_angle_tolerance_);
        ROS_INFO("  min_points_wall: %d", min_points_wall_);
    }

    void filterCloud(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        // Convert the input message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        if(cloud->empty())
        {
            ROS_WARN("Received empty point cloud, skipping filtering.");
            return;
        }

        // Extract max X points
        double max_x = -std::numeric_limits<double>::max();
        for(const auto &point : cloud->points)
        {
            if(point.x > max_x)
                max_x = point.x;
        }

        // Select points within max_x_tolerance of the max x value
        pcl::PointCloud<pcl::PointXYZ>::Ptr max_x_points(new pcl::PointCloud<pcl::PointXYZ>);
        for(const auto &point : cloud->points)
        {
            if(std::fabs(point.x - max_x) <= max_x_tolerance_)
                max_x_points->points.push_back(point);
        }

        if(max_x_points->points.size() < static_cast<size_t>(min_points_wall_))
        {
            ROS_WARN("Not enough points near max x to detect wall (found %zu, required %d). Publishing original cloud.", 
                     max_x_points->points.size(), min_points_wall_);
            cloud_pub_.publish(*input);
            return;
        }

        // Fit a plane using RANSAC to the selected max x points
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        seg.setInputCloud(max_x_points);
        seg.segment(*inliers, *coefficients);

        if(inliers->indices.empty())
        {
            ROS_WARN("RANSAC failed to detect a valid plane. Publishing original cloud.");
            cloud_pub_.publish(*input);
            return;
        }

        // Check the alignment of the plane with the X-axis.
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];
        double normal_magnitude = std::sqrt(a*a + b*b + c*c);
        double angle_with_x = std::acos(std::fabs(a) / normal_magnitude) * 180.0 / M_PI;
        if(angle_with_x > max_angle_tolerance_)
        {
            ROS_WARN("Detected plane is not well aligned with the X-axis (angle: %.2f° > %.2f°). Publishing original cloud.", 
                     angle_with_x, max_angle_tolerance_);
            cloud_pub_.publish(*input);
            return;
        }

        // ROS_INFO("Detected wall plane: a=%.3f, b=%.3f, c=%.3f, d=%.3f (angle with X-axis: %.2f°)",
        //          a, b, c, d, angle_with_x);

        // Filter out points close to the detected plane
        pcl::PointIndices::Ptr points_to_remove(new pcl::PointIndices);
        for(size_t i = 0; i < cloud->points.size(); ++i)
        {
            double distance = std::fabs(a * cloud->points[i].x +
                                        b * cloud->points[i].y +
                                        c * cloud->points[i].z + d) / normal_magnitude;
            if(distance <= filtering_distance_threshold_)
            {
                points_to_remove->indices.push_back(i);
            }
        }

        // ROS_INFO("Filtering: Removing %zu points within %.3f m of the wall plane.", 
        //          points_to_remove->indices.size(), filtering_distance_threshold_);

        // Remove points near the wall plane from the original cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(points_to_remove);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = input->header;
        cloud_pub_.publish(output);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_filter");
    WallFilter wf;
    ros::spin();
    return 0;
}

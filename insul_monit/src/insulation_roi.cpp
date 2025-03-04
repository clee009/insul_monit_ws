#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <deque>

ros::Publisher marker_pub;
std::deque<std::tuple<float, float, float, float>> bbox_history;
int N = 5;  // Default moving average window size

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert ROS point cloud message to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->points.empty())
    {
        // ROS_WARN("Point cloud is empty!");
        return;
    }

    // Initialize bounding box min/max values
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    // Find the min and max values in X and Y
    for (const auto& point : cloud->points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;

        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
    }

    // Add the current bounding box to the history queue
    bbox_history.push_back(std::make_tuple(min_x, min_y, max_x, max_y));

    // Maintain a fixed size for the queue
    if (bbox_history.size() > N)
    {
        bbox_history.pop_front(); // Remove the oldest entry
    }

    // Compute the moving average of bounding box coordinates
    float avg_min_x = 0, avg_min_y = 0, avg_max_x = 0, avg_max_y = 0;
    for (const auto& bbox : bbox_history)
    {
        avg_min_x += std::get<0>(bbox);
        avg_min_y += std::get<1>(bbox);
        avg_max_x += std::get<2>(bbox);
        avg_max_y += std::get<3>(bbox);
    }
    int size = bbox_history.size();
    avg_min_x /= size;
    avg_min_y /= size;
    avg_max_x /= size;
    avg_max_y /= size;

    // Create a visualization marker
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = cloud_msg->header.frame_id; // Use same frame as the point cloud
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.ns = "bounding_box";
    bbox_marker.id = 0;
    bbox_marker.type = visualization_msgs::Marker::LINE_STRIP;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    // Set the marker properties
    bbox_marker.scale.x = 0.02; // Line width
    bbox_marker.color.r = 1.0;
    bbox_marker.color.g = 0.0;
    bbox_marker.color.b = 0.0;
    bbox_marker.color.a = 1.0;
    
    // Define the smoothed bounding box corners in XY plane
    geometry_msgs::Point p1, p2, p3, p4, p5;
    p1.x = avg_min_x; p1.y = avg_min_y; p1.z = 0.0;
    p2.x = avg_max_x; p2.y = avg_min_y; p2.z = 0.0;
    p3.x = avg_max_x; p3.y = avg_max_y; p3.z = 0.0;
    p4.x = avg_min_x; p4.y = avg_max_y; p4.z = 0.0;
    p5 = p1; // Close the loop

    // Add points to the marker
    bbox_marker.points.push_back(p1);
    bbox_marker.points.push_back(p2);
    bbox_marker.points.push_back(p3);
    bbox_marker.points.push_back(p4);
    bbox_marker.points.push_back(p5);

    // Publish the marker
    marker_pub.publish(bbox_marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "insulation_roi");
    ros::NodeHandle nh("~"); // Use private namespace

    // Get the moving average window size from the parameter server
    nh.param("N", N, 5); // Default value = 5

    ROS_INFO("Using N = %d for moving average", N);

    // Subscribe to point cloud topic
    ros::Subscriber pcl_sub = nh.subscribe("/voxel_grid/roi/output", 1, pointCloudCallback);

    // Publisher for marker visualization
    marker_pub = nh.advertise<visualization_msgs::Marker>("insulation_roi", 1);

    ros::spin();
}

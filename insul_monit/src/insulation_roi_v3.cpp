#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <deque>
#include <insul_monit/CentroidInfo.h>
#include <std_msgs/Header.h>
#include <pcl/filters/passthrough.h>

// Structure to store computed data for each message
struct MessageData {
    double centroid_x;
    double centroid_y;
    double std_major;    // Standard deviation along the major axis (sqrt(eigenvalue))
    double std_minor;    // Standard deviation along the minor axis
    double angle;    // Orientation of the major axis (radians)
    double min_x;
    double min_y;
    double max_x;
    double max_y;
    double density;
};

// Global buffer for moving average and parameters
std::deque<MessageData> data_buffer;
int window_size = 10;         // Moving average window size (set via parameter)
double centroid_bias = 0.0;   // Bias fraction (0 = no bias, 1 = centroid becomes max_x)
int cloud_size = 10;          // Min point cloud size threshold
double timeout = 3.0;         // Timeout threshold in sec to clear data
// Note: The bias is applied as: 
//    biased_centroid_x = avg_centroid_x + centroid_bias * (avg_max_x - avg_centroid_x)

ros::Publisher marker_pub;
ros::Publisher centroid_info_pub;

// Track the last valid centroid time
static ros::Time last_valid_time;


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Clear moving average buffer if time between valid point clouds above threshold
    if ((ros::Time::now() - last_valid_time).toSec() > timeout) {
        // ROS_WARN("Clearing centroid buffer due to timeout.");
        data_buffer.clear();
    }

    // Convert the input message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);    
    if (cloud->empty()) {
        // ROS_WARN("Cloud empty");

        // Publish CentroidInfo message
        insul_monit::CentroidInfo centroid_info_msg;
        centroid_info_msg.header.stamp = ros::Time::now();
        centroid_info_msg.header.frame_id = "cavity";
        centroid_info_msg.biased_centroid.x = NAN;
        centroid_info_msg.biased_centroid.y = NAN;
        centroid_info_msg.biased_centroid.z = NAN;
        centroid_info_msg.x_offset = NAN;
        centroid_info_msg.density = NAN;
        centroid_info_msg.valid = false;
        centroid_info_pub.publish(centroid_info_msg);
        
        return;
    } else if (cloud->size() < cloud_size) {
        // ROS_WARN("Cloud below minimum size");

        // Publish CentroidInfo message
        insul_monit::CentroidInfo centroid_info_msg;
        centroid_info_msg.header.stamp = ros::Time::now();
        centroid_info_msg.header.frame_id = "cavity";
        centroid_info_msg.biased_centroid.x = NAN;
        centroid_info_msg.biased_centroid.y = NAN;
        centroid_info_msg.biased_centroid.z = NAN;
        centroid_info_msg.x_offset = NAN;
        centroid_info_msg.density = NAN;
        centroid_info_msg.valid = false;
        centroid_info_pub.publish(centroid_info_msg);
        
        return;
    } else {
        last_valid_time = ros::Time::now();
    }

    // pcl::PointXYZ minp, maxp;
    // pcl::getMinMax3D (*cloud, minp, maxp);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(0.0, maxp.x - 0.05);
    // pass.filter(*cloud);

    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-0.5, -0.5);
    // pass.filter(*cloud);

    
    // Compute 2D centroid (using x and y)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    
    // Compute covariance matrix in the XY plane
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance_matrix);
    
    // Extract the XY covariance
    Eigen::Matrix2f covariance_xy;
    covariance_xy << covariance_matrix(0,0), covariance_matrix(0,1),
                     covariance_matrix(1,0), covariance_matrix(1,1);
    
    // Compute eigenvalues and eigenvectors for the standard deviation ellipse
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance_xy);
    Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
    Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
    
    double std_major = sqrt(eigenvalues[1]);  // Standard deviation along the major axis
    double std_minor = sqrt(eigenvalues[0]);  // Standard deviation along the minor axis
    double angle = atan2(eigenvectors(1, 1), eigenvectors(0, 1)); // Orientation (radians)
    
    // Compute 2D bounding box from the cloud
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    
    // Compute density as number of points divided by bounding box area
    double area = (maxPt.x - minPt.x) * (maxPt.y - minPt.y);
    double density = cloud->size() / area;
    
    // Store computed values in a MessageData struct
    MessageData current;
    current.centroid_x = centroid[0];
    current.centroid_y = centroid[1];
    current.std_major = std_major;
    current.std_minor = std_minor;
    current.angle = angle;
    current.min_x = minPt.x;
    current.min_y = minPt.y;
    current.max_x = maxPt.x;
    current.max_y = maxPt.y;
    current.density = density;

    // ROS_INFO("area: %f", area);
    // ROS_INFO("density: %f",  current.density);
    
    // Add current message data to the moving average buffer
    data_buffer.push_back(current);
    if (data_buffer.size() > static_cast<size_t>(window_size))
        data_buffer.pop_front();
    
    // Compute moving averages over the buffer
    double sum_centroid_x = 0.0, sum_centroid_y = 0.0;
    double sum_std_major = 0.0, sum_std_minor = 0.0;
    double sum_sin = 0.0, sum_cos = 0.0;
    double sum_min_x = 0.0, sum_min_y = 0.0;
    double sum_max_x = 0.0, sum_max_y = 0.0;
    double sum_density = 0.0;
    
    for (const auto &d : data_buffer) {
        sum_centroid_x += d.centroid_x;
        sum_centroid_y += d.centroid_y;
        sum_std_major += d.std_major;
        sum_std_minor += d.std_minor;
        sum_sin += sin(d.angle);
        sum_cos += cos(d.angle);
        sum_min_x += d.min_x;
        sum_min_y += d.min_y;
        sum_max_x += d.max_x;
        sum_max_y += d.max_y;
        sum_density += d.density;
    }
    
    size_t n = data_buffer.size();
    double avg_centroid_x = sum_centroid_x / n;
    double avg_centroid_y = sum_centroid_y / n;
    double avg_std_major = sum_std_major / n;
    double avg_std_minor = sum_std_minor / n;
    double avg_angle = atan2(sum_sin / n, sum_cos / n);
    double avg_min_x = sum_min_x / n;
    double avg_min_y = sum_min_y / n;
    double avg_max_x = sum_max_x / n;
    double avg_max_y = sum_max_y / n;
    double avg_density = sum_density / n;

    // ROS_INFO("buffer size: %ld", n);

    if (n < window_size) {
        // ROS_WARN("Data buffer less than moving average window size");
        return;
    }
    
    // Compute a biased x value for the centroid. The bias is defined as a fraction of the 
    // x-distance from the centroid to the bounding box's maximum x value.
    double biased_centroid_x = avg_centroid_x + centroid_bias * (avg_max_x - avg_centroid_x);
    
    ros::Time now = ros::Time::now();
    
    // Publish the (biased) centroid marker
    visualization_msgs::Marker centroid_marker;
    centroid_marker.header.frame_id = input->header.frame_id;
    centroid_marker.header.stamp = now;
    centroid_marker.ns = "centroid";
    centroid_marker.id = 0;
    centroid_marker.type = visualization_msgs::Marker::SPHERE;
    centroid_marker.action = visualization_msgs::Marker::ADD;
    centroid_marker.pose.position.x = biased_centroid_x;
    centroid_marker.pose.position.y = avg_centroid_y;
    centroid_marker.pose.position.z = 0;
    centroid_marker.scale.x = 0.1;
    centroid_marker.scale.y = 0.1;
    centroid_marker.scale.z = 0.1;
    centroid_marker.color.r = 1.0;
    centroid_marker.color.g = 0.0;
    centroid_marker.color.b = 0.0;
    centroid_marker.color.a = 1.0;
    centroid_marker.lifetime = ros::Duration(0.5);
    marker_pub.publish(centroid_marker);
    
    // Publish the ellipse marker using the biased centroid as the center
    visualization_msgs::Marker ellipse_marker;
    ellipse_marker.header.frame_id = input->header.frame_id;
    ellipse_marker.header.stamp = now;
    ellipse_marker.ns = "ellipse";
    ellipse_marker.id = 1;
    ellipse_marker.type = visualization_msgs::Marker::CYLINDER;
    ellipse_marker.action = visualization_msgs::Marker::ADD;
    ellipse_marker.pose.position.x = biased_centroid_x;
    ellipse_marker.pose.position.y = avg_centroid_y;
    ellipse_marker.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, avg_angle);
    ellipse_marker.pose.orientation.x = q.x();
    ellipse_marker.pose.orientation.y = q.y();
    ellipse_marker.pose.orientation.z = q.z();
    ellipse_marker.pose.orientation.w = q.w();
    ellipse_marker.scale.x = avg_std_major * 2;  // Diameter along major axis
    ellipse_marker.scale.y = avg_std_minor * 2;  // Diameter along minor axis
    ellipse_marker.scale.z = 0.01;
    ellipse_marker.color.r = 0.0;
    ellipse_marker.color.g = 1.0;
    ellipse_marker.color.b = 0.0;
    ellipse_marker.color.a = 0.5;
    ellipse_marker.lifetime = ros::Duration(0.5);
    marker_pub.publish(ellipse_marker);
    
    // Publish the bounding box marker
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = input->header.frame_id;
    bbox_marker.header.stamp = now;
    bbox_marker.ns = "bounding_box";
    bbox_marker.id = 2;
    bbox_marker.type = visualization_msgs::Marker::LINE_STRIP;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    bbox_marker.scale.x = 0.02;
    bbox_marker.color.r = 0.0;
    bbox_marker.color.g = 0.0;
    bbox_marker.color.b = 1.0;
    bbox_marker.color.a = 1.0;
    bbox_marker.lifetime = ros::Duration(0.5);
    
    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = avg_min_x; p1.y = avg_min_y;
    p2.x = avg_max_x; p2.y = avg_min_y;
    p3.x = avg_max_x; p3.y = avg_max_y;
    p4.x = avg_min_x; p4.y = avg_max_y;
    bbox_marker.points.push_back(p1);
    bbox_marker.points.push_back(p2);
    bbox_marker.points.push_back(p3);
    bbox_marker.points.push_back(p4);
    bbox_marker.points.push_back(p1);
    marker_pub.publish(bbox_marker);

    // Publish CentroidInfo message
    insul_monit::CentroidInfo centroid_info_msg;
    centroid_info_msg.header.stamp = ros::Time::now();
    centroid_info_msg.header.frame_id = "cavity";
    centroid_info_msg.biased_centroid.x = biased_centroid_x;
    centroid_info_msg.biased_centroid.y = avg_centroid_y;
    centroid_info_msg.biased_centroid.z = 0;
    centroid_info_msg.x_offset = avg_max_x - biased_centroid_x;
    centroid_info_msg.density = avg_density;
    centroid_info_msg.valid = true;
    centroid_info_pub.publish(centroid_info_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "insulation_roi_node");
    ros::NodeHandle nh("~");
    
    // Get parameters from the ROS parameter server
    nh.param("moving_average_window", window_size, 10);
    nh.param("centroid_bias", centroid_bias, 0.5);
    nh.param("cloud_size", cloud_size, 10);
    nh.param("timeout", timeout, 3.0);
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("/output", 10);
    centroid_info_pub = nh.advertise<insul_monit::CentroidInfo>("/centroid_info", 10);

    last_valid_time = ros::Time::now();
    ros::Subscriber sub = nh.subscribe("/input", 1, cloudCallback);
    
    ros::spin();
    return 0;
}

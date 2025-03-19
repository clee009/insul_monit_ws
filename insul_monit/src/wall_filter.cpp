#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

class WallFilter
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    std::string wall_x_param_;
    double tolerance_;

public:
    WallFilter() : nh_("~")
    {
        // Load parameters from launch file or set defaults
        nh_.param<std::string>("wall_x_param", wall_x_param_, "/detected_wall_x");
        nh_.param("tolerance", tolerance_, 0.05); // Default: 5 cm tolerance

        // Subscribe to point cloud topic
        cloud_sub_ = nh_.subscribe("/input", 1, &WallFilter::filterCloud, this);

        // Advertise the filtered point cloud topic
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output", 1);

        ROS_INFO("Wall filter node initialized.");
        ROS_INFO("Using wall X parameter: %s", wall_x_param_.c_str());
        ROS_INFO("Filtering tolerance set to: %.3f", tolerance_);
    }

    void filterCloud(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        double wall_x;
        if (!ros::param::get(wall_x_param_, wall_x))
        {
            ROS_WARN_THROTTLE(5, "Wall X parameter [%s] not set. Skipping filtering.", wall_x_param_.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (std::fabs(cloud->points[i].x - wall_x) <= tolerance_)
                inliers->indices.push_back(i);
        }

        // ROS_INFO("Filtering %zu points near wall X = %.3f within tolerance %.3f", 
        //          inliers->indices.size(), wall_x, tolerance_);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = input->header;
        cloud_pub_.publish(output);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_filter");
    WallFilter filter;
    ros::spin();
    return 0;
}

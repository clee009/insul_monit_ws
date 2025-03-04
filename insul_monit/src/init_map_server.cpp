#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

class InitMapService
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher pointcloud_pub_;
    ros::ServiceServer service_;
    sensor_msgs::PointCloud2 latest_pointcloud_;
    bool has_pointcloud_;

public:
    InitMapService()
    {
        // Get parameters
        std::string pointcloud_topic, output_topic;

        // Subscribe to the PointCloud2 topic
        pointcloud_sub_ = nh_.subscribe("/input", 1, &InitMapService::pointcloudCallback, this);

        // Publisher for on-demand point cloud
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output", 1);

        // Create the service
        service_ = nh_.advertiseService("/init_map", &InitMapService::handleServiceRequest, this);

        has_pointcloud_ = false;

        ROS_INFO("PointCloud service server is running...");
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        latest_pointcloud_ = *msg;
        has_pointcloud_ = true;
    }

    bool handleServiceRequest(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if (has_pointcloud_)
        {
            pointcloud_pub_.publish(latest_pointcloud_);
            ROS_INFO("Published latest point cloud.");
            res.success = true;
            res.message = "Point cloud published";
        }
        else
        {
            ROS_WARN("No point cloud received yet.");
            res.success = false;
            res.message = "No point cloud available yet";
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_service_server");
    InitMapService pcs;
    ros::spin();
    return 0;
}

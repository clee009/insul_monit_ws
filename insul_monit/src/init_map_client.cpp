#include <ros/ros.h>
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "init_map_client");
    ros::NodeHandle nh;

    // Wait for service to be available
    ros::service::waitForService("/get_pointcloud");

    // Create a service client
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/get_pointcloud");

    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("Service Response: %s", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }

    return 0;
}

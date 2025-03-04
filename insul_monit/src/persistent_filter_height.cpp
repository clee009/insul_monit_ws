#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class HeightClusteringFilter {
public:
    HeightClusteringFilter(ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("N", N_, 10);
        nh_.param("epsilon", epsilon_, 0.05f);

        cloud_sub_ = nh_.subscribe("/input", 1, &HeightClusteringFilter::cloudCallback, this);
        cloud_pub_ = nh_.advertise<PointCloudT>("/output", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    
    int N_; // Number of nearest neighbors to consider
    float epsilon_; // Height threshold for filtering

    void cloudCallback(const PointCloudT::ConstPtr &cloud) {
        if (cloud->empty()) return;

        PointCloudT::Ptr filtered_cloud(new PointCloudT);
        filtered_cloud->header = cloud->header;

        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(cloud);

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            PointT search_point = cloud->points[i];

            std::vector<int> point_indices;
            std::vector<float> point_distances;

            // Find N nearest neighbors in the x, y plane (ignoring z)
            if (kdtree.nearestKSearch(search_point, N_, point_indices, point_distances) > 0) {
                float min_z = std::numeric_limits<float>::max();

                // Compute minimum z-height in the neighborhood
                for (int idx : point_indices) {
                    min_z = std::min(min_z, cloud->points[idx].z);
                }

                // Keep points only if their z is within epsilon of min_z
                if (search_point.z <= min_z + epsilon_) {
                    filtered_cloud->points.push_back(search_point);
                }
            }
        }

        cloud_pub_.publish(filtered_cloud);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "height_clustering_filter");
    ros::NodeHandle nh("~");
    HeightClusteringFilter filter(nh);
    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>
#include <deque>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash {
    std::size_t operator()(const VoxelKey &k) const {
        return (std::hash<int>()(k.x) * 73856093) + (std::hash<int>()(k.y) * 19349663) 
          + (std::hash<int>()(k.z) * 83492791);
    }
};

class PersistentVoxelFilter {
public:
    PersistentVoxelFilter(ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("N", N_, 3);
        nh_.param("voxel_size", voxel_size_, 0.05f);

        cloud_sub_ = nh_.subscribe("/input", 1, &PersistentVoxelFilter::cloudCallback, this);
        cloud_pub_ = nh_.advertise<PointCloudT>("/output", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher neg_cloud_pub_;
    std::unordered_map<VoxelKey, int, VoxelHash> voxel_occupancy_;
    std::deque<std::unordered_map<VoxelKey, bool, VoxelHash>> frame_history_;
    int N_;  // Number of frames required for a voxel to be persistent
    float voxel_size_; // Voxel size in meters

    void cloudCallback(const PointCloudT::ConstPtr &cloud) {
        std::unordered_map<VoxelKey, bool, VoxelHash> current_frame_voxels;
        
        for (const auto &point : cloud->points) {
            VoxelKey voxel = { static_cast<int>(point.x / voxel_size_),
                               static_cast<int>(point.y / voxel_size_),
                               static_cast<int>(point.z / voxel_size_)};
            current_frame_voxels[voxel] = true;
        }

        frame_history_.push_back(current_frame_voxels);
        if (frame_history_.size() > N_) {
            auto &oldest_frame = frame_history_.front();
            for (const auto &kv : oldest_frame) {
                if (voxel_occupancy_[kv.first] > 0) {
                    voxel_occupancy_[kv.first]--;
                }
            }
            frame_history_.pop_front();
        }

        for (const auto &kv : current_frame_voxels) {
            voxel_occupancy_[kv.first]++;
        }

        PointCloudT::Ptr filtered_cloud(new PointCloudT);
        filtered_cloud->header = cloud->header;
        for (const auto &point : cloud->points) {
            VoxelKey voxel = { static_cast<int>(point.x / voxel_size_),
                               static_cast<int>(point.y / voxel_size_),
                               static_cast<int>(point.z / voxel_size_)};
            if (voxel_occupancy_[voxel] >= N_) {
                filtered_cloud->points.push_back(point);
            }
        }

        cloud_pub_.publish(filtered_cloud);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "persistent_filter");
    ros::NodeHandle nh("~");
    PersistentVoxelFilter filter(nh);
    ros::spin();
    return 0;
}

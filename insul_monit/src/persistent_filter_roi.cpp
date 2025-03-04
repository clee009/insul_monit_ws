#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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
        return (std::hash<int>()(k.x) * 73856093) + 
               (std::hash<int>()(k.y) * 19349663) + 
               (std::hash<int>()(k.z) * 83492791);
    }
};

class PersistentVoxelFilter {
public:
    PersistentVoxelFilter(ros::NodeHandle &nh) : nh_(nh) {
        nh_.param("N", N_, 3);
        nh_.param("voxel_size", voxel_size_, 0.05f);

        cloud_sub_ = nh_.subscribe("/voxel_grid/depth/output", 1, &PersistentVoxelFilter::cloudCallback, this);
        cloud_pub_ = nh_.advertise<PointCloudT>("/filtered_cloud", 1);   // Persistent (grounded) particles
        neg_cloud_pub_ = nh_.advertise<PointCloudT>("/airborne_particles", 1);  // Airborne particles
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher neg_cloud_pub_;
    std::unordered_map<VoxelKey, int, VoxelHash> voxel_occupancy_;
    std::deque<std::unordered_map<VoxelKey, bool, VoxelHash>> frame_history_;
    int N_;  // Number of frames required for persistence
    float voxel_size_; // Voxel size in meters

    void cloudCallback(const PointCloudT::ConstPtr &cloud) {
        std::unordered_map<VoxelKey, bool, VoxelHash> current_frame_voxels;
        
        // Identify occupied voxels in the current frame
        for (const auto &point : cloud->points) {
            VoxelKey voxel = { static_cast<int>(point.x / voxel_size_),
                               static_cast<int>(point.y / voxel_size_),
                               static_cast<int>(point.z / voxel_size_)};
            current_frame_voxels[voxel] = true;
        }

        // Manage frame history
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

        // Update voxel occupancy count
        for (const auto &kv : current_frame_voxels) {
            voxel_occupancy_[kv.first]++;
        }

        // Create point clouds for persistent and airborne particles
        PointCloudT::Ptr filtered_cloud(new PointCloudT);
        PointCloudT::Ptr airborne_cloud(new PointCloudT);
        filtered_cloud->header = cloud->header;
        airborne_cloud->header = cloud->header;

        for (const auto &point : cloud->points) {
            VoxelKey voxel = { static_cast<int>(point.x / voxel_size_),
                               static_cast<int>(point.y / voxel_size_),
                               static_cast<int>(point.z / voxel_size_)};

            if (voxel_occupancy_[voxel] >= N_) {
                // Persistent (grounded) particles
                filtered_cloud->points.push_back(point);
            } else {
                // Airborne particles
                airborne_cloud->points.push_back(point);
            }
        }

        // Publish both clouds
        cloud_pub_.publish(filtered_cloud);
        neg_cloud_pub_.publish(airborne_cloud);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "persistent_filter");
    ros::NodeHandle nh("~");
    PersistentVoxelFilter filter(nh);
    ros::spin();
    return 0;
}

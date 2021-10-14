#ifndef  VIRTUAL_LOCAL_MAP_H_
#define  VIRTUAL_LOCAL_MAP_H_
#include <list>
#include <mutex>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <png.h>
#include <actuator/actuator.h>
#include <diagnosis_msgs/SlamInfo.h>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_datatypes.h"

class VirtualLocalMap {
  class GlobalMapConfig {
   public:
    GlobalMapConfig() {}
    GlobalMapConfig(double w, double h,
                    double resolution, double orig_x, double orig_y) {
      width_ = w;
      height_ = h;
      resolution_ = resolution;
      origin_x_ = orig_x;
      origin_y_ = orig_y;
    }
    double width_;
    double height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
  };
 public:
  VirtualLocalMap();
  ~VirtualLocalMap();
  void run();
  // just for test
  void current_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);
 private:
  bool load_global_map_png(std::string &path);
  bool create_global_kdtree(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr);
  pcl::PointCloud<pcl::PointXYZ> search_local_points(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                                                     double cur_x, double cur_y, double radius = 10);
  void publish_points(const pcl::PointCloud<pcl::PointXYZ> &points, const ros::Publisher &pub);
  void actuator_callback(const actuator::actuatorConstPtr &msg);
  void slam_info_callback(const diagnosis_msgs::SlamInfoConstPtr &msg);
  double  HZ_;
  double radius_;
  std::string param_path_;
  std::string frame_id_;
  ros::NodeHandle nh_;
  ros::Subscriber location_sub_;
  ros::Subscriber test_location_sub_;
  ros::Publisher  local_map_Pub_;
  ros::Publisher  global_map_Pub_;
  std::mutex mtx_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_points_ptr_;
  GlobalMapConfig global_map_config_;
  pcl::KdTreeFLANN<pcl::PointXYZ> global_kdtree_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::PointCloud<pcl::PointXYZ> local_points_in_world_;
  geometry_msgs::PoseStamped cur_pose_;

  // test
  bool test_;
  ros::Subscriber goal_sub_;
};

#endif

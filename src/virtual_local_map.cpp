#include "virtual_local_map.h"
#include "lodepng.h"
VirtualLocalMap::VirtualLocalMap(): nh_("~"), global_points_ptr_(nullptr), test_(false), radius_(20) {
  nh_.getParam("HZ", HZ_);
  nh_.getParam("paramPath", param_path_);
  nh_.getParam("width", global_map_config_.width_);
  nh_.getParam("height", global_map_config_.height_);
  nh_.getParam("resolution", global_map_config_.resolution_);
  nh_.getParam("position_x", global_map_config_.origin_x_);
  nh_.getParam("position_y", global_map_config_.origin_y_);
  nh_.getParam("frame_id", frame_id_);
  nh_.getParam("test", test_);
  nh_.getParam("radius", radius_);

  local_map_Pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/virtual_local_map", 1);
  global_map_Pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/virtual_global_map", 1);
  if (test_) {
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &VirtualLocalMap::current_pose_callback, this);
  } else {
    location_sub_ = nh_.subscribe("/actuator", 1, &VirtualLocalMap::actuator_callback, this);
  }
  test_location_sub_ = nh_.subscribe("/slam_info", 1, &VirtualLocalMap::slam_info_callback, this);
  //TODO(lifei) load the global map .pcb
  ROS_INFO_STREAM("start to load map");
  // create kdtree
  load_global_map_png(param_path_);
  ROS_INFO_STREAM(" loaded map success");
  create_global_kdtree(global_points_ptr_);

  ROS_INFO_STREAM("VIRTURAL LOCAL MAP:");
  ROS_INFO_STREAM("HZ " << HZ_);
  ROS_INFO_STREAM("paramPath " << param_path_);
  ROS_INFO_STREAM("map width " << global_map_config_.width_);
  ROS_INFO_STREAM("map height " << global_map_config_.height_);
  ROS_INFO_STREAM("map resolution " << global_map_config_.resolution_);
  ROS_INFO_STREAM("map position_x " << global_map_config_.origin_x_);
  ROS_INFO_STREAM("map position_y " << global_map_config_.origin_y_);
  ROS_INFO_STREAM("frame_id " << frame_id_);
  ROS_INFO_STREAM("test " << test_);
  ROS_INFO_STREAM("radius  " << radius_);
}
VirtualLocalMap::~VirtualLocalMap() {
}

void VirtualLocalMap::run(void) {
  ros::Rate rate(HZ_);
  while (ros::ok()) {
    local_points_in_world_ = search_local_points(global_kdtree_,
                                                 cur_pose_.pose.position.x,
                                                 cur_pose_.pose.position.y, radius_);
    Eigen::Quaternionf q(cur_pose_.pose.orientation.w,
                         cur_pose_.pose.orientation.x,
                         cur_pose_.pose.orientation.y,
                         cur_pose_.pose.orientation.z);
    Eigen::Translation3f t(cur_pose_.pose.position.x,
                           cur_pose_.pose.position.y, 0);
    Eigen::Affine3f w_transform_c = t * q.toRotationMatrix();
    pcl::PointCloud<pcl::PointXYZ> local_points_in_car;
    pcl::transformPointCloud(local_points_in_world_, local_points_in_car, w_transform_c.inverse());
    publish_points(local_points_in_car, local_map_Pub_);
    publish_points(*global_points_ptr_, global_map_Pub_);
    ros::spinOnce();
    rate.sleep();
  }
}

bool VirtualLocalMap::load_global_map_png(std::string &path) {
  std::vector<unsigned char> image; //the raw pixels
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>());
  unsigned width, height;
  unsigned error = lodepng::decode(image, width, height, path);
  if (error) {
    ROS_WARN_STREAM("decoder error " << error << ": " << lodepng_error_text(error));
    return false;
  }
  int channel = 4;
  long int position = 0;
  for (int r = 0; r <= image.size(); r += channel) {
    if ((uint)image[r] != 0) continue;
    position = r / channel;
    cloud_src->points.push_back(
      pcl::PointXYZ((double(position % width)) * global_map_config_.resolution_ + global_map_config_.origin_x_,
                    (double(position / width)) * global_map_config_.resolution_ + global_map_config_.origin_y_, 0));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>());
  voxel_filter_.setInputCloud(cloud_src);
  voxel_filter_.setLeafSize(0.2f, 0.2f, 0.2f);
  voxel_filter_.filter(*cloud_filter);
  global_points_ptr_.swap(cloud_filter);
  return true;
}

bool VirtualLocalMap::create_global_kdtree(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr) {
  if (ptr == nullptr) return false;
  global_kdtree_.setInputCloud(ptr);
  return true;
}

pcl::PointCloud<pcl::PointXYZ> VirtualLocalMap::search_local_points(
  const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
  double cur_x, double cur_y, double radius) {
  pcl::PointXYZ searchPoint, out_point;
  pcl::PointCloud<pcl::PointXYZ> out_point_cloud;
  searchPoint.x = cur_x;
  searchPoint.y = cur_y;
  searchPoint.z = 0;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
      out_point.x = global_points_ptr_->points[ pointIdxRadiusSearch[i] ].x;
      out_point.y = global_points_ptr_->points[pointIdxRadiusSearch[i] ].y;
      out_point.z = global_points_ptr_->points[pointIdxRadiusSearch[i] ].z;
      out_point_cloud.push_back(out_point);
    }
  }
  return out_point_cloud;
}

void VirtualLocalMap::publish_points(const pcl::PointCloud<pcl::PointXYZ> &points,
                                     const ros::Publisher &pub) {
  if (pub.getNumSubscribers() <= 0) return;
  sensor_msgs::PointCloud2 ros_point_cloud;
  pcl::toROSMsg(points, ros_point_cloud);
  ros_point_cloud.header.frame_id = frame_id_.c_str();
  ros_point_cloud.header.stamp = ros::Time::now();
  pub.publish(ros_point_cloud);

}

void VirtualLocalMap::current_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  cur_pose_ = *msg;
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double roll, pitch, theta;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, theta);//进行转换
  std::cout << x << ", " << y << std::endl;
  std::cout << "theta" << theta << std::endl;

}

void VirtualLocalMap::actuator_callback(const actuator::actuatorConstPtr &msg) {
  // TODO(lifei) xuejian tigong
}

void VirtualLocalMap::slam_info_callback(const diagnosis_msgs::SlamInfoConstPtr &msg) {
  cur_pose_ = msg->pose;
  double x = cur_pose_.pose.position.x;
  double y = cur_pose_.pose.position.y;
  double roll, pitch, theta;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(cur_pose_.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, theta);//进行转换
  std::cout << x << ", " << y << std::endl;
  std::cout << "theta" << theta << std::endl;
}



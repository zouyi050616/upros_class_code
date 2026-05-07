#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

class LaserScanToPointCloud {
public:
  LaserScanToPointCloud(ros::NodeHandle& nh) {
    nh.param("x_init", x_init_, 0.0);
    nh.param("y_init", y_init_, 0.0);
    nh.param("yaw_init", yaw_init_, 0.785);  //1.57

    scan_sub_ = nh.subscribe("/scan_filtered", 10, &LaserScanToPointCloud::scanCallback, this);
    odom_sub_ = nh.subscribe("/odom", 10, &LaserScanToPointCloud::odomCallback, this);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/bcsh/upros_class_code/src/upros_navigation/maps/write.pcd", *target_cloud_) == -1) {
      ROS_ERROR("Couldn't read PCD file");
      return;
    }
    kdtree_.setInputCloud(target_cloud_);
    previous_transformation_ = Eigen::Matrix4f::Identity();
    map_to_odom_ = transformMatrix(x_init_, y_init_, yaw_init_);
    ROS_INFO("Loaded PCD map with %zu points", target_cloud_->points.size());
  }

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    auto start = ros::Time::now();

    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan, cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud = performICP(input_cloud, target_cloud_);
    if (aligned_cloud) {
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*aligned_cloud, output);
      output.header.frame_id = "map";
      output.header.stamp = scan->header.stamp;
      cloud_pub_.publish(output);

      map_to_odom_ = previous_transformation_ * odom_transformation_.inverse();

      Eigen::Vector3f trans = map_to_odom_.block<3, 1>(0, 3);
      Eigen::Matrix3f rot = map_to_odom_.block<3, 3>(0, 0);
      Eigen::Quaternionf q(rot);

// 定义时间偏移量（例如减去0.1秒，可根据需要调整）
ros::Duration time_offset(1.0);  // 负值表示向前偏移（过去的时间）

// 计算偏移后的时间戳
ros::Time adjusted_timestamp = scan->header.stamp + time_offset;

tf::Transform tf_transform;
tf_transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
tf_transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

// 使用调整后的时间戳发布TF变换
tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, adjusted_timestamp, "map", "odom_combined"));    
    }

    auto end = ros::Time::now();
    double duration = (end - start).toSec() * 1000.0;
    ROS_INFO("Time taken: %f ms", duration);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;

    Eigen::Matrix3f rot = quaternionToMatrix(ori.x, ori.y, ori.z, ori.w);
    odom_transformation_.block<3, 3>(0, 0) = rot;
    odom_transformation_(0, 3) = pos.x;
    odom_transformation_(1, 3) = pos.y;
    odom_transformation_(2, 3) = pos.z;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr performICP(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(200);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(5);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final, map_to_odom_ * odom_transformation_);

    if (icp.hasConverged()) {
      previous_transformation_ = icp.getFinalTransformation();
      Eigen::Vector3f t = previous_transformation_.block<3, 1>(0, 3);
      Eigen::Matrix3f r = previous_transformation_.block<3, 3>(0, 0);
      Eigen::Vector3f euler = r.eulerAngles(2, 1, 0) * (180 / M_PI);

      ROS_INFO("ICP Translation: x=%f, y=%f", t.x(), t.y());
      ROS_INFO("ICP Rotation Yaw: %f deg", euler.x());

      float overlap = calculateOverlapRate(Final.makeShared(), 0.1);
      ROS_INFO("Overlap rate: %f", overlap);

      return Final.makeShared();
    } else {
      ROS_WARN("ICP did not converge.");
      return nullptr;
    }
  }

  float calculateOverlapRate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, float threshold) {
    int count = 0;
    for (const auto& pt : source->points) {
      std::vector<int> idx(1);
      std::vector<float> dist(1);
      if (kdtree_.nearestKSearch(pt, 1, idx, dist) > 0) {
        if (dist[0] < threshold * threshold)
          count++;
      }
    }
    return static_cast<float>(count) / source->points.size();
  }

  Eigen::Matrix3f quaternionToMatrix(double x, double y, double z, double w) {
    Eigen::Matrix3f m;
    double xx = x * x, yy = y * y, zz = z * z;
    m(0, 0) = 1 - 2 * (yy + zz); m(0, 1) = 2 * (x*y - z*w); m(0, 2) = 2 * (x*z + y*w);
    m(1, 0) = 2 * (x*y + z*w); m(1, 1) = 1 - 2 * (xx + zz); m(1, 2) = 2 * (y*z - x*w);
    m(2, 0) = 2 * (x*z - y*w); m(2, 1) = 2 * (y*z + x*w); m(2, 2) = 1 - 2 * (xx + yy);
    return m;
  }

  Eigen::Matrix4f transformMatrix(double x, double y, double yaw) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    double c = cos(yaw), s = sin(yaw);
    mat(0, 0) = c; mat(0, 1) = -s; mat(0, 3) = x;
    mat(1, 0) = s; mat(1, 1) = c;  mat(1, 3) = y;
    return mat;
  }

private:
  ros::Subscriber scan_sub_, odom_sub_;
  ros::Publisher cloud_pub_;
  laser_geometry::LaserProjection projector_;
  tf::TransformBroadcaster tf_broadcaster_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

  Eigen::Matrix4f previous_transformation_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f odom_transformation_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_to_odom_ = Eigen::Matrix4f::Identity();

  double x_init_, y_init_, yaw_init_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_to_point_cloud");
  ros::NodeHandle nh("~");
  LaserScanToPointCloud node(nh);
  ros::spin();
  return 0;
}


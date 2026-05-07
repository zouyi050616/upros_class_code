#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <limits>

class PointCloudToLaserScan {
public:
  PointCloudToLaserScan() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    scan_sub_ = nh.subscribe("/scan", 10, &PointCloudToLaserScan::scanCallback, this);
    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan_filtered", 10);

    projector_ = new laser_geometry::LaserProjection();

    pnh.param("radius_search", radius_search_, 0.2);
    pnh.param("min_neighbors", min_neighbors_, 20);
    pnh.param("min_distance", min_distance_, 0.3);  // 原点滤除半径
    pnh.param("angle_min", angle_min_, -M_PI);
    pnh.param("angle_max", angle_max_, M_PI);
    pnh.param("angle_increment", angle_increment_, 0.0058); // ~0.33°
    pnh.param("range_min", range_min_, 0.1);
    pnh.param("range_max", range_max_, 30.0);
  }

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::PointCloud2 cloud_msg;
    projector_->projectLaser(*scan, cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *raw_cloud);

    // 1. 剔除距离原点小于 min_distance_ 的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : raw_cloud->points) {
      if (std::hypot(pt.x, pt.y) >= min_distance_) {
        trimmed_cloud->points.push_back(pt);
      }
    }
    trimmed_cloud->width = trimmed_cloud->points.size();
    trimmed_cloud->height = 1;
    trimmed_cloud->is_dense = true;

    // 2. 半径滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(trimmed_cloud);
    radius_filter.setRadiusSearch(radius_search_);
    radius_filter.setMinNeighborsInRadius(min_neighbors_);
    radius_filter.filter(*filtered);

    // 3. 转换回 LaserScan 格式
    int beam_count = std::ceil((angle_max_ - angle_min_) / angle_increment_);
    sensor_msgs::LaserScan output;
    output.header.stamp = scan->header.stamp;
    output.header.frame_id = scan->header.frame_id;
    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = scan->time_increment;
    output.scan_time = scan->scan_time;
    output.range_min = range_min_;
    output.range_max = range_max_;
    output.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

    for (const auto& pt : filtered->points) {
      float r = std::hypot(pt.x, pt.y);
      float angle = std::atan2(pt.y, pt.x);
      if (angle < angle_min_ || angle > angle_max_ || r < range_min_ || r > range_max_) continue;

      int index = std::floor((angle - angle_min_) / angle_increment_);
      if (index >= 0 && index < beam_count) {
        if (r < output.ranges[index]) {
          output.ranges[index] = r;
        }
      }
    }

    scan_pub_.publish(output);
    ROS_INFO("Published filtered LaserScan with %lu points (after trim and radius filter)", filtered->points.size());
  }

  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;
  laser_geometry::LaserProjection* projector_;

  // 参数
  double radius_search_;
  int min_neighbors_;
  double min_distance_;
  double angle_min_, angle_max_, angle_increment_;
  double range_min_, range_max_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "filtered_scan_publisher");
  PointCloudToLaserScan node;
  ros::spin();
  return 0;
}


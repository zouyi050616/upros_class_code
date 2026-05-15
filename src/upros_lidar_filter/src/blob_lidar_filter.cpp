#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

class LaserFilterNode
{
public:
    LaserFilterNode() : nh_("~")
    {
        // 可调参数说明：
        // min_range/max_range：保留的雷达距离范围。min_range 太大可能漏掉近处障碍；max_range 对导航一般不用太远。
        // neighbor_window：向左右各看多少个雷达点。调大能识别更宽的小簇，但也会更容易把噪点连成有效障碍。
        // min_neighbors：一个点至少需要多少个邻居才保留。噪点多就调大到 4；真实细障碍被删就调小到 2。
        // neighbor_radius：邻居之间允许的空间距离。噪点多就调小到 0.10；墙面/盒子被删就调大到 0.18。
        // front_noise_*：只屏蔽正前方近距离噪点。方盒子是 30cm 时，front_noise_max_range 建议 0.30~0.35。
        // near_cluster_*：删除机器人周边的近距离小簇。周边孤立点多就调大 near_cluster_max_points。
        // spike_*：去掉障碍物边缘偶尔突出来的 1-2 个近点。突刺还在就把 spike_range_threshold 调小到 0.08。
        // temporal_*：时间一致性滤波。随机闪点就把 temporal_required_hits 调大到 3；真实障碍反应慢就调回 2。
        nh_.param("min_range", min_range_, 0.25);
        nh_.param("max_range", max_range_, 12.0);
        nh_.param("neighbor_window", neighbor_window_, 4);
        nh_.param("min_neighbors", min_neighbors_, 3);
        nh_.param("neighbor_radius", neighbor_radius_, 0.15);
        nh_.param("beam_radius_scale", beam_radius_scale_, 2.5);
        nh_.param("front_noise_filter", front_noise_filter_, true);
        nh_.param("front_noise_angle", front_noise_angle_, 0.35);
        nh_.param("front_noise_max_range", front_noise_max_range_, 0.35);
        nh_.param("near_cluster_filter", near_cluster_filter_, true);
        nh_.param("near_cluster_max_range", near_cluster_max_range_, 0.70);
        nh_.param("near_cluster_max_points", near_cluster_max_points_, 8);
        nh_.param("near_cluster_tolerance", near_cluster_tolerance_, 0.10);
        nh_.param("spike_filter", spike_filter_, true);
        nh_.param("spike_max_points", spike_max_points_, 2);
        nh_.param("spike_side_window", spike_side_window_, 4);
        nh_.param("spike_range_threshold", spike_range_threshold_, 0.12);
        nh_.param("spike_cluster_tolerance", spike_cluster_tolerance_, 0.06);
        nh_.param("temporal_filter", temporal_filter_, true);
        nh_.param("temporal_required_hits", temporal_required_hits_, 2);
        nh_.param("temporal_angle_window", temporal_angle_window_, 1);
        nh_.param("temporal_range_tolerance", temporal_range_tolerance_, 0.12);

        neighbor_window_ = std::max(1, neighbor_window_);
        min_neighbors_ = std::max(1, min_neighbors_);
        min_neighbors_ = std::min(min_neighbors_, neighbor_window_ * 2);
        neighbor_radius_ = std::max(0.01, neighbor_radius_);
        beam_radius_scale_ = std::max(1.0, beam_radius_scale_);
        front_noise_angle_ = std::max(0.0, front_noise_angle_);
        front_noise_max_range_ = std::max(min_range_, front_noise_max_range_);
        near_cluster_max_range_ = std::max(min_range_, near_cluster_max_range_);
        near_cluster_max_points_ = std::max(1, near_cluster_max_points_);
        near_cluster_tolerance_ = std::max(0.01, near_cluster_tolerance_);
        spike_max_points_ = std::max(1, spike_max_points_);
        spike_side_window_ = std::max(1, spike_side_window_);
        spike_range_threshold_ = std::max(0.01, spike_range_threshold_);
        spike_cluster_tolerance_ = std::max(0.01, spike_cluster_tolerance_);
        temporal_required_hits_ = std::max(1, temporal_required_hits_);
        temporal_angle_window_ = std::max(0, temporal_angle_window_);
        temporal_range_tolerance_ = std::max(0.01, temporal_range_tolerance_);

        sub_ = nh_.subscribe("/scan", 1, &LaserFilterNode::laserScanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        sensor_msgs::LaserScan filtered_scan = *msg;
        const size_t scan_size = filtered_scan.ranges.size();
        if (scan_size == 0)
        {
            pub_.publish(filtered_scan);
            return;
        }

        std::vector<bool> valid(scan_size, false);
        std::vector<bool> keep(scan_size, false);

        // 第一步：过滤范围外的点 距离过滤
        for (size_t i = 0; i < scan_size; ++i)
        {
            const float range = filtered_scan.ranges[i];
            const bool finite_range = std::isfinite(range);
            const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            const bool front_noise = front_noise_filter_ &&
                                     std::abs(angle) <= front_noise_angle_ &&
                                     range <= front_noise_max_range_;
            valid[i] = finite_range &&
                       range >= min_range_ &&
                       range <= max_range_ &&
                       !front_noise;
            if (!valid[i])
            {
                filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // 第二步：删除机器人周边的近距离小簇，避免自体回波在局部地图里变成障碍。
        if (near_cluster_filter_)
        {
            removeNearSmallClusters(msg->ranges, valid, filtered_scan.ranges);
        }

        // 第三步：删除短尖刺簇。典型表现是障碍物表面偶尔向外突出来 1-2 个更近的点。
        if (spike_filter_)
        {
            removeShortSpikes(msg->ranges, valid, filtered_scan.ranges);
        }

        // 第四步：在原始 scan 上统计附近 beam 的空间邻居数，去掉单点/小簇噪声。
        for (size_t i = 0; i < scan_size; ++i)
        {
            if (!valid[i])
            {
                continue;
            }

            int neighbors = 0;
            for (int offset = -neighbor_window_; offset <= neighbor_window_; ++offset)
            {
                if (offset == 0)
                {
                    continue;
                }

                const int neighbor_index = static_cast<int>(i) + offset;
                if (neighbor_index < 0 || neighbor_index >= static_cast<int>(scan_size))
                {
                    continue;
                }

                if (!valid[neighbor_index])
                {
                    continue;
                }

                const double angle_delta = std::abs(offset * msg->angle_increment);
                const double distance = pointDistance(msg->ranges[i],
                                                      msg->ranges[neighbor_index],
                                                      angle_delta);
                const double beam_spacing =
                    std::max(msg->ranges[i], msg->ranges[neighbor_index]) *
                    std::abs(msg->angle_increment) * beam_radius_scale_;
                const double allowed_distance = std::max(neighbor_radius_, beam_spacing);

                if (distance <= allowed_distance)
                {
                    ++neighbors;
                    if (neighbors >= min_neighbors_)
                    {
                        keep[i] = true;
                        break;
                    }
                }
            }
        }

        for (size_t i = 0; i < scan_size; ++i)
        {
            if (valid[i] && !keep[i])
            {
                filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // 第五步：时间一致性滤波。只闪一两帧的点不发布，避免 costmap 被随机点堵住。
        if (temporal_filter_)
        {
            applyTemporalConsistency(filtered_scan.ranges);
        }

        pub_.publish(filtered_scan);
    }

private:
    double pointDistance(double range_a, double range_b, double angle_delta) const
    {
        const double cos_delta = std::cos(angle_delta);
        const double squared_distance =
            range_a * range_a + range_b * range_b - 2.0 * range_a * range_b * cos_delta;
        return std::sqrt(std::max(0.0, squared_distance));
    }

    void removeShortSpikes(const std::vector<float> &input_ranges,
                           std::vector<bool> &valid,
                           std::vector<float> &output_ranges) const
    {
        const int scan_size = static_cast<int>(input_ranges.size());
        int i = 0;
        while (i < scan_size)
        {
            if (!valid[i])
            {
                ++i;
                continue;
            }

            const int start = i;
            int end = i;
            while (end + 1 < scan_size &&
                   valid[end + 1] &&
                   std::abs(input_ranges[end + 1] - input_ranges[end]) <= spike_cluster_tolerance_)
            {
                ++end;
            }

            const int cluster_size = end - start + 1;
            if (cluster_size <= spike_max_points_ && isNearSpike(input_ranges, valid, start, end))
            {
                for (int j = start; j <= end; ++j)
                {
                    valid[j] = false;
                    output_ranges[j] = std::numeric_limits<float>::quiet_NaN();
                }
            }

            i = end + 1;
        }
    }

    void removeNearSmallClusters(const std::vector<float> &input_ranges,
                                 std::vector<bool> &valid,
                                 std::vector<float> &output_ranges) const
    {
        const int scan_size = static_cast<int>(input_ranges.size());
        int i = 0;
        while (i < scan_size)
        {
            if (!valid[i])
            {
                ++i;
                continue;
            }

            const int start = i;
            int end = i;
            double min_range = input_ranges[i];
            while (end + 1 < scan_size &&
                   valid[end + 1] &&
                   std::abs(input_ranges[end + 1] - input_ranges[end]) <= near_cluster_tolerance_)
            {
                ++end;
                min_range = std::min(min_range, static_cast<double>(input_ranges[end]));
            }

            const int cluster_size = end - start + 1;
            if (min_range <= near_cluster_max_range_ && cluster_size <= near_cluster_max_points_)
            {
                for (int j = start; j <= end; ++j)
                {
                    valid[j] = false;
                    output_ranges[j] = std::numeric_limits<float>::quiet_NaN();
                }
            }

            i = end + 1;
        }
    }

    bool isNearSpike(const std::vector<float> &ranges,
                     const std::vector<bool> &valid,
                     int start,
                     int end) const
    {
        const int scan_size = static_cast<int>(ranges.size());
        double cluster_range = 0.0;
        for (int i = start; i <= end; ++i)
        {
            cluster_range += ranges[i];
        }
        cluster_range /= static_cast<double>(end - start + 1);

        bool has_left = false;
        bool has_right = false;
        double left_range = 0.0;
        double right_range = 0.0;

        for (int i = start - 1; i >= 0 && i >= start - spike_side_window_; --i)
        {
            if (valid[i])
            {
                has_left = true;
                left_range = ranges[i];
                break;
            }
        }

        for (int i = end + 1; i < scan_size && i <= end + spike_side_window_; ++i)
        {
            if (valid[i])
            {
                has_right = true;
                right_range = ranges[i];
                break;
            }
        }

        return has_left &&
               has_right &&
               cluster_range + spike_range_threshold_ < left_range &&
               cluster_range + spike_range_threshold_ < right_range;
    }

    void applyTemporalConsistency(std::vector<float> &ranges)
    {
        const size_t scan_size = ranges.size();
        if (previous_ranges_.size() != scan_size || hit_counts_.size() != scan_size)
        {
            previous_ranges_.assign(scan_size, std::numeric_limits<float>::quiet_NaN());
            hit_counts_.assign(scan_size, 0);
        }

        const std::vector<float> old_ranges = previous_ranges_;
        const std::vector<int> old_counts = hit_counts_;
        std::vector<float> new_ranges(scan_size, std::numeric_limits<float>::quiet_NaN());
        std::vector<int> new_counts(scan_size, 0);

        for (size_t i = 0; i < scan_size; ++i)
        {
            if (!std::isfinite(ranges[i]))
            {
                continue;
            }

            int best_old_count = 0;
            const int center = static_cast<int>(i);
            for (int offset = -temporal_angle_window_; offset <= temporal_angle_window_; ++offset)
            {
                const int old_index = center + offset;
                if (old_index < 0 || old_index >= static_cast<int>(scan_size))
                {
                    continue;
                }

                if (std::isfinite(old_ranges[old_index]) &&
                    std::abs(ranges[i] - old_ranges[old_index]) <= temporal_range_tolerance_)
                {
                    best_old_count = std::max(best_old_count, old_counts[old_index]);
                }
            }

            const int hit_count = best_old_count + 1;
            new_ranges[i] = ranges[i];
            new_counts[i] = std::min(hit_count, temporal_required_hits_);
            if (hit_count < temporal_required_hits_)
            {
                ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        previous_ranges_.swap(new_ranges);
        hit_counts_.swap(new_counts);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double min_range_;
    double max_range_;
    int neighbor_window_;
    int min_neighbors_;
    double neighbor_radius_;
    double beam_radius_scale_;
    bool front_noise_filter_;
    double front_noise_angle_;
    double front_noise_max_range_;
    bool near_cluster_filter_;
    double near_cluster_max_range_;
    int near_cluster_max_points_;
    double near_cluster_tolerance_;
    bool spike_filter_;
    int spike_max_points_;
    int spike_side_window_;
    double spike_range_threshold_;
    double spike_cluster_tolerance_;
    bool temporal_filter_;
    int temporal_required_hits_;
    int temporal_angle_window_;
    double temporal_range_tolerance_;
    std::vector<float> previous_ranges_;
    std::vector<int> hit_counts_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_filter");
    LaserFilterNode node;
    ros::spin();
    return 0;
}

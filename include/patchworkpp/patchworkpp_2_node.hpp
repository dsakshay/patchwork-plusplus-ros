#pragma once

#include "patchworkpp_2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "fpn_msgs/msg/polygon_array.hpp"

#include <Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>
#include <queue>
#include <mutex>
#include <vector>

using namespace std;

class PatchWorkPPNode : public rclcpp::Node
{
public:
    PatchWorkPPNode();

    void init_params();
    void init_publishers();
    geometry_msgs::msg::PolygonStamped set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split);
    void visualise();
    void set_ground_likelihood(double likelihood);
    void estimate_ground(pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground, double &time_taken);

private:
    std::shared_ptr<PatchWorkPP> patchworkpp_;
    PatchWorkParams params_;

    fpn_msgs::msg::PolygonArray poly_list_;

    rclcpp::Publisher<fpn_msgs::msg::PolygonArray>::SharedPtr pub_plane_viz_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_revert_pc_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_reject_pc_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_normal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_noise_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_vertical_;

};
#include "patchworkpp/patchworkpp_2_node.hpp"

using namespace std;

PatchWorkPPNode::PatchWorkPPNode() : Node("patchworkpp_2_node")
{
    RCLCPP_INFO(this->get_logger(), "Node initialising!");
    init_params();
    patchworkpp_ = std::make_shared<PatchWorkPP>(params_);
    init_publishers();
    init_subscribers();

    RCLCPP_INFO(this->get_logger(), "Node initialised!");
}

void PatchWorkPPNode::init_params()
{
    RCLCPP_WARN(this->get_logger(), "Params initialising!");
    this->declare_parameter("verbose", false);
    this->declare_parameter("sensor_height", 1.723);
    this->declare_parameter("num_iter", 3);
    this->declare_parameter("num_lpr", 20);
    this->declare_parameter("num_min_pts", 10);
    this->declare_parameter("th_seeds", 0.4);
    this->declare_parameter("th_dist", 0.3);
    this->declare_parameter("th_seeds_v", 0.4);
    this->declare_parameter("max_r", 80.0);
    this->declare_parameter("min_r", 2.7);
    this->declare_parameter("uprightness_thr", 0.5);
    this->declare_parameter("adaptive_seed_selection_margin", -1.1);
    this->declare_parameter("RNR_ver_angle_thr", -15.0);
    this->declare_parameter("RNR_intensity_thr", 0.2);
    this->declare_parameter("max_flatness_storage", 1000);
    this->declare_parameter("max_elevation_storage", 1000);
    this->declare_parameter("enable_RNR", true);
    this->declare_parameter("enable_RVPF", true);
    this->declare_parameter("enable_TGR", true);

    RCLCPP_WARN(this->get_logger(), "Declaring params before nested ones");

    this->declare_parameter("czm.num_zones", 4);
    this->declare_parameter("czm.num_sectors_each_zone", vector<int>{16, 32, 54, 32});
    this->declare_parameter("czm.num_rings_each_zone", vector<int>{2, 4, 4, 4});
    this->declare_parameter("czm.elevation_thresholds", vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("czm.flatness_thresholds", vector<double>{0.0, 0.0, 0.0, 0.0});

    RCLCPP_WARN(this->get_logger(), "Declaring nested params ones");
    this->declare_parameter("visualize", true);

    this->declare_parameter("cloud_topic", "/ouster/cloud");
    this->declare_parameter("frame_id", "");

    this->get_parameter("verbose", params_.verbose);
    this->get_parameter("sensor_height", params_.sensor_height);
    this->get_parameter("num_iter", params_.num_iter);
    this->get_parameter("num_lpr", params_.num_lpr);
    this->get_parameter("num_min_pts", params_.num_min_pts);
    this->get_parameter("th_seeds", params_.th_seeds);
    this->get_parameter("th_dist", params_.th_dist);
    this->get_parameter("th_seeds_v", params_.th_seeds_v);
    this->get_parameter("th_dist_v", params_.th_dist_v);
    this->get_parameter("max_r", params_.max_range);
    this->get_parameter("min_r", params_.min_range);
    this->get_parameter("uprightness_thr", params_.uprightness_thr);
    this->get_parameter("adaptive_seed_selection_margin", params_.adaptive_seed_selection_margin);
    this->get_parameter("RNR_ver_angle_thr", params_.RNR_ver_angle_thr);
    this->get_parameter("RNR_intensity_thr", params_.RNR_intensity_thr);
    this->get_parameter("max_flatness_storage", params_.max_flatness_storage);
    this->get_parameter("max_elevation_thr", params_.max_elevation_storage);
    this->get_parameter("enable_RNR", params_.enable_RNR);
    this->get_parameter("enable_RVPF", params_.enable_RVPF);
    this->get_parameter("enable_TGR", params_.enable_TGR);

    RCLCPP_WARN(this->get_logger(), "Getting params before nested ones");

    
    this->get_parameter("czm.num_zones", params_.num_zones);

    rclcpp::Parameter num_sectors_each_zone_param("czm.num_sectors_each_zone", vector<int>({}));
    rclcpp::Parameter num_rings_each_zone_param("czm.num_rings_each_zone", vector<int>({}));
    rclcpp::Parameter elevation_thr_param("czm.elevation_thresholds", vector<double>({}));
    rclcpp::Parameter flatness_thr_param("czm.flatness_thresholds", vector<double>({}));
    
    this->get_parameter("czm.num_sectors_each_zone", num_sectors_each_zone_param);
    this->get_parameter("czm.num_rings_each_zone", num_rings_each_zone_param);
    this->get_parameter("czm.elevation_thresholds", elevation_thr_param);
    this->get_parameter("czm.flatness_thresholds", flatness_thr_param);

    RCLCPP_WARN(this->get_logger(), "Getting nested params ");
    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("frame_id", frame_id_);

    params_.num_sectors_each_zone = num_sectors_each_zone_param.as_integer_array();
    params_.num_rings_each_zone = num_rings_each_zone_param.as_integer_array();
    params_.elevation_thr = elevation_thr_param.as_double_array();
    params_.flatness_thr = flatness_thr_param.as_double_array();

    this->get_parameter("visualise", params_.visualise);

    // RCLCPP_WARN(this->get_logger(), "Getting polylist headers");
    // poly_list_.header.frame_id = "map";
    // poly_list_.polygons.reserve(patchworkpp_->num_polygons);
    // RCLCPP_WARN(this->get_logger(), "Got polylist headers");

    RCLCPP_WARN(this->get_logger(), "initialised params!");
}

void PatchWorkPPNode::init_publishers()
{
    pub_plane_viz_ = this->create_publisher<fpn_msgs::msg::PolygonArray>("plane", 100);
    pub_revert_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("revert_pc", 100);
    pub_reject_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reject_pc", 100);
    pub_normal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("normals", 100);
    pub_noise_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("noise", 100);
    pub_vertical_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vertical", 100);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 100);
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_cloud", 100);
    pub_non_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_cloud", 100);
    
}

void PatchWorkPPNode::init_subscribers()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos, std::bind(&PatchWorkPPNode::cloud_cb, this, std::placeholders::_1));
}

void PatchWorkPPNode::cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    double time_taken;

    pcl::PointCloud<PointT> pc_curr;
    pcl::PointCloud<PointT> pc_ground;
    pcl::PointCloud<PointT> pc_non_ground;

    pcl::fromROSMsg(*msg, pc_curr);

    this->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    pub_cloud_->publish(this->cloud2msg(pc_curr, msg->header.stamp, msg->header.frame_id));
    pub_ground_->publish(this->cloud2msg(pc_ground, msg->header.stamp, msg->header.frame_id));
    pub_cloud_->publish(this->cloud2msg(pc_non_ground, msg->header.stamp, msg->header.frame_id));
}

sensor_msgs::msg::PointCloud2 PatchWorkPPNode::cloud2msg(pcl::PointCloud<PointT> cloud, const rclcpp::Time& stamp, std::string frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_ROS;

    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;

    return cloud_ROS;
}

geometry_msgs::msg::PolygonStamped PatchWorkPPNode::set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split)
{
    geometry_msgs::msg::PolygonStamped polygons;
    geometry_msgs::msg::Point32 point;

    // RL
    double zone_min_range = params_.min_ranges[zone_idx];
    double r_len = r_idx * params_.ring_sizes[zone_idx] + zone_min_range;
    double angle = theta_idx * params_.sector_sizes[zone_idx];

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);
    // RU
    r_len = r_len + params_.ring_sizes[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx) {
        angle = angle + params_.sector_sizes[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    r_len = r_len - params_.ring_sizes[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx) {
        angle = angle - params_.sector_sizes[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    return polygons;
}

void PatchWorkPPNode::estimate_ground(pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground, double &time_taken)
{
    // unique_lock<recursive_mutex> lock(mutex_);
    poly_list_.header.stamp = this->get_clock()->now();
    poly_list_.header.frame_id = cloud_in.header.frame_id;
    if (!poly_list_.polygons.empty()) poly_list_.polygons.clear();
    if (!poly_list_.likelihood.empty()) poly_list_.likelihood.clear();

    static double start, t0, t1, t2, end;

    double pca_time_ = 0.0;
    double t_revert = 0.0;
    double t_total_ground = 0.0;
    double t_total_estimate = 0.0;

    start = this->get_clock()->now().seconds();

    cloud_ground.clear();
    cloud_nonground.clear();

    if (params_.enable_RNR) patchworkpp_->reflected_noise_removal(cloud_in, cloud_nonground);

    t1 = this->get_clock()->now().seconds();

    int concentric_idx = 0;

    double t_sort = 0;

    vector<RevertCandidate> candidates;
    vector<double> ringwise_flatness;

    for (int zone_idx = 0; zone_idx < params_.num_zones; ++zone_idx) {

        auto zone = patchworkpp_->ConcentricZoneModel_[zone_idx];

        for (int ring_idx = 0; ring_idx < params_.num_rings_each_zone[zone_idx]; ++ring_idx) {
            for (int sector_idx = 0; sector_idx < params_.num_sectors_each_zone[zone_idx]; ++sector_idx) {

                if (zone[ring_idx][sector_idx].points.size() < params_.num_min_pts)
                {
                    cloud_nonground += zone[ring_idx][sector_idx];
                    continue;
                }

                // --------- region-wise sorting (faster than global sorting method) ---------------- //
                double t_sort_0 = this->get_clock()->now().seconds();

                sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].points.end(), point_z_cmp);

                double t_sort_1 = this->get_clock()->now().seconds();
                t_sort += (t_sort_1 - t_sort_0);
                // ---------------------------------------------------------------------------------- //

                double t_tmp0 = this->get_clock()->now().seconds();
                patchworkpp_->extract_piecewiseground(zone_idx, zone[ring_idx][sector_idx], patchworkpp_->regionwise_ground_, patchworkpp_->regionwise_nonground_);

                double t_tmp1 = this->get_clock()->now().seconds();
                t_total_ground += t_tmp1 - t_tmp0;
                pca_time_ += t_tmp1 - t_tmp0;

                // Status of each patch
                // used in checking uprightness, elevation, and flatness, respectively
                const double ground_uprightness = patchworkpp_->normal_(2);
                const double ground_elevation   = patchworkpp_->pc_mean_(2, 0);
                const double ground_flatness    = patchworkpp_->singular_values_.minCoeff();
                const double line_variable      = patchworkpp_->singular_values_(1) != 0 ? patchworkpp_->singular_values_(0)/patchworkpp_->singular_values_(1) : std::numeric_limits<double>::max();

                double heading = 0.0;
                for(int i=0; i<3; i++) heading += patchworkpp_->pc_mean_(i,0)*patchworkpp_->normal_(i);

                if (params_.visualise) {
                    auto polygons = this->set_polygons(zone_idx, ring_idx, sector_idx, 3);
                    polygons.header = poly_list_.header;
                    poly_list_.polygons.push_back(polygons);
                    patchworkpp_->set_ground_likelihood_estimation_status(zone_idx, ring_idx, concentric_idx, ground_uprightness, ground_elevation, ground_flatness);

                    pcl::PointXYZINormal tmp_p;
                    tmp_p.x = patchworkpp_->pc_mean_(0,0);
                    tmp_p.y = patchworkpp_->pc_mean_(1,0);
                    tmp_p.z = patchworkpp_->pc_mean_(2,0);
                    tmp_p.normal_x = patchworkpp_->normal_(0);
                    tmp_p.normal_y = patchworkpp_->normal_(1);
                    tmp_p.normal_z = patchworkpp_->normal_(2);
                    patchworkpp_->normals_.points.emplace_back(tmp_p);
                }

                double t_tmp2 = this->get_clock()->now().seconds();

                /*
                    About 'is_heading_outside' condidition, heading should be smaller than 0 theoretically.
                    ( Imagine the geometric relationship between the surface normal vector on the ground plane and
                        the vector connecting the sensor origin and the mean point of the ground plane )

                    However, when the patch is far awaw from the sensor origin,
                    heading could be larger than 0 even if it's ground due to lack of amount of ground plane points.

                    Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near condition )
                */
                bool is_upright         = ground_uprightness > params_.uprightness_thr;
                bool is_not_elevated    = ground_elevation < params_.elevation_thr[concentric_idx];
                bool is_flat            = ground_flatness < params_.flatness_thr[concentric_idx];
                bool is_near_zone       = concentric_idx < params_.num_rings_of_interest;
                bool is_heading_outside = heading < 0.0;

                /*
                    Store the elevation & flatness variables
                    for A-GLE (Adaptive Ground Likelihood Estimation)
                    and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
                */
                if (is_upright && is_not_elevated && is_near_zone)
                {
                    params_.update_elevation[concentric_idx].push_back(ground_elevation);
                    params_.update_flatness[concentric_idx].push_back(ground_flatness);

                    ringwise_flatness.push_back(ground_flatness);
                }

                // Ground estimation based on conditions
                if (!is_upright)
                {
                    cloud_nonground += patchworkpp_->regionwise_ground_;
                }
                else if (!is_near_zone)
                {
                    cloud_ground += patchworkpp_->regionwise_ground_;
                }
                else if (!is_heading_outside)
                {
                    cloud_nonground += patchworkpp_->regionwise_ground_;
                }
                else if (is_not_elevated || is_flat)
                {
                    cloud_ground += patchworkpp_->regionwise_ground_;
                }
                else
                {
                    RevertCandidate candidate(concentric_idx, sector_idx, ground_flatness, line_variable, patchworkpp_->pc_mean_, patchworkpp_->regionwise_ground_);
                    candidates.push_back(candidate);
                }
                // Every regionwise_nonground is considered nonground.
                cloud_nonground += patchworkpp_->regionwise_nonground_;

                double t_tmp3 = this->get_clock()->now().seconds();
                t_total_estimate += t_tmp3 - t_tmp2;
            }

            double t_bef_revert = this->get_clock()->now().seconds();

            if (!candidates.empty())
            {
                if (params_.enable_TGR)
                {
                    patchworkpp_->temporal_ground_revert(cloud_ground, cloud_nonground, ringwise_flatness, candidates, concentric_idx);
                }
                else
                {
                    for (size_t i=0; i<candidates.size(); i++)
                    {
                        cloud_nonground += candidates[i].regionwise_ground;
                    }
                }

                candidates.clear();
                ringwise_flatness.clear();
            }

            double t_aft_revert = this->get_clock()->now().seconds();

            t_revert += t_aft_revert - t_bef_revert;

            concentric_idx++;

        }
    }

    double t_update = this->get_clock()->now().seconds();

    patchworkpp_->update_elevation_thr();
    patchworkpp_->update_flatness_thr();

    end = this->get_clock()->now().seconds();

    time_taken = end - start;

    if (params_.visualise)
    {
        this->visualise();
    }

    patchworkpp_->revert_pc_.clear();
    patchworkpp_->reject_pc_.clear();
    patchworkpp_->normals_.clear();
    patchworkpp_->noise_pc_.clear();
    patchworkpp_->vertical_pc_.clear();

}

void PatchWorkPPNode::visualise()
{
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(patchworkpp_->revert_pc_, cloud_ROS);
    cloud_ROS.header.stamp = this->get_clock()->now();
    cloud_ROS.header.frame_id = frame_id_;
    pub_revert_pc_->publish(cloud_ROS);

    pcl::toROSMsg(patchworkpp_->reject_pc_, cloud_ROS);
    cloud_ROS.header.stamp = this->get_clock()->now();
    cloud_ROS.header.frame_id = frame_id_;
    pub_reject_pc_->publish(cloud_ROS);

    pcl::toROSMsg(patchworkpp_->normals_, cloud_ROS);
    cloud_ROS.header.stamp = this->get_clock()->now();
    cloud_ROS.header.frame_id = frame_id_;
    pub_normal_->publish(cloud_ROS);

    pcl::toROSMsg(patchworkpp_->noise_pc_, cloud_ROS);
    cloud_ROS.header.stamp = this->get_clock()->now();
    cloud_ROS.header.frame_id = frame_id_;
    pub_noise_->publish(cloud_ROS);

    pcl::toROSMsg(patchworkpp_->vertical_pc_, cloud_ROS);
    cloud_ROS.header.stamp = this->get_clock()->now();
    cloud_ROS.header.frame_id = frame_id_;
    pub_vertical_->publish(cloud_ROS);

    pub_plane_viz_->publish(poly_list_);
}

void PatchWorkPPNode::set_ground_likelihood(double likelihood)
{
    likelihood = patchworkpp_->get_likelihood();
    poly_list_.likelihood.push_back(likelihood);
}


int main(int argc, char** argv) {
    std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PatchWorkPPNode>();

    rclcpp::Rate loop_rate(20ms);
    while(rclcpp::ok()) {
        // TODO: implement try-catch here.
        // RCLCPP_INFO(node->get_logger(), "In while loop!");
        // node->run();
        rclcpp::spin_some(node);

        loop_rate.sleep();
    }
    // rclcpp::spin(node);
    // node->run();

    rclcpp::shutdown();

    return 0;
}
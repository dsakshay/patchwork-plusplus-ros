#include "patchworkpp/patchworkpp_2.hpp"


PatchWorkPP::PatchWorkPP(PatchWorkParams & params)
{
    params_ = params;
    this->initialise();
}

void PatchWorkPP::initialise()
{
    if (params_.num_zones != 4 || params_.num_sectors_each_zone.size() != params_.num_rings_each_zone.size()) {
            throw invalid_argument("Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone");
        }
    if (params_.elevation_thr.size() != params_.flatness_thr.size()) {
        throw invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
    }

    params_.num_rings_of_interest = params_.elevation_thr.size();

    num_polygons = std::inner_product(params_.num_rings_each_zone.begin(), params_.num_rings_each_zone.end(), params_.num_sectors_each_zone.begin(), 0);

    revert_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

    params_.min_range_z2 = (7 * params_.min_range + params_.max_range) / 8.0;
    params_.min_range_z3 = (3 * params_.min_range + params_.max_range) / 4.0;
    params_.min_range_z4 = (params_.min_range + params_.max_range) / 2.0;

    params_.min_ranges = {params_.min_range, params_.min_range_z2, params_.min_range_z3, params_.min_range_z4};
    params_.ring_sizes = {(params_.min_range_z2 - params_.min_range) / params_.num_rings_each_zone.at(0),
                    (params_.min_range_z3 - params_.min_range_z2) / params_.num_rings_each_zone.at(1),
                    (params_.min_range_z4 - params_.min_range_z3) / params_.num_rings_each_zone.at(2),
                    (params_.max_range - params_.min_range_z4) / params_.num_rings_each_zone.at(3)};

    params_.sector_sizes = {2 * M_PI / params_.num_sectors_each_zone.at(0), 2 * M_PI / params_.num_sectors_each_zone.at(1),
                    2 * M_PI / params_.num_sectors_each_zone.at(2),
                    2 * M_PI / params_.num_sectors_each_zone.at(3)};

    for (int i = 0; i < params_.num_zones; i++) {
        Zone z;
        initialise_zone(z, params_.num_sectors_each_zone[i], params_.num_rings_each_zone[i]);
        ConcentricZoneModel_.push_back(z);
    }    
}


void PatchWorkPP::initialise_zone(Zone &z, int num_sectors, int num_rings) {
    z.clear();
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(1000);
    Ring ring;
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++) {
        z.emplace_back(ring);
    }
}

void PatchWorkPP::flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings) {
    for (int i = 0; i < num_sectors; i++) {
        for (int j = 0; j < num_rings; j++) {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

void PatchWorkPP::flush_patches(vector<Zone> &czm) {
    for (int k = 0; k < params_.num_zones; k++) {
        for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
            for (int j = 0; j < params_.num_sectors_each_zone[k]; j++) {
                if (!czm[k][i][j].points.empty()) czm[k][i][j].points.clear();
            }
        }
    }

    if( params_.verbose ) cout << "Flushed patches" << endl;
}

void PatchWorkPP::estimate_plane(const pcl::PointCloud<PointT> &ground) {
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));

    if (normal_(2) < 0) { for(int i=0; i<3; i++) normal_(i) *= -1; }

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}


void PatchWorkPP::extract_initial_seeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds, double th_seed) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < params_.num_lpr; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seed) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}


void PatchWorkPP::extract_initial_seeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < params_.num_lpr; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + params_.th_seeds) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}


void PatchWorkPP::reflected_noise_removal(pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_nonground)
{
    for (int i=0; i<cloud_in.size(); i++)
    {
        double r = sqrt( cloud_in[i].x*cloud_in[i].x + cloud_in[i].y*cloud_in[i].y );
        double z = cloud_in[i].z;
        double ver_angle_in_deg = atan2(z, r)*180/M_PI;

        if ( ver_angle_in_deg < params_.RNR_ver_angle_thr && z < -params_.sensor_height && cloud_in[i].intensity < params_.RNR_intensity_thr)
        {
            cloud_nonground.push_back(cloud_in[i]);
            noise_pc_.push_back(cloud_in[i]);
            noise_idxs_.push(i);
        }
    }

    if (params_.verbose) cout << "[ RNR ] Num of noises : " << noise_pc_.points.size() << endl;
}




void PatchWorkPP::update_elevation_thr(void)
{
    for (int i=0; i < params_.num_rings_of_interest; i++)
    {
        if (params_.update_elevation[i].empty()) continue;

        double update_mean = 0.0, update_stdev = 0.0;

        if (i == 0) {
            params_.elevation_thr[i] = update_mean + 3*update_stdev;
            params_.sensor_height = -update_mean;
        }
        else params_.elevation_thr[i] = update_mean + 2*update_stdev;

        int exceed_num = params_.update_elevation[i].size() - params_.max_elevation_storage;
        if (exceed_num > 0) params_.update_elevation[i].erase(params_.update_elevation[i].begin(), params_.update_elevation[i].begin() + exceed_num);
    }

    if (params_.verbose)
    {
        cout << "sensor height: " << params_.sensor_height << endl;
        cout << (boost::format("elevation_thr_  :   %0.4f,  %0.4f,  %0.4f,  %0.4f")
                % params_.elevation_thr[0] % params_.elevation_thr[1] % params_.elevation_thr[2] % params_.elevation_thr[3]).str() << endl;
    }

    return;
}


void PatchWorkPP::update_flatness_thr(void)
{
    for (int i = 0; i<params_.num_rings_of_interest; i++)
    {
        if (params_.update_flatness[i].empty()) break;
        if (params_.update_flatness[i].size() <= 1) break;

        double update_mean = 0.0, update_stdev = 0.0;
        this->calc_mean_stdev(params_.update_flatness[i], update_mean, update_stdev);
        params_.flatness_thr[i] = update_mean+update_stdev;

        // if (verbose_) { cout << "flatness threshold [" << i << "]: " << flatness_thr_[i] << endl; }

        int exceed_num = params_.update_flatness[i].size() - params_.max_flatness_storage;
        if (exceed_num > 0) params_.update_flatness[i].erase(params_.update_flatness[i].begin(), params_.update_flatness[i].begin() + exceed_num);
    }

    if (params_.verbose)
    {
        cout << (boost::format("flatness_thr_   :   %0.4f,  %0.4f,  %0.4f,  %0.4f")
                % params_.flatness_thr[0] % params_.flatness_thr[1] % params_.flatness_thr[2] % params_.flatness_thr[3]).str() << endl;
    }

    return;
}


void PatchWorkPP::temporal_ground_revert(pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground,
                                               std::vector<double> ring_flatness, std::vector<RevertCandidate> candidates,
                                               int concentric_idx)
{
    if (params_.verbose) std::cout << "\033[1;34m" << "=========== Temporal Ground Revert (TGR) ===========" << "\033[0m" << endl;

    double mean_flatness = 0.0, stdev_flatness = 0.0;
    calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);

    if (params_.verbose)
    {
        cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
             << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness << std::endl;
    }

    for( size_t i=0; i<candidates.size(); i++ )
    {
        RevertCandidate candidate = candidates[i];

        // Debug
        if(params_.verbose)
        {
            cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
                 << " / flatness: " << candidate.ground_flatness
                 << " / line_variable: " << candidate.line_variable
                 << " / ground_num : " << candidate.regionwise_ground.size()
                 << "\033[0m" << endl;
        }

        double mu_flatness = mean_flatness + 1.5*stdev_flatness;
        double prob_flatness = 1/(1+exp( (candidate.ground_flatness-mu_flatness)/(mu_flatness/10) ));

        if (candidate.regionwise_ground.size() > 1500 && candidate.ground_flatness < params_.th_dist*params_.th_dist) prob_flatness = 1.0;

        double prob_line = 1.0;
        if (candidate.line_variable > 8.0 )//&& candidate.line_dir > M_PI/4)// candidate.ground_elevation > elevation_thr_[concentric_idx])
        {
            // if (verbose_) cout << "line_dir: " << candidate.line_dir << endl;
            prob_line = 0.0;
        }

        bool revert = prob_line*prob_flatness > 0.5;

        if ( concentric_idx < params_.num_rings_of_interest )
        {
            if (revert)
            {
                if (params_.verbose)
                {
                    cout << "\033[1;32m" << "REVERT TRUE" << "\033[0m" << endl;
                }

                revert_pc_ += candidate.regionwise_ground;
                cloud_ground += candidate.regionwise_ground;
            }
            else
            {
                if (params_.verbose)
                {
                    cout << "\033[1;31m" << "FINAL REJECT" << "\033[0m" << endl;
                }
                reject_pc_ += candidate.regionwise_ground;
                cloud_nonground += candidate.regionwise_ground;
            }
        }
    }

    if (params_.verbose) std::cout << "\033[1;34m" << "====================================================" << "\033[0m" << endl;
}

void PatchWorkPP::extract_piecewiseground(const int zone_idx, const pcl::PointCloud<PointT> &src,
    pcl::PointCloud<PointT> &dst,
    pcl::PointCloud<PointT> &non_ground_dst)
{
    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();

    // 1. Region-wise Vertical Plane Fitting (R-VPF)
    // : removes potential vertical plane under the ground plane
    pcl::PointCloud<PointT> src_wo_verticals;
    src_wo_verticals = src;

    if (params_.enable_RVPF)
    {
        for (int i = 0; i < params_.num_iter; i++)
        {
            this->extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, params_.th_seeds_v);
            this->estimate_plane(ground_pc_);

            if (zone_idx == 0 && normal_(2) < params_.uprightness_thr)
            {
                pcl::PointCloud<PointT> src_tmp;
                src_tmp = src_wo_verticals;
                src_wo_verticals.clear();

                Eigen::MatrixXf points(src_tmp.points.size(), 3);
                int j = 0;
                for (auto &p:src_tmp.points) {
                    points.row(j++) << p.x, p.y, p.z;
                }
                // ground plane model
                Eigen::VectorXf result = points * normal_;

                for (int r = 0; r < result.rows(); r++) {
                    if (result[r] < params_.th_dist_v - d_ && result[r] > -params_.th_dist_v - d_) {
                        non_ground_dst.points.push_back(src_tmp[r]);
                        vertical_pc_.points.push_back(src_tmp[r]);
                    } else {
                        src_wo_verticals.points.push_back(src_tmp[r]);
                    }
                }
            }
            else break;
        }
    }

    this->extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
    this->estimate_plane(ground_pc_);

    // 2. Region-wise Ground Plane Fitting (R-GPF)
    // : fits the ground plane

    //pointcloud to matrix
    Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
    int j = 0;
    for (auto &p:src_wo_verticals.points) {
        points.row(j++) << p.x, p.y, p.z;
    }

    for (int i = 0; i < params_.num_iter; i++) {

        ground_pc_.clear();

        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++) {
            if (i < params_.num_iter - 1) {
                if (result[r] < params_.th_dist - d_ ) {
                    ground_pc_.points.push_back(src_wo_verticals[r]);
                }
            } else { // Final stage
                if (result[r] < params_.th_dist - d_ ) {
                    dst.points.push_back(src_wo_verticals[r]);
                } else {
                    non_ground_dst.points.push_back(src_wo_verticals[r]);
                }
            }
        }

        if (i < params_.num_iter -1) this->estimate_plane(ground_pc_);
        else this->estimate_plane(dst);
    }
}


void PatchWorkPP::set_ground_likelihood_estimation_status(
        const int zone_idx, const int ring_idx,
        const int concentric_idx,
        const double z_vec,
        const double z_elevation,
        const double ground_flatness) {
    if (z_vec > params_.uprightness_thr) { //orthogonal
        if (concentric_idx < params_.num_rings_of_interest) {
            if (z_elevation > params_.elevation_thr[concentric_idx]) {
                if (params_.flatness_thr[concentric_idx] > ground_flatness) {
                    likelihood = FLAT_ENOUGH;
                } else {
                    likelihood = TOO_HIGH_ELEVATION;
                }
            } else {
                likelihood = UPRIGHT_ENOUGH;
            }
        } else {
            likelihood = UPRIGHT_ENOUGH;
        }
    } else { // tilted
        likelihood = TOO_TILTED;
    }
}

double PatchWorkPP::get_likelihood()
{
    return likelihood;
}


void PatchWorkPP::calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev)
{
    if (vec.size() <= 1) return;

    mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

    for (int i=0; i<vec.size(); i++) { stdev += (vec.at(i)-mean)*(vec.at(i)-mean); }
    stdev /= vec.size()-1;
    stdev = sqrt(stdev);
}


double PatchWorkPP::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    // if (y >= 0) {
    //     return atan2(y, x); // 1, 2 quadrant
    // } else {
    //     return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    // }

    double angle = atan2(y, x);
    return angle > 0 ? angle : 2*M_PI+angle;
}


double PatchWorkPP::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}


void PatchWorkPP::pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm, pcl::PointCloud<PointT> &cloud_nonground) {

    for (int i=0; i<src.size(); i++) {
        if ((!noise_idxs_.empty()) &&(i == noise_idxs_.front())) {
            noise_idxs_.pop();
            continue;
        }

        PointT pt = src.points[i];

        double r = this->xy2radius(pt.x, pt.y);
        if ((r <= params_.max_range) && (r > params_.min_range)) {
            double theta = this->xy2theta(pt.x, pt.y);

            int zone_idx = 0;
            if ( r < params_.min_ranges[1] ) zone_idx = 0;
            else if ( r < params_.min_ranges[2] ) zone_idx = 1;
            else if ( r < params_.min_ranges[3] ) zone_idx = 2;
            else zone_idx = 3;

            int ring_idx = min(static_cast<int64_t>(((r - params_.min_ranges[zone_idx]) / params_.ring_sizes[zone_idx])), (params_.num_rings_each_zone[zone_idx] - 1));
            int sector_idx = min(static_cast<int64_t>((theta / params_.sector_sizes[zone_idx])), (params_.num_sectors_each_zone[zone_idx] - 1));

            czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
        }
        else {
            cloud_nonground.push_back(pt);
        }
    }

    if (params_.verbose ) cout << "[ CZM ] Divides pointcloud into the concentric zone model" << endl;
}

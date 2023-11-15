#include <Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>
#include <queue>
#include <mutex>
#include <algorithm>

#include "patchworkpp/utils.hpp"

#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55
#define FLAT_ENOUGH 0.2
#define TOO_HIGH_ELEVATION 0.0
#define TOO_TILTED 1.0
#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

typedef pcl::PointXYZI PointT;

using namespace std;

bool point_z_cmp(PointT a, PointT b) { return a.z < b.z; }

struct RevertCandidate
{
    int concentric_idx;
    int sector_idx;
    double ground_flatness;
    double line_variable;
    Eigen::Vector4f pc_mean;
    pcl::PointCloud<PointT> regionwise_ground;

    RevertCandidate(int c_idx_, int s_idx_, double flatness_, double line_var_, Eigen::Vector4f pc_mean_, pcl::PointCloud<PointT> ground_) 
     : concentric_idx(c_idx_), sector_idx(s_idx_), ground_flatness(flatness_), line_variable(line_var_), pc_mean(pc_mean_), regionwise_ground(ground_) {}
};

struct PatchWorkParams
{
	int num_iter;
    int num_lpr;
    int num_min_pts;
    int num_zones;
    int num_rings_of_interest;


    // For visualization
    bool visualise;
    
    vector<double> ring_sizes;
    vector<double> min_ranges;
    vector<double> elevation_thr;
    vector<double> flatness_thr;
    vector<int64_t> num_sectors_each_zone;
    vector<int64_t> num_rings_each_zone;
    vector<double> sector_sizes;

    int max_flatness_storage, max_elevation_storage;
    std::vector<double> update_flatness[4];
    std::vector<double> update_elevation[4];

    double sensor_height;
    double th_seeds;
    double th_dist;
    double th_seeds_v;
    double th_dist_v;
    double max_range;
    double min_range;
    double uprightness_thr;
    double adaptive_seed_selection_margin;
    double min_range_z2; // 12.3625
    double min_range_z3; // 22.025
    double min_range_z4; // 41.35
    double RNR_ver_angle_thr;
    double RNR_intensity_thr;

    bool verbose;
    bool enable_RNR;
    bool enable_RVPF;
    bool enable_TGR;
};

class PatchWorkPP
{
    public:
        typedef std::vector<pcl::PointCloud<PointT>> Ring;
        typedef std::vector<Ring> Zone;

        PatchWorkPP(PatchWorkParams & params);

        void reflected_noise_removal(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud_nonground);
        
        void extract_piecewiseground(
                const int zone_idx, const pcl::PointCloud<PointT> &src,
                pcl::PointCloud<PointT> &dst,
                pcl::PointCloud<PointT> &non_ground_dst);
        
        void set_ground_likelihood_estimation_status(
                const int zone_idx, const int ring_idx,
                const int concentric_idx,
                const double z_vec,
                const double z_elevation,
                const double ground_flatness);

        void temporal_ground_revert(pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground,
                                std::vector<double> ring_flatness, std::vector<RevertCandidate> candidates,
                                int concentric_idx);
        
        void update_elevation_thr();
        
        void update_flatness_thr();
		
        double get_likelihood();
		
		bool visualise_from_node = false;

        pcl::PointCloud<PointT> revert_pc_, reject_pc_, noise_pc_, vertical_pc_, ground_pc_;
        pcl::PointCloud<pcl::PointXYZINormal> normals_;
        pcl::PointCloud<PointT> regionwise_ground_, regionwise_nonground_;

		int num_polygons;
		double likelihood;
    
        vector<Zone> ConcentricZoneModel_;
        VectorXf normal_;
        MatrixXf pnormal_;
        VectorXf singular_values_;
        Eigen::Matrix3f cov_;
        Eigen::Vector4f pc_mean_;
	
	private:
        // class methods
        void initialise();


        void initialise_zone(Zone &z, int num_sectors, int num_rings);

        void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);
        
        // CZM - Concentric Zone Model
        void flush_patches(vector<Zone> &czm);

        void pc2czm(const pcl::PointCloud<PointT> &src, vector<Zone> &czm, pcl::PointCloud<PointT> &cloud_nonground);

        void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);


        double xy2theta(const double &x, const double &y);

        double xy2radius(const double &x, const double &y);

        void estimate_plane(const pcl::PointCloud<PointT> &ground);


        void extract_initial_seeds(
                const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
                pcl::PointCloud<PointT> &init_seeds);
        
        void extract_initial_seeds(
                const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
                pcl::PointCloud<PointT> &init_seeds, double th_seed);


        // class variables
		PatchWorkParams params_;
        std::recursive_mutex mutex_;

        float d_;



        queue<int> noise_idxs_;




};

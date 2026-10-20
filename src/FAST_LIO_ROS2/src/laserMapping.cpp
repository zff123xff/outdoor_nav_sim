// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <sstream>
#include <limits>
#include <atomic>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include "dynamic_filter.hpp"
#include "visibility_filter.hpp"
#include "region_map.hpp"
#include "observed_map_update.hpp"
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#ifdef FAST_LIO_HAS_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
std::atomic<int> publish_count{0};
std::atomic<size_t> imu_received_count{0}, imu_rejected_count{0};
size_t imu_consumed_count = 0;
double max_received_imu_gap = 0.;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;
double output_map_leaf_size = 0.2;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// Opt-in observations only: diagnostics never change the filter state.
std::ofstream lio_diagnostics;
size_t diagnostic_scan = 0;
int diagnostic_iteration = 0;
double diagnostic_previous_stamp = std::numeric_limits<double>::quiet_NaN();
std::vector<unsigned char> diagnostic_match_reason;

void diagnostic_number(std::ostream &out, double value) {
    if (std::isfinite(value)) out << std::setprecision(17) << value;
    else out << "null";
}

template<class Vector>
void diagnostic_vector(std::ostream &out, const Vector &value, int count) {
    out << '[';
    for (int i=0;i<count;++i) {
        if(i) out << ',';
        diagnostic_number(out,value[i]);
    }
    out << ']';
}

void write_lio_diagnostics(const char *phase, const state_ikfom &state,
                           const std::string &extra="") {
    if (!lio_diagnostics.is_open()) return;
    auto &out=lio_diagnostics;
    out << "{\"scan\":" << diagnostic_scan << ",\"phase\":\"" << phase
        << "\",\"stamp\":";
    diagnostic_number(out,Measures.lidar_end_time);
    out << ",\"scan_begin\":"; diagnostic_number(out,Measures.lidar_beg_time);
    out << ",\"previous_state_stamp\":"; diagnostic_number(out,diagnostic_previous_stamp);
    out << ",\"position\":"; diagnostic_vector(out,state.pos,3);
    out << ",\"orientation_xyzw\":"; diagnostic_vector(out,state.rot.coeffs(),4);
    out << ",\"velocity\":"; diagnostic_vector(out,state.vel,3);
    out << ",\"gyro_bias\":"; diagnostic_vector(out,state.bg,3);
    out << ",\"accel_bias\":"; diagnostic_vector(out,state.ba,3);
    out << ",\"gravity\":"; diagnostic_vector(out,state.grav,3);
    const auto covariance=kf.get_P();
    out << ",\"covariance_diagonal\":";
    diagnostic_vector(out,covariance.diagonal(),covariance.rows());
    out << ",\"imu_count\":" << Measures.imu.size();
    if(!Measures.imu.empty()) {
        out << ",\"imu_first_stamp\":";
        diagnostic_number(out,get_time_sec(Measures.imu.front()->header.stamp));
        out << ",\"imu_last_stamp\":";
        diagnostic_number(out,get_time_sec(Measures.imu.back()->header.stamp));
    }
    size_t pending_imu, pending_lidar;
    double received_stamp, received_gap;
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        pending_imu=imu_buffer.size(); pending_lidar=lidar_buffer.size();
        received_stamp=last_timestamp_imu; received_gap=max_received_imu_gap;
    }
    out << ",\"imu_queue_size\":" << pending_imu
        << ",\"lidar_queue_size\":" << pending_lidar
        << ",\"latest_received_imu_stamp\":";
    diagnostic_number(out,received_stamp);
    out << ",\"max_received_imu_gap\":";
    diagnostic_number(out,received_gap);
    out << ",\"imu_received_total\":" << imu_received_count.load()
        << ",\"imu_rejected_total\":" << imu_rejected_count.load()
        << ",\"imu_consumed_total\":" << imu_consumed_count
        << ",\"imu_coverage\":\"" << p_imu->coverage().reason << "\""
        << ",\"imu_propagation_max_gap\":";
    diagnostic_number(out,p_imu->coverage().max_gap);
    out << ",\"raw_points\":" << (Measures.lidar ? Measures.lidar->size() : 0)
        << ",\"retained_points\":" << feats_undistort->size()
        << ",\"map_points\":" << ikdtree.validnum() << extra << "}\n";
    out.flush();
}

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    // Preprocessing stays in the serial processing group, outside the IMU lock.
    p_pre->process(msg, ptr);
    const double preprocess_time=omp_get_wtime()-preprocess_start_time;
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    if(scan_count<MAXN) s_plot11[scan_count] = preprocess_time;
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
#ifdef FAST_LIO_HAS_LIVOX
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    const double preprocess_time=omp_get_wtime()-preprocess_start_time;
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    if(scan_count<MAXN) s_plot11[scan_count] = preprocess_time;
    sig_buffer.notify_all();
}

#endif

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    const auto &acc=msg_in->linear_acceleration;
    const auto &gyro=msg_in->angular_velocity;
    if(!std::isfinite(acc.x)||!std::isfinite(acc.y)||!std::isfinite(acc.z)||
       !std::isfinite(gyro.x)||!std::isfinite(gyro.y)||!std::isfinite(gyro.z)) {
        ++imu_rejected_count;
        return;
    }
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    std::lock_guard<std::mutex> lock(mtx_buffer);
    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    if(timestamp==last_timestamp_imu) { ++imu_rejected_count; return; }
    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }
    else if(last_timestamp_imu>=0) max_received_imu_gap=std::max(max_received_imu_gap,timestamp-last_timestamp_imu);
    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    ++imu_received_count;
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    meas.imu = fast_lio::take_imu_until(imu_buffer,lidar_end_time,[](const auto &sample) {
        return get_time_sec(sample->header.stamp);
    });
    imu_consumed_count += meas.imu.size();

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

std::unique_ptr<dynamic_lio::RegionMap> region_map;
bool region_enabled = false;
std::vector<bool> region_scan_admissible(100000,true);
Eigen::Isometry3d region_prediction_pose=Eigen::Isometry3d::Identity();
// Unknown/new geometry is admissible; previously dynamic geometry waits for recovery.
bool region_map_eligible(const Eigen::Vector3d &world) {
    return !region_enabled || !region_map || region_map->eligible(world);
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        const auto &point=feats_down_world->points[i];
        if (!region_scan_admissible[i] || !region_map_eligible(Eigen::Vector3d(point.x,point.y,point.z))) continue;
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI());
    laserCloudWorld->reserve(size);
    for (int i = 0; i < size; i++)
    {
        PointType world{};
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], &world);
        const auto &p=laserCloudFullRes->points[i];
        if(region_map_eligible(region_prediction_pose*Eigen::Vector3d(p.x,p.y,p.z)) &&
           region_map_eligible(Eigen::Vector3d(world.x,world.y,world.z))) laserCloudWorld->push_back(world);
    }
    *pcl_wait_pub += *laserCloudWorld;
    // Filter the accumulated world map, including overlap between scans.
    PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
    downSizeFilterMap.setInputCloud(pcl_wait_pub);
    downSizeFilterMap.filter(*filtered);
    pcl_wait_pub.swap(filtered);

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

std::vector<double> dynamic_match_weights(100000, 1.);
std::vector<double> dynamic_effect_weights(100000, 1.);

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0;
    if(lio_diagnostics.is_open()) diagnostic_match_reason.assign(feats_down_size,0);

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) {
            if(lio_diagnostics.is_open()) diagnostic_match_reason[i]=1;
            continue;
        }

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            if(lio_diagnostics.is_open()) diagnostic_match_reason[i]=3;
            double region_weight=1.;
            if (region_enabled && region_map) {
                region_weight=region_map->weight(p_global);
                std::vector<double> target_weights;
                for (const auto &target:points_near)
                    target_weights.push_back(region_map->weight(Eigen::Vector3d(target.x,target.y,target.z)));
                std::sort(target_weights.begin(),target_weights.end());
                if(!target_weights.empty()) region_weight=std::min(region_weight,target_weights[target_weights.size()/2]);
            }
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
                dynamic_effect_weights[i] = std::min(dynamic_match_weights[i],region_weight);
                if(lio_diagnostics.is_open()) diagnostic_match_reason[i]=4;
            }
        }
        else if(lio_diagnostics.is_open()) diagnostic_match_reason[i]=2;
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            dynamic_effect_weights[effct_feat_num] = dynamic_effect_weights[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        if(lio_diagnostics.is_open()) {
            std::ostringstream details;
            details << ",\"iteration\":" << diagnostic_iteration++
                << ",\"effective_points\":0,\"downsampled_points\":" << feats_down_size;
            for(int reason=1;reason<=3;++reason) details << ",\"reject_" << reason << "\":"
                << std::count(diagnostic_match_reason.begin(),diagnostic_match_reason.end(),reason);
            write_lio_diagnostics("matching",s,details.str());
        }
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
        const double scale = std::sqrt(dynamic_effect_weights[i]);
        ekfom_data.h_x.row(i) *= scale;
        ekfom_data.h(i) *= scale;
    }
    solve_time += omp_get_wtime() - solve_start_;
    if(lio_diagnostics.is_open()) {
        std::ostringstream details;
        details << std::setprecision(17) << ",\"iteration\":" << diagnostic_iteration++
            << ",\"downsampled_points\":" << feats_down_size
            << ",\"effective_points\":" << effct_feat_num << ",\"mean_abs_residual\":";
        diagnostic_number(details,res_mean_last);
        double weight_sum=0.;
        for(int i=0;i<effct_feat_num;++i) weight_sum+=dynamic_effect_weights[i];
        details << ",\"weight_sum\":"; diagnostic_number(details,weight_sum);
        for(int reason=1;reason<=3;++reason) details << ",\"reject_" << reason << "\":"
            << std::count(diagnostic_match_reason.begin(),diagnostic_match_reason.end(),reason);
        const Eigen::Matrix<double,6,6> information=
            ekfom_data.h_x.leftCols(6).transpose()*ekfom_data.h_x.leftCols(6);
        details << ",\"pose_information_row_major\":[";
        for(int row=0;row<6;++row) for(int col=0;col<6;++col) {
            if(row || col) details << ',';
            diagnostic_number(details,information(row,col));
        }
        details << ']';
        write_lio_diagnostics("matching",s,details.str());
    }
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("diagnostics.file", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<int>("common.imu_queue_depth", 2000);
        this->declare_parameter<double>("common.imu_max_gap", .05);
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("output_map_leaf_size", 0.2);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        const auto diagnostics_file=this->get_parameter("diagnostics.file").as_string();
        if(!diagnostics_file.empty()) {
            lio_diagnostics.open(diagnostics_file,std::ios::out);
            if(!lio_diagnostics) throw std::runtime_error("Cannot open LIO diagnostics file: "+diagnostics_file);
        }
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        output_map_leaf_size = this->get_parameter("output_map_leaf_size").as_double();
        if (!std::isfinite(output_map_leaf_size) || output_map_leaf_size <= 0)
            throw std::runtime_error("output_map_leaf_size must be finite and positive");
        downSizeFilterMap.setLeafSize(output_map_leaf_size, output_map_leaf_size, output_map_leaf_size);
        dynamic_enabled_ = declare_parameter<bool>("dynamic.enabled", false);
        cluster_tracking_enabled_ = declare_parameter<bool>("dynamic.cluster_tracking_enabled", false);
        dynamic_lio::Config dynamic_config;
        dynamic_config.voxel = declare_parameter<double>("dynamic.voxel", 0.12);
        dynamic_config.cluster_tolerance = declare_parameter<double>("dynamic.cluster_tolerance", 0.30);
        dynamic_config.range = declare_parameter<double>("dynamic.range", 25.0);
        dynamic_config.min_cluster_points = declare_parameter<int>("dynamic.min_cluster_points", 12);
        dynamic_config.max_cluster_points = declare_parameter<int>("dynamic.max_cluster_points", 4000);
        dynamic_config.max_object_width = declare_parameter<double>("dynamic.max_object_width", 3.0);
        dynamic_config.max_object_height = declare_parameter<double>("dynamic.max_object_height", 2.8);
        dynamic_config.ground_distance = declare_parameter<double>("dynamic.ground_distance", 0.08);
        dynamic_config.association_distance = declare_parameter<double>("dynamic.association_distance", 0.8);
        dynamic_config.min_speed = declare_parameter<double>("dynamic.min_speed", 0.20);
        dynamic_config.min_displacement = declare_parameter<double>("dynamic.min_displacement", 0.25);
        dynamic_config.observation_time = declare_parameter<double>("dynamic.observation_time", 0.8);
        dynamic_config.confirmations = declare_parameter<int>("dynamic.confirmations", 3);
        dynamic_config.padding = declare_parameter<double>("dynamic.padding", 0.10);
        dynamic_config.track_timeout = declare_parameter<double>("dynamic.track_timeout", 1.0);
        dynamic_config.history_seconds = declare_parameter<double>("dynamic.history_seconds", 5.0);
        dynamic_config.hold_seconds = declare_parameter<double>("dynamic.hold_seconds", 2.0);
        dynamic_config.max_pose_step = declare_parameter<double>("dynamic.max_pose_step", 0.8);
        dynamic_config.max_pose_angle = declare_parameter<double>("dynamic.max_pose_angle", 0.45);
        dynamic_filter_.reset(new dynamic_lio::Filter(dynamic_config));
        dynamic_lio::VisibilityConfig visibility;
        visibility.voxel = declare_parameter<double>("dynamic.visibility_voxel", 0.20);
        visibility.margin = declare_parameter<double>("dynamic.visibility_margin", 0.30);
        visibility.range = dynamic_config.range;
        visibility.history = dynamic_config.history_seconds;
        visibility.min_observation = declare_parameter<double>("dynamic.free_observation_time", 0.4);
        visibility.confirmations = declare_parameter<int>("dynamic.free_confirmations", 3);
        visibility.increment = declare_parameter<double>("dynamic.confidence_increment", 0.35);
        visibility.decrement = declare_parameter<double>("dynamic.confidence_decrement", 0.15);
        visibility.remove_threshold = declare_parameter<double>("dynamic.remove_confidence", 0.8);
        visibility.backend_threshold = declare_parameter<double>("dynamic.backend_max_confidence", 0.2);
        const int visibility_capacity=declare_parameter<int>("dynamic.max_visibility_voxels", 300000);
        if(visibility_capacity<=0) throw std::invalid_argument("Visibility capacity must be positive");
        visibility.capacity=static_cast<size_t>(visibility_capacity);
        visibility_filter_.reset(new dynamic_lio::VisibilityFilter(visibility));
        const bool region_requested=declare_parameter<bool>("dynamic.region.enabled", true);
        region_enabled = dynamic_enabled_ && region_requested;
        dynamic_lio::RegionConfig region;
        region.voxel=declare_parameter<double>("dynamic.region.voxel", .5);
        region.surface_voxel=visibility.voxel; region.range=visibility.range;
        region.dynamic_weight=declare_parameter<double>("dynamic.region.dynamic_weight", .2);
        region.recovering_weight=declare_parameter<double>("dynamic.region.recovering_weight", .8);
        region.dynamic_hold=declare_parameter<double>("dynamic.region.dynamic_hold", 2.);
        region.static_time=declare_parameter<double>("dynamic.region.static_time", 5.);
        region.static_observations=declare_parameter<int>("dynamic.region.static_observations", 20);
        region.observation_gap=declare_parameter<double>("dynamic.region.observation_gap", .5);
        region.overlap=declare_parameter<double>("dynamic.region.surface_overlap", .8);
        region.history=declare_parameter<double>("dynamic.region.history_seconds", 120.);
        const int region_capacity=declare_parameter<int>("dynamic.region.max_regions", 200000);
        if(region_capacity<=0) throw std::invalid_argument("Invalid region capacity");
        region.capacity=region_capacity; region_map.reset(new dynamic_lio::RegionMap(region));
        map_update_enabled_ = declare_parameter<bool>("map_update.enabled", false);
        const double update_voxel = declare_parameter<double>("map_update.voxel", 0.20);
        const double update_margin = declare_parameter<double>("map_update.surface_margin", 0.30);
        const int update_confirmations = declare_parameter<int>("map_update.confirmations", 2);
        const double update_range = declare_parameter<double>("map_update.range", 25.0);
        const int update_capacity = declare_parameter<int>("map_update.max_voxels", 2000000);
        if (update_capacity < 1) throw std::invalid_argument("map_update.max_voxels must be positive");
        map_updater_.reset(new dynamic_lio::MapUpdater(update_voxel, update_margin, update_confirmations, update_range, update_capacity));
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu->set_max_imu_gap(this->get_parameter("common.imu_max_gap").as_double());

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        recovery_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/go2/recovering", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr message) {
                if (!message->data || invalid_after_reset_) return;
                invalid_after_reset_ = true;
                std::lock_guard<std::mutex> lock(mtx_buffer);
                lidar_buffer.clear(); time_buffer.clear(); imu_buffer.clear();
                RCLCPP_WARN(get_logger(),
                    "Simulation pose reset: mapping stopped to protect the existing map. "
                    "Restart go2_mapping_launch.py before continuing mapping.");
            });
        if (p_pre->lidar_type == AVIA)
        {
#ifdef FAST_LIO_HAS_LIVOX
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
#else
            throw std::runtime_error("Livox CustomMsg support requires livox_ros_driver2; Go2 uses lidar_type 5");
#endif
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::UniquePtr message) {
                    if (!invalid_after_reset_) standard_pcl_cbk(std::move(message));
                });
        }
        // Only sensor reception runs concurrently. KF/tree/map callbacks remain
        // mutually exclusive in the default group, preserving their ownership.
        const int imu_queue_depth=this->get_parameter("common.imu_queue_depth").as_int();
        if(imu_queue_depth<1) throw std::invalid_argument("common.imu_queue_depth must be positive");
        imu_callback_group_=this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group=imu_callback_group_;
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(rclcpp::KeepLast(imu_queue_depth)),
            [this](sensor_msgs::msg::Imu::UniquePtr message) {
                if (!invalid_after_reset_) imu_cbk(std::move(message));
            }, imu_options);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        region_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic/region_weight", 5);
        confidence_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic/confidence", 5);
        dynamic_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic/points", 5);
        erase_boxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic/erase_boxes", 100);
        dynamic_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic/objects", 5);
        dynamic_status_pub_ = create_publisher<std_msgs::msg::String>("/dynamic/status", 5);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    void timer_callback()
    {
        if (invalid_after_reset_) return;
        if(sync_packages(Measures))
        {
            // The boundary sample is retained by ImuProcess; release the rest
            // once this scan finishes, including all early-return paths.
            struct ReleaseImuBatch {
                MeasureGroup &packet;
                ~ReleaseImuBatch() { packet.imu.clear(); }
            } release_imu{Measures};
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            ++diagnostic_scan;
            diagnostic_iteration=0;
            write_lio_diagnostics("posterior_previous",kf.get_x());
            if(!p_imu->Process(Measures, kf, feats_undistort)) {
                const auto &coverage=p_imu->coverage();
                write_lio_diagnostics("imu_scan_skipped",kf.get_x());
                if(coverage.reason!="initializing")
                    RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,
                        "Scan skipped: %s, maximum IMU gap %.6f s. No laser/map update; "
                        "restart mapping to reinitialize after missing IMU data.",
                        coverage.reason.c_str(),coverage.max_gap);
                return;
            }
            state_point = kf.get_x();
            write_lio_diagnostics("imu_prediction",state_point);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            if (dynamic_enabled_) filter_dynamic_points();
            else if (map_update_enabled_) update_observed_map(*feats_undistort);
            write_lio_diagnostics("after_dynamic_filter",state_point);
            if (feats_undistort->size() < 5) return;

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            for (int i=0; i<feats_down_size; ++i) {
                const auto &p=feats_down_body->points[i];
                dynamic_match_weights[i] = dynamic_enabled_ ? visibility_filter_->weight(
                    lidar_world_pose()*Eigen::Vector3d(p.x,p.y,p.z)) : 1.;
                const Eigen::Vector3d world=lidar_world_pose()*Eigen::Vector3d(p.x,p.y,p.z);
                if(region_enabled) dynamic_match_weights[i]=std::min(dynamic_match_weights[i],region_map->weight(world));
                region_scan_admissible[i]=region_map_eligible(world);
            }
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    PointVector admitted;
                    for(int i=0;i<feats_down_size;++i) if(region_scan_admissible[i]) admitted.push_back(feats_down_world->points[i]);
                    if(admitted.size()>5) ikdtree.Build(admitted);
                    remember_static_scan();
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            write_lio_diagnostics("laser_posterior",state_point);
            diagnostic_previous_stamp=Measures.lidar_end_time;
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            remember_static_scan();
            write_lio_diagnostics("after_map_insert",state_point);
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) {
                if (dynamic_enabled_) {
                    const auto frontend_cloud=feats_undistort;
                    PointCloudXYZI::Ptr trusted(new PointCloudXYZI());
                    for (const auto &p : *frontend_cloud) if (visibility_filter_->backendStatic(
                            dynamic_prediction_pose_*Eigen::Vector3d(p.x,p.y,p.z)) && region_map_eligible(
                            dynamic_prediction_pose_*Eigen::Vector3d(p.x,p.y,p.z)) && region_map_eligible(
                            lidar_world_pose()*Eigen::Vector3d(p.x,p.y,p.z))) trusted->push_back(p);
                    feats_undistort=trusted;
                    publish_frame_body(pubLaserCloudFull_body_);
                    feats_undistort=frontend_cloud;
                } else publish_frame_body(pubLaserCloudFull_body_);
            }
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (invalid_after_reset_) return;
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            if (pcl_wait_pub->empty()) {
                res->success = false;
                res->message = "Map is empty; wait for mapping and enable publish.map_en.";
                return;
            }
            try {
                save_to_pcd();
                res->success = true;
                res->message = "Map saved.";
            } catch (const std::exception &e) {
                res->success = false;
                res->message = e.what();
            }
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    void filter_dynamic_points()
    {
        Eigen::Isometry3d world_from_lidar = Eigen::Isometry3d::Identity();
        world_from_lidar.linear() = state_point.rot.toRotationMatrix() * state_point.offset_R_L_I.toRotationMatrix();
        world_from_lidar.translation() = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        dynamic_prediction_pose_=world_from_lidar;
        region_prediction_pose=world_from_lidar;
        const PointCloudXYZI original = *feats_undistort;
        update_observed_map(original);  // All returns, including moving occluders, bound visibility.
        dynamic_lio::Result result;
        result.state = "visibility";
        if (cluster_tracking_enabled_)
            result = dynamic_filter_->process(*feats_undistort, world_from_lidar, lidar_end_time);
        const size_t cluster_removed_points = result.dynamic_indices.size();
        const auto visibility_boxes = visibility_filter_->process(*feats_undistort, original, world_from_lidar, lidar_end_time);
        result.erase_boxes.insert(result.erase_boxes.end(), visibility_boxes.begin(), visibility_boxes.end());
        result.erase_boxes.insert(result.erase_boxes.end(), observed_erase_boxes_.begin(), observed_erase_boxes_.end());
        // Pair this geometry invalidation with exactly the same odometry/cloud stamp in PGO.
        visualization_msgs::msg::MarkerArray erasures;
        visualization_msgs::msg::Marker metadata;
        metadata.header.frame_id="camera_init"; metadata.header.stamp=get_ros_time(lidar_end_time);
        metadata.action=visualization_msgs::msg::Marker::DELETEALL;
        erasures.markers.push_back(metadata);
        for (const auto &box : result.erase_boxes) {
            visualization_msgs::msg::Marker m; m.header=metadata.header;
            m.type=visualization_msgs::msg::Marker::CUBE; m.action=visualization_msgs::msg::Marker::ADD;
            m.id=erasures.markers.size(); m.pose.orientation.w=1.;
            const auto center=(box.low+box.high)/2; const auto size=box.high-box.low;
            m.pose.position.x=center.x(); m.pose.position.y=center.y(); m.pose.position.z=center.z();
            m.scale.x=size.x(); m.scale.y=size.y(); m.scale.z=size.z(); erasures.markers.push_back(m);
        }
        erase_boxes_pub_->publish(erasures);
        if (!result.erase_boxes.empty()) {
            vector<BoxPointType> boxes;
            for (const auto &box : result.erase_boxes) {
                BoxPointType value;
                for (int axis = 0; axis < 3; ++axis) {
                    value.vertex_min[axis] = box.low[axis]; value.vertex_max[axis] = box.high[axis];
                }
                boxes.push_back(value);
            }
            if (ikdtree.Root_Node != nullptr) ikdtree.Delete_Point_Boxes(boxes);
            dynamic_lio::Filter::erase(*pcl_wait_pub, result.erase_boxes);
            dynamic_lio::Filter::erase(*pcl_wait_save, result.erase_boxes);
        }
        std::vector<bool> already_removed(original.size(),false);
        for(size_t i:result.dynamic_indices) already_removed[i]=true;
        PointCloudXYZI confidence_points;
        size_t uncertain_points=0;
        double max_confidence=0.;
        for(size_t i=0;i<original.size();++i) {
            const auto &p=original[i];
            const Eigen::Vector3d world=world_from_lidar*Eigen::Vector3d(p.x,p.y,p.z);
            const double confidence=already_removed[i]?1.:visibility_filter_->score(world);
            max_confidence=std::max(max_confidence,confidence);
            if(confidence>0 && !already_removed[i] && !visibility_filter_->isDynamic(world)) ++uncertain_points;
            if(visibility_filter_->isDynamic(world) && !already_removed[i]) result.dynamic_indices.push_back(i);
            PointType q=p;q.x=world.x();q.y=world.y();q.z=world.z();q.intensity=confidence;
            confidence_points.push_back(q);
        }
        if(region_enabled) {
            std::vector<Eigen::Vector3d> moving;
            for(size_t index:result.dynamic_indices) {
                const auto &p=original[index];moving.push_back(world_from_lidar*Eigen::Vector3d(p.x,p.y,p.z));
            }
            region_map->process(original,world_from_lidar,moving,observed_erase_boxes_,lidar_end_time);
        }
        size_t region_quarantined=0;
        if(region_enabled) {
            PointCloudXYZI region_points;
            for(const auto &p:original) {
                const Eigen::Vector3d world=world_from_lidar*Eigen::Vector3d(p.x,p.y,p.z);
                PointType q=p;q.x=world.x();q.y=world.y();q.z=world.z();q.intensity=region_map->weight(world);
                region_points.push_back(q);
                if(!region_map->eligible(world)) ++region_quarantined;
            }
            sensor_msgs::msg::PointCloud2 region_message;
            pcl::toROSMsg(region_points,region_message);region_message.header=metadata.header;
            region_cloud_pub_->publish(region_message);
        }
        sensor_msgs::msg::PointCloud2 confidence_message;
        pcl::toROSMsg(confidence_points,confidence_message);
        confidence_message.header=metadata.header;confidence_cloud_pub_->publish(confidence_message);
        PointCloudXYZI dynamic_points;
        for (size_t index : result.dynamic_indices) {
            PointType point = original[index];
            const Eigen::Vector3d world = world_from_lidar * Eigen::Vector3d(point.x, point.y, point.z);
            point.x = world.x(); point.y = world.y(); point.z = world.z();
            dynamic_points.push_back(point);
        }
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(dynamic_points, message);
        message.header.frame_id = "camera_init"; message.header.stamp = get_ros_time(lidar_end_time);
        dynamic_cloud_pub_->publish(message);
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.header = message.header; clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);
        for (const auto &detection : result.moving) {
            visualization_msgs::msg::Marker marker;
            marker.header = message.header; marker.ns = "moving_objects"; marker.id = detection.id;
            marker.type = visualization_msgs::msg::Marker::CUBE; marker.action = visualization_msgs::msg::Marker::ADD;
            const Eigen::Vector3d center = (detection.box.low+detection.box.high)/2;
            const Eigen::Vector3d size = detection.box.high-detection.box.low;
            marker.pose.position.x = center.x(); marker.pose.position.y = center.y(); marker.pose.position.z = center.z();
            marker.pose.orientation.w = 1;
            marker.scale.x = std::max(0.05, size.x()); marker.scale.y = std::max(0.05, size.y()); marker.scale.z = std::max(0.05, size.z());
            marker.color.r = 1; marker.color.a = 0.35; markers.markers.push_back(marker);
        }
        dynamic_markers_pub_->publish(markers);
        std_msgs::msg::String status;
        std::ostringstream text;
        text << std::setprecision(17) << "{\"stamp\":" << lidar_end_time << ",\"state\":\"" << result.state
             << "\",\"input_points\":" << original.size() << ",\"static_points\":" << feats_undistort->size()
             << ",\"dynamic_points\":" << result.dynamic_indices.size() << ",\"clusters\":" << result.clusters
             << ",\"cluster_removed_points\":" << cluster_removed_points
             << ",\"visibility_removed_points\":" << result.dynamic_indices.size()-cluster_removed_points
             << ",\"tracks\":" << result.tracks << ",\"moving_objects\":" << result.moving.size()
             << ",\"erase_boxes\":" << result.erase_boxes.size()
             << ",\"max_dynamic_confidence\":" << max_confidence
             << ",\"uncertain_points\":" << uncertain_points
             << ",\"visibility_cells\":" << visibility_filter_->cells()
             << ",\"visibility_removed_voxels\":" << visibility_boxes.size()
             << ",\"observed_free_voxels\":" << observed_free_voxels_
             << ",\"tracked_map_voxels\":" << map_updater_->occupiedVoxels()
             << ",\"region_quarantined_points\":" << region_quarantined
             << ",\"region_unknown\":" << (region_enabled?region_map->stats().unknown:0)
             << ",\"region_static\":" << (region_enabled?region_map->stats().stable:0)
             << ",\"region_dynamic\":" << (region_enabled?region_map->stats().dynamic:0)
             << ",\"region_recovering\":" << (region_enabled?region_map->stats().recovering:0)
             << ",\"region_at_capacity\":" << (region_enabled&&region_map->stats().at_capacity?"true":"false")
             << ",\"map_update_at_capacity\":" << (map_updater_->atCapacity() ? "true" : "false") << "}";
        status.data = text.str(); dynamic_status_pub_->publish(status);
    }
    Eigen::Isometry3d lidar_world_pose() const {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = state_point.rot.toRotationMatrix() * state_point.offset_R_L_I.toRotationMatrix();
        pose.translation() = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        return pose;
    }
    void update_observed_map(const PointCloudXYZI &raw) {
        observed_erase_boxes_.clear();
        if (!map_update_enabled_) return;
        const auto removed = map_updater_->clearObservedFree(raw, lidar_world_pose(), lidar_end_time);
        observed_free_voxels_ = removed.size();
        observed_erase_boxes_=removed;
        if (removed.empty()) return;
        vector<BoxPointType> boxes;
        for (const auto &box : removed) {
            BoxPointType value;
            for (int axis = 0; axis < 3; ++axis) {
                value.vertex_min[axis] = box.low[axis]; value.vertex_max[axis] = box.high[axis];
            }
            boxes.push_back(value);
        }
        if (ikdtree.Root_Node != nullptr) ikdtree.Delete_Point_Boxes(boxes);
        map_updater_->eraseCleared(*pcl_wait_pub, removed);
        map_updater_->eraseCleared(*pcl_wait_save, removed);
    }
    void remember_static_scan() {
        if (map_update_enabled_) {
            PointCloudXYZI admitted;
            for(const auto &p:*feats_undistort) if(region_map_eligible(
                region_prediction_pose*Eigen::Vector3d(p.x,p.y,p.z)) && region_map_eligible(
                lidar_world_pose()*Eigen::Vector3d(p.x,p.y,p.z))) admitted.push_back(p);
            map_updater_->observeStatic(admitted,lidar_world_pose());
        }
    }
    std::vector<dynamic_lio::Box> observed_erase_boxes_;
    bool map_update_enabled_{false};
    size_t observed_free_voxels_{0};
    std::unique_ptr<dynamic_lio::MapUpdater> map_updater_;
    bool dynamic_enabled_{false};
    bool cluster_tracking_enabled_ = false;
    std::unique_ptr<dynamic_lio::Filter> dynamic_filter_;
    std::unique_ptr<dynamic_lio::VisibilityFilter> visibility_filter_;
    Eigen::Isometry3d dynamic_prediction_pose_=Eigen::Isometry3d::Identity();
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr erase_boxes_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_cloud_pub_,confidence_cloud_pub_,region_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dynamic_status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_subscription_;
    std::atomic<bool> invalid_after_reset_{false};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
#ifdef FAST_LIO_HAS_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
#endif

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto node=std::make_shared<LaserMappingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);
    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}

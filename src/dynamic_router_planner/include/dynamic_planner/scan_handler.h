#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include "utility.h"

struct ScanHandlerParams {
    ScanHandlerParams() = default;
    float sensor_range;
    float voxel_size;
    float ceil_height;
    int inflate_size;
};

enum GridStatus {
    EMPTY = 0,
    SCAN  = 1,
    OBS   = 2,
    RAY   = 3,
};

class ScanHandler {

public:
    ScanHandler() = default;
    ~ScanHandler() = default;

    void Init(const ScanHandlerParams& params);

    void UpdateRobotPosition(const Point3D& odom_pos);
    
    void SetCurrentScanCloud(const PointCloudPtr& scanCloudIn);

    void SetSurroundObsCloud(const PointCloudPtr& obsCloudIn, 
                             const bool& is_fiWlter_cloud=false);

    void ExtractDyObsCloud(const PointCloudPtr& cloudIn, const PointCloudPtr& dyObsCloudOut);

    void GridVisualCloud(const PointCloudPtr& cloudOut, const GridStatus& type);

    void ReInitGrids();

    inline PCLPoint Ind2PCLPoint(const int& ind) {
        PCLPoint pcl_p;
        Eigen::Vector3d pos = voxel_grids_->Ind2Pos(ind);
        pcl_p.x = pos.x(), pcl_p.y = pos.y(), pcl_p.z = pos.z();
        return pcl_p;
    }

private:
    ScanHandlerParams scan_params_;
    Eigen::Vector3i center_sub_;
    int row_num_, col_num_, level_num_;
    bool is_grids_init_ = false;

    std::unique_ptr<grid_ns::Grid<char>> voxel_grids_;

    void SetMapOrigin(const Point3D& ori_robot_pos);
    void SetRayCloud(const Eigen::Vector3i& point_sub);

};


#endif
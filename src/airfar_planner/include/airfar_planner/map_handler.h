#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float ceil_length;
    float ceil_height;
    float grid_max_length;
    float grid_max_height;
    int neighbor_layers;
};

class MapHandler {

public:
    MapHandler() = default;
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos, std::vector<int>& layer_idxs);

    /** Update global cloud grid with incoming clouds 
     * @param CloudIn incoming cloud ptr
    */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudIn);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);

    /** Extract Surrounding Free & Obs clouds 
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut, const int& layer_id=-1);
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);

    /**
     * Get neihbor ceils center positions
     * @param neighbor_centers[out] neighbor centers stack
    */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor ceils center positions
     * @param occupancy_centers[out] occupanied ceils center stack
    */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param
    */ 
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /**
     * @brief Reset Current Grip Map Clouds
     */
    void ResetGripMapCloud();


    /* Get global map size */
    Eigen::Vector3i GetMapSize() const { return world_obs_cloud_grid_->GetSize();};

    inline int GetLayerId(const Point3D& point) {
        if (!is_init_) return -1;
        return world_obs_cloud_grid_->Pos2Sub(point.x, point.y, point.z).z();
    }


private:

    MapHandlerParams map_params_;
    int row_num_, col_num_, level_num_, neighbor_Lnum_, neighbor_Hnum_;
    Point3D map_origin_;
    Point3D neghbor_ceils_origin_;
    Point3D odom_pos_;
    bool is_init_ = false;

    // Surrounding neighbor cloud grid indices stack
    std::vector<int> neighbor_indices_;
    std::vector<int> global_visited_induces_;
    std::vector<int> util_obs_modified_list_;
    std::vector<int> util_free_modified_list_;
    std::vector<int> util_remove_check_list_;
    
    std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_obs_cloud_grid_;
    std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_free_cloud_grid_;
 
};

#endif
#ifndef NODE_STRUCT_H
#define NODE_STRUCT_H

#include "point_struct.h"

enum NodeType {
    NOT_DEFINED = 0,
    GROUND      = 1,
    AIR         = 2
};

enum NodeFreeDirect {
  UNKNOW  =  0,
  CONVEX  =  1,
  CONCAVE =  2,
  PILLAR  =  3,
  INSERT = 4
  // TOP_LAYER = 1
};

typedef std::pair<Point3D, Point3D> PointPair;
typedef std::pair<PointPair, PointPair> NavPair;

namespace LiDARMODEL {
    /* array resolution: 1 degree */
    static const int kHorizontalFOV = 360;  
    static const int kVerticalFOV = 31; 
    static const float kAngleResX = 0.2;
    static const float kAngleResY = 2.0;    
} // NODEPARAMS

struct Polygon
{
  Polygon() = default;
  int layer_id;
  std::size_t N;
  std::vector<Point3D> vertices;
  bool is_connect;
  bool is_visiable;
  bool is_robot_inside;
  bool is_pillar;
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;

struct CTNode
{
    CTNode() = default;
    int layer_id;
    Point3D position;
    bool is_global_match;
    bool is_wall_corner;
    bool is_wall_insert;
    std::size_t nav_node_id;
    NodeFreeDirect free_direct;

    PointPair surf_dirs;
    PolygonPtr poly_ptr;
    std::shared_ptr<CTNode> front;
    std::shared_ptr<CTNode> back;
    std::shared_ptr<CTNode> up;
    std::shared_ptr<CTNode> down;

    std::vector<std::shared_ptr<CTNode>> connect_nodes;
};

typedef std::shared_ptr<CTNode> CTNodePtr;
typedef std::vector<CTNodePtr> CTNodeStack;
typedef std::pair<CTNodePtr, CTNodePtr> CTNodePair;

struct NavNode
{
    NavNode() = default;
    int layer_id;
    std::size_t id;
    Point3D position;
    // z axis adjust
    float near_odom_dist;
    PointPair surf_dirs;
    std::deque<Point3D> pos_filter_vec;
    std::deque<PointPair> surf_dirs_vec;
    CTNodePtr ctnode;
    bool is_contour_match;
    bool is_top_layer = false;
    bool is_inserted = false;
    bool is_odom;
    bool is_goal;
    bool is_intermediate;
    bool is_near_nodes;
    bool is_wide_near;
    bool is_merged;
    bool is_frontier;
    bool is_finalized;
    bool is_navpoint;
    bool is_wall_corner;
    bool is_wall_insert;
    int clear_dumper_count;
    int finalize_counter;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;
    std::vector<std::shared_ptr<NavNode>> contour_connects;
    std::shared_ptr<NavNode> up_node = NULL;
    std::shared_ptr<NavNode> down_node =NULL;
    std::unordered_map<int, std::deque<int>> contour_votes;
    std::unordered_map<int, std::deque<int>> edge_votes;
    std::vector<std::shared_ptr<NavNode>> potential_contours;
    std::vector<std::shared_ptr<NavNode>> potential_edges;
    std::vector<std::shared_ptr<NavNode>> trajectory_connects;
    std::unordered_map<int, std::deque<int>> trajectory_votes; 
    NodeType node_type; 
    NodeFreeDirect free_direct;
    // planner members
    bool is_block_to_goal;
    bool is_traversable;
    bool is_free_traversable;
    bool is_reach_goal;
    float gscore, fgscore;
    std::shared_ptr<NavNode> parent;
    std::shared_ptr<NavNode> free_parent;
    std::pair<std::shared_ptr<NavNode>, std::shared_ptr<NavNode>> insert_node_parents;
    
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::pair<NavNodePtr, NavPair> MapPair;
typedef std::pair<NavNodePtr, NavNodePtr> NavNodePair;

struct nodeptr_equal
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->id == n2->id;
  }
};

struct nodeptr_hash
{
  std::size_t operator() (const NavNodePtr& n_ptr) const
  {
    return std::hash<int>()(n_ptr->id);
  }
};

struct nodeptr_gcomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->gscore > n2->gscore;
  }
};

struct nodeptr_fgcomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->fgscore > n2->fgscore;
  }
};

struct nodeptr_icomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->position.intensity < n2->position.intensity;
  }
};

#endif

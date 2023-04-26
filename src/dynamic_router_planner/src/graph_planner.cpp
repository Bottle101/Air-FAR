/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "dynamic_planner/graph_planner.h"

/***************************************************************************************/

const char INIT_BIT  = char(0); // 0000
const char OBS_BIT   = char(1); // 0001
const char FREE_BIT  = char(2); // 0010


void GraphPlanner::Init(const ros::NodeHandle& nh, const GraphPlannerParams& params) {
    nh_ = nh;
    gp_params_ = params;
    is_goal_init_ = false;
    current_graph_.clear();
    // attemptable planning listener
    attemptable_sub_ = nh_.subscribe("/planning_attemptable", 5, &GraphPlanner::AttemptStatusCallBack, this);
    planning_time_pub_ = nh_.advertise<std_msgs::Float32>("/far_planning_time", 5);
    traverse_time_pub_ = nh_.advertise<std_msgs::Float32>("/far_traverse_time", 5);
    reach_goal_pub_    = nh_.advertise<std_msgs::Bool>("/far_reach_goal_status", 5);
}

void GraphPlanner::UpdateGraphTraverability(const NodePtrStack& graph,
                                            const NavNodePtr& odom_node_ptr) 
{
    if (odom_node_ptr == NULL || graph.empty()) {
        ROS_ERROR("GP: Update global graph traversablity fails.");
        return;
    }
    odom_node_ptr_ = odom_node_ptr;
    current_graph_ = graph;
    this->InitNodesStates(current_graph_);
    // start expand the whole current_graph_
    odom_node_ptr_->gscore = 0.0;
    std::unordered_set<std::size_t> open_set;
    std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_gcomp> open_queue;
    std::unordered_set<NavNodePtr, nodeptr_hash, nodeptr_equal> close_set;
    // Expansion from odom node to all reachable navigation node
    open_queue.push(odom_node_ptr_);
    open_set.insert(odom_node_ptr_->id);
    
    std_msgs::Float32 planning_time;
    DPUtil::Timer.start_time("Path_Searching");
    reached_goal_msg_.data = false;

    while (!open_set.empty()) {
        const NavNodePtr current = open_queue.top();
        open_queue.pop();
        open_set.erase(current->id);
        close_set.insert(current);
        current->is_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if (close_set.count(neighbor)) continue;
            const float temp_gscore = current->gscore + this->EulerCost(current, neighbor);
            if (temp_gscore < neighbor->gscore) {
                neighbor->parent = current;
                neighbor->gscore = temp_gscore;
                if (!open_set.count(neighbor->id)) {
                    open_queue.push(neighbor);
                    open_set.insert(neighbor->id);
                }
            }
        }
    }
    planning_time.data = DPUtil::Timer.end_time("Path_Searching");
    planning_time_pub_.publish(planning_time);
    reach_goal_pub_.publish(reached_goal_msg_);

    std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_fgcomp> fopen_queue;
    std::unordered_set<std::size_t> fopen_set;
    close_set.clear();
    // Expansion from odom node to all covered navigation node
    odom_node_ptr_->fgscore = 0.0;
    fopen_queue.push(odom_node_ptr_);
    fopen_set.insert(odom_node_ptr_->id);
    while (!fopen_set.empty()) {
        const NavNodePtr current = fopen_queue.top();
        fopen_queue.pop();
        fopen_set.erase(current->id);
        close_set.insert(current);
        current->is_free_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if (neighbor->is_frontier || close_set.count(neighbor)) continue;
            const float e_dist = this->EulerCost(current, neighbor);
            if (neighbor == goal_node_ptr_ && e_dist > DPUtil::kSensorRange) continue; // To goal distance should less than sensor range
            const float temp_fgscore = current->fgscore + e_dist;
            if (temp_fgscore < neighbor->fgscore) {
                neighbor->free_parent = current;
                neighbor->fgscore = temp_fgscore;
                if (!fopen_set.count(neighbor->id)) {
                    fopen_queue.push(neighbor);
                    fopen_set.insert(neighbor->id);
                } 
            }
        }
    }
}

void GraphPlanner::UpdateGoalNavNodeConnects(const NavNodePtr& goal_node_ptr,
                                             const NodePtrStack& graphIn)
{
    if (goal_node_ptr_ == NULL || is_use_internav_goal_) return;
    this->ReEvaluateGoalStatus(goal_node_ptr_, graphIn);
    for (const auto& node_ptr : graphIn) {
        if (node_ptr == goal_node_ptr_) continue;
        if (this->IsValidConnectToGoal(node_ptr, goal_node_ptr_)) {
            const bool is_directly_connect = node_ptr->is_odom ? true : false;
            DynamicGraph::RecordPolygonEdge(node_ptr, goal_node_ptr_, gp_params_.votes_size, is_directly_connect);
        } else {
            DynamicGraph::DeletePolygonEdge(node_ptr, goal_node_ptr_, gp_params_.votes_size);
        }
        const auto it = goal_node_ptr_->edge_votes.find(node_ptr->id);
        if (it != goal_node_ptr_->edge_votes.end() && DPUtil::IsVoteTrue(it->second)) {
            DynamicGraph::AddEdge(node_ptr, goal_node_ptr_);
            node_ptr->is_block_to_goal = false;
            node_ptr->is_reach_goal = true;
        } else {
            DynamicGraph::EraseEdge(node_ptr, goal_node_ptr_);
            node_ptr->is_block_to_goal = true;
            node_ptr->is_reach_goal = false;
        }
    }
}

bool GraphPlanner::IsValidConnectToGoal(const NavNodePtr& node_ptr, const NavNodePtr& goal_node_ptr) {
    if (!node_ptr->is_block_to_goal || IsResetBlockStatus(node_ptr)) {
        if (DynamicGraph::IsConnectInVerticalConstrain(node_ptr, goal_node_ptr) && 
            ContourGraph::IsNavToGoalConnectFreePolygon(node_ptr, goal_node_ptr))
        {
            return true;
        }
    }
    return false;
}

bool GraphPlanner::NextGoalPlanning(PointStack& global_path,
                                    Point3D& _nav_goal,
                                    Point3D& _goal_p,
                                    bool& _is_fails,
                                    bool& _is_free_nav) 
{
    if (!is_goal_init_) return false;

    std_msgs::Float32 traverse_timer;
    traverse_timer.data = DPUtil::Timer.record_time("Overall_executing");
    traverse_time_pub_.publish(traverse_timer);

    if (odom_node_ptr_ == NULL || goal_node_ptr_ == NULL || current_graph_.empty()) {
        ROS_ERROR("GP: Graph or Goal is not initialized correctly.");
        return false;
    }
    _is_fails = false;
    global_path.clear();
    _goal_p = goal_node_ptr_->position;
    if (current_graph_.size() == 1) {
        const Point3D diff_p = (_goal_p - odom_node_ptr_->position).normalize();
        const Point3D next_goal = odom_node_ptr_->position + diff_p * 5.0;
        // update global path
        global_path.push_back(odom_node_ptr_->position);
        global_path.push_back(next_goal);
        _nav_goal = this->NextNavWaypointFromPath(global_path);
        _is_free_nav = is_free_nav_goal_;
        return true;       
    }
    if ((odom_node_ptr_->position - _goal_p). norm() < gp_params_.converge_dist || 
        (odom_node_ptr_->position - origin_goal_pos_).norm() < gp_params_.converge_dist)
    {
        ROS_INFO("GP: *********** Goal Reached! ***********");

        reached_goal_msg_.data = true;
        traverse_timer.data = DPUtil::Timer.end_time("Overall_executing");
        traverse_time_pub_.publish(traverse_timer);
        reach_goal_pub_.publish(reached_goal_msg_);

        global_path.push_back(odom_node_ptr_->position);
        if ((odom_node_ptr_->position - _goal_p). norm() > gp_params_.converge_dist) _goal_p = origin_goal_pos_;
        global_path.push_back(_goal_p);
        _nav_goal = _goal_p;
        _is_free_nav = is_free_nav_goal_;
        this->GoalReset();
        is_goal_init_ = false;
        return true;
    }
    // check free space navigation status
    if (gp_params_.is_autoswitch && command_is_free_nav_) {
        if (goal_node_ptr_->is_free_traversable || (is_free_nav_goal_ && is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred)) {
            is_free_nav_goal_ = true;
        } else {
            is_free_nav_goal_ = false;
        }
    }   
    _is_free_nav = is_free_nav_goal_;
    const NavNodePtr reach_nav_node = is_free_nav_goal_ ? goal_node_ptr_->free_parent : goal_node_ptr_->parent;
    if (is_global_path_init_ && last_waypoint_dist_ > gp_params_.adjust_radius && reach_nav_node != NULL && reach_nav_node != odom_node_ptr_) {
        // momentum planning if movement towards to current waypoints is less than threshold
        const float cur_waypoint_dist = (odom_node_ptr_->position - next_waypoint_).norm();
        if (cur_waypoint_dist > gp_params_.converge_dist && (abs((odom_node_ptr_->position - last_planning_odom_).norm() < gp_params_.momentum_dist) 
            && abs(cur_waypoint_dist - last_waypoint_dist_) < gp_params_.momentum_dist)) 
        {
            ROS_INFO("GP: inside momentum distance, follow the previous waypoint.");
            recorded_path_[0] = odom_node_ptr_->position;
            global_path = recorded_path_;
            _nav_goal = this->NextNavWaypointFromPath(global_path);
            return true;
        }
    }
    if (reach_nav_node == NULL) {
        // no reachable goal found
        if (is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred) {
            ROS_WARN("GP: no reachable nav node found, path momentum planning...");
            global_path = recorded_path_;
            _nav_goal = this->NextNavWaypointFromPath(global_path);
            if (!odom_node_ptr_->connect_nodes.empty()) {
                path_momentum_counter_ ++;
            }
            return true;
        } else {
            if (gp_params_.is_autoswitch && is_free_nav_goal_) {
                ROS_WARN("GP: free navigation fails, auto swiching to attemptable navigation...");
                if (is_global_path_init_) {
                    global_path = recorded_path_;
                    _nav_goal = this->NextNavWaypointFromPath(global_path);
                } else {
                    _nav_goal = _goal_p;
                }
                is_free_nav_goal_ = false;
                return true;
            }
            ROS_ERROR("****************** FAIL TO REACH GOAL ******************");
            this->GoalReset();
            is_goal_init_ = false, _is_fails = true;
            return false;
        }
    }
    if (this->ReconstructPath(goal_node_ptr_, is_free_nav_goal_, global_path)) {
        _nav_goal = this->NextNavWaypointFromPath(global_path);
        this->RecordPathInfo(global_path);
        return true;
    }
    this->GoalReset();
    is_goal_init_ = false, _is_fails = true;
    return false;
}

bool GraphPlanner::ReconstructPath(const NavNodePtr& goal_node_ptr,
                                   const bool& is_free_nav,
                                   PointStack& global_path)
{
    if (goal_node_ptr == NULL || (!is_free_nav && goal_node_ptr->parent == NULL) || (is_free_nav && goal_node_ptr->free_parent == NULL)) {
        ROS_ERROR("GP: reconstruct path error: goal node or its parent equals to NULL.");
        return false;
    }
    global_path.clear();
    NavNodePtr check_ptr = goal_node_ptr;
    global_path.push_back(check_ptr->position);
    if (is_free_nav) {
        while (true) {
            const NavNodePtr parent_ptr = check_ptr->free_parent;
            global_path.push_back(parent_ptr->position);
            if (parent_ptr->free_parent == NULL) break;
            check_ptr = parent_ptr;
        }
    } else {
        while (true) {
            const NavNodePtr parent_ptr = check_ptr->parent;
            global_path.push_back(parent_ptr->position);
            if (parent_ptr->parent == NULL) break;
            check_ptr = parent_ptr;
        } 
    }
    std::reverse(global_path.begin(), global_path.end()); 
    return true;
}

Point3D GraphPlanner::NextNavWaypointFromPath(const PointStack& global_path) {
    if (global_path.size() < 2) {
        ROS_ERROR("GP: global path size less than 2.");
        return goal_node_ptr_->position;
    }
    Point3D nav_waypoint;
    const std::size_t path_size = global_path.size();
    std::size_t nav_idx = 1;
    nav_waypoint = global_path[nav_idx];
    float dist = (nav_waypoint - odom_node_ptr_->position).norm();
    while (dist < gp_params_.converge_dist) {
        nav_idx ++;
        if (nav_idx < path_size) {
            nav_waypoint = global_path[nav_idx];
            dist = (nav_waypoint - odom_node_ptr_->position).norm();
        } else break;
    }
    return nav_waypoint;
}

void GraphPlanner::UpdateGoal(const Point3D& goal, const int& layer_id, const bool& is_free_nav) {
    this->GoalReset();
    is_use_internav_goal_ = false;
    float min_dist = DPUtil::kNearDist;
    for (const auto& node_ptr : current_graph_) {
        node_ptr->is_block_to_goal = false;
        if (DPUtil::IsNavpoint && node_ptr->is_navpoint) { // check if goal is near internav node
            const float cur_dist = (node_ptr->position - goal).norm();
            if (cur_dist < min_dist) {
                is_use_internav_goal_ = true;
                goal_node_ptr_ = node_ptr;
                min_dist = cur_dist;
            }
        }
    }
    if (!is_use_internav_goal_) {
        DynamicGraph::CreateNavNodeFromPoint(goal, layer_id, goal_node_ptr_, false, false, true);
        DynamicGraph::AddNodeToGraph(goal_node_ptr_);
    }
    ROS_INFO("GP: *********** new goal updated ***********");
    is_goal_init_ = true;
    is_global_path_init_ = false;
    origin_goal_pos_ = goal_node_ptr_->position;
    origin_layer_id_ = layer_id;
    is_origin_free_ = is_use_internav_goal_ ? true : false;
    command_is_free_nav_ = is_free_nav;
    is_free_nav_goal_ = command_is_free_nav_;
    next_waypoint_ = Point3D(0,0,0);
    last_waypoint_dist_ = 0.0;
    last_planning_odom_ = Point3D(0,0,0);
    recorded_path_.clear();
    path_momentum_counter_ = 0;
    reached_goal_msg_.data = false;
    DPUtil::Timer.start_time("Overall_executing", true);
}

void GraphPlanner::AttemptStatusCallBack(const std_msgs::Bool& msg) {
    if (command_is_free_nav_) { // current goal is not attemptable
        if (msg.data) {
            ROS_WARN("GP: switch to attemptable planning mode.");
            command_is_free_nav_ = false;
        }
    } else { // current attemptable planning
        if (!msg.data) {
            ROS_WARN("GP: planning without attempting.");
            command_is_free_nav_ = true; 
        }
    }
}

void GraphPlanner::ReEvaluateGoalStatus(const NavNodePtr& goal_ptr, const NodePtrStack& graphIn) {
    if (is_use_internav_goal_) return; // return if using an exsiting internav node as goal
    bool is_position_change =  false;
    // reproject to nearby free space
    int min_layer_dist = DPUtil::layerIdx2Height_.size();
    int min_layer_id = goal_ptr->layer_id;
    for (const auto& node_ptr : graphIn) {
        if (node_ptr == goal_node_ptr_) continue;
        const int cur_layer_diff = origin_layer_id_ - node_ptr->layer_id;
        int adjust_layer_id = node_ptr->layer_id; 
        if (cur_layer_diff > 0) {
            adjust_layer_id = node_ptr->layer_id + std::min(abs(cur_layer_diff), DPUtil::kNeighborLayers);
        } else if (cur_layer_diff < 0) {
            adjust_layer_id = node_ptr->layer_id - std::min(abs(cur_layer_diff), DPUtil::kNeighborLayers);
        }
        const int cur_layer_dist = abs(adjust_layer_id - origin_layer_id_);
        if (cur_layer_dist < min_layer_dist) {
            min_layer_dist = cur_layer_dist;
            min_layer_id = adjust_layer_id;
        }
    }
    if (min_layer_id != goal_ptr->layer_id) {
        goal_ptr->layer_id = min_layer_id;
        goal_ptr->position.z = DPUtil::layerIdx2Height_[min_layer_id];
        ROS_WARN("GP: goal layer adjusted to %d, with absolute height: %f", min_layer_id, goal_ptr->position.z);
        is_position_change = true;
    }
    if (is_position_change) {
        for (const auto& node_ptr : graphIn) {
            node_ptr->is_block_to_goal = false;
        }
        DynamicGraph::ClearNodeConnectInGraph(goal_node_ptr_);
    }
}

/******************************* UNUSE CODE ******************************************/


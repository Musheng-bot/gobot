#pragma once

#include "base.h"
#include "map/geometric_topo_map.h"
#include "map/occupancy_2d.h"
#include "map/semantic_topo_map.h"
#include "planner/astar.h"
#include "planner/minco.h"

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace rush {

/// RushExtended is a class for calling various lower-level tools, like global planning and local
/// planning.
class RushExtended {
public:
    enum class RobotType {
        LEG,    // 腿
        TURTLE, // 龟
    };

    enum class NavMode {
        NORMAL, // 普通巡航
        CHASE,  // 追击
    };

    enum class NavStatus {
        IDLE,      // default status, no mission right now
        PLANNING,  // after receiving nav_command
        EXECUTING, // path_plan finished, only control plan executing
        REACHED,   // reached the goal
        ERROR,     // in error status due to unknown reason
    };

    using RushExtendedParam = struct Param {
        RushExtended::RobotType mode;    // 轮腿/龟
        std::string map_name;            // 使用的地图
        std::string decision_map_folder; // decision配置
        float path_plan_freq;            // 全局路径规划频率
        float control_freq;              // 控制器规划控制量频率
    };

    RushExtended();
    ~RushExtended();

    void set_goal(const Pose &pose);
    void set_robot_pose(const Pose &pose);
    void set_nav_mode(NavMode mode);

    RobotType get_robot_type() const;
    NavMode get_nav_mode() const;

    void tick_map_update();
    void tick_plan(const TopographyPath &topography_path);
    void tick_control();
    void tick_replan_check();

    static bool check_param(const RushExtendedParam &param);

private:
    RushExtendedParam param_;
    Pose robot_pose_;

    struct {
        std::uint32_t task_id;
        std::uint32_t plan_id;
        Pose goal_pose;
        std::optional<Path2f> path;
    } plan_cmd_pack_; // 这是一次路径规划内的数据，需要始终保证同步

    struct {
        ControlMode mode;
        // if VX_VY, it represents (vx, vy);
        // else if V_OMEGA, it represents (v, omega)
        std::pair<float, float> control_value;
    } control_cmd_pack_;

    // maps
    Occupancy2d global_map_;
    Occupancy2d local_map_;
    SemanticTopoMap semantic_topo_map_;
    GeometricTopoMap geometric_topo_map_;

    // planners
    AStar astar_planner_;
    Minco minco_optimizer_;

    // middle states
    std::atomic<NavMode> nav_mode_;
    std::atomic<NavStatus> nav_status_;
    Path2f global_path_;
};

} // namespace rush
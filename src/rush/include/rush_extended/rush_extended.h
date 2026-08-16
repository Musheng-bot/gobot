#pragma once

#include "base.h"
#include "planner/planner_base.h"

#include <atomic>
#include <string>
#include <thread>

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
        IDLE,   // 无任务
    };

    typedef struct Param {
        RushExtended::RobotType mode;    // 轮腿/龟
        std::string map_name;            // 使用的地图
        std::string decision_map_folder; // decision配置
        float path_plan_freq;            // 全局路径规划频率
        float control_freq;              // 控制器规划控制量频率
    } RushExtendedParam;

    RushExtended();
    ~RushExtended();

    void set_goal(const Pose &pose);

    void set_robot_pose(const Pose &pose);

    RobotType robot_type() const;

    void set_nav_mode(NavMode mode);

    static bool check_param(const RushExtendedParam &param);

    bool is_running() const;

private:
    RushExtendedParam param_;
    Pose robot_pose_;
    Pose goal_pose_;

    PathPlanner::Ptr path_planner_;

    std::thread path_planner_thread_;

    std::atomic<NavMode> nav_mode_;

    void plan_path_thread_func();
};

} // namespace rush
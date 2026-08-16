#pragma once

#include "base.h"
#include "map/map_base.h"

#include <memory>
#include <optional>
#include <string>

namespace rush {

class PathPlanner {
public:
    using Ptr = std::shared_ptr<PathPlanner>;

    struct PlanResult {
        std::optional<Path2f> path;
        std::string detail;

        bool is_success() const {
            return path.has_value();
        }
    };

    virtual ~PathPlanner();

    virtual PlanResult plan_path(const Pose &robot_pose, const Pose &goal_pose,
                                 bool use_fixed_goal_heading) = 0;
};

class Optimizer {
public:
    using Ptr = std::shared_ptr<Optimizer>;

    struct OptimizeResult {
        std::optional<Path2f> path;
        std::string detail;

        bool is_success() const {
            return path.has_value();
        }
    };

    Optimizer(Map2d::SharedPtr map);
    virtual ~Optimizer();

    virtual OptimizeResult optimize(const Pose &robot_pose, const Pose &goal_pose,
                                    const Path2f &original_path, bool use_fixed_goal_heading);
};

class Controller {
public:
    using Ptr = std::shared_ptr<Controller>;
};

} // namespace rush
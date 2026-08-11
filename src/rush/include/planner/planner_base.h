#pragma once

#include "map/map_base.h"

namespace gobot {

class Planner {
public:
    Planner(Map2d::SharedPtr map);
    virtual ~Planner();

    Map2d::SharedPtr get_map() const;

private:
    Map2d::SharedPtr map_;
};

class PathPlanner {

};

class Optimizer : public Planner {
public:
    Optimizer(Map2d::SharedPtr map);
    ~Optimizer() override;
};

struct ControlValue {
    float speed;
    float omega;
};

class Controller : public Planner {

};

} // namespace gobot
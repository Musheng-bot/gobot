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

class GlobalPlanner : public Planner {
public:
    GlobalPlanner(Map2d::SharedPtr map);
    ~GlobalPlanner() override;
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

class LocalPlanner : public Planner {
public:
    LocalPlanner(Map2d::SharedPtr map);
    ~LocalPlanner() override;
};

} // namespace gobot
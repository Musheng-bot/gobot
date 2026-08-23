#pragma once

#include "base.h"
#include "map/map_config.h"

namespace rush {

class GeometricTopoMap {
public:
    GeometricTopoMap();
    ~GeometricTopoMap();

    Path2f translate_to_real_path(const TopographyPath &topography_path);
};

} // namespace rush
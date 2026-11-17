#ifndef TERRAIN_NAV_MAPPING__TRAVERSABILITY_HPP_
#define TERRAIN_NAV_MAPPING__TRAVERSABILITY_HPP_

#include <grid_map_core/GridMap.hpp>

namespace terrain_nav_mapping
{

struct TraversabilityParams
{
  double slope_crit_rad{0.349};
  double step_crit{0.15};
  double roughness_crit{0.08};
  int plane_window{5};
  int step_window{3};
  int min_valid_cells{6};
};

namespace layers
{
constexpr const char * kElevation = "elevation";
constexpr const char * kSlope = "slope";
constexpr const char * kRoughness = "roughness";
constexpr const char * kStep = "step";
constexpr const char * kTraversability = "traversability";
}

void computeTraversability(grid_map::GridMap & map, const TraversabilityParams & params);

}

#endif

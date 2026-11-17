#include <gtest/gtest.h>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <cmath>

#include "terrain_nav_mapping/traversability.hpp"

using terrain_nav_mapping::TraversabilityParams;
using terrain_nav_mapping::computeTraversability;
namespace layers = terrain_nav_mapping::layers;

namespace
{

grid_map::GridMap makeMap()
{
  grid_map::GridMap map({layers::kElevation});
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.2);
  map[layers::kElevation].setConstant(NAN);
  return map;
}

float atCentre(const grid_map::GridMap & map, const char * layer)
{
  return map.atPosition(layer, grid_map::Position(0.0, 0.0));
}

}

TEST(Traversability, FlatGroundIsFree)
{
  auto map = makeMap();
  map[layers::kElevation].setConstant(0.0);
  computeTraversability(map, TraversabilityParams{});
  EXPECT_NEAR(atCentre(map, layers::kSlope), 0.0, 1e-4);
  EXPECT_NEAR(atCentre(map, layers::kTraversability), 0.0, 1e-4);
}

TEST(Traversability, PlaneSlopeIsMeasured)
{
  auto map = makeMap();
  const double angle = 10.0 * M_PI / 180.0;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position p;
    map.getPosition(*it, p);
    map.at(layers::kElevation, *it) = static_cast<float>(std::tan(angle) * p.x());
  }
  TraversabilityParams params;
  computeTraversability(map, params);
  EXPECT_NEAR(atCentre(map, layers::kSlope), angle, 1e-3);
  EXPECT_NEAR(atCentre(map, layers::kRoughness), 0.0, 1e-4);
  EXPECT_NEAR(atCentre(map, layers::kTraversability), 0.5, 0.02);
}

TEST(Traversability, StepIsLethal)
{
  auto map = makeMap();
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position p;
    map.getPosition(*it, p);
    map.at(layers::kElevation, *it) = p.x() > 0.1 ? 0.3f : 0.0f;
  }
  computeTraversability(map, TraversabilityParams{});
  const float cost = map.atPosition(layers::kTraversability, grid_map::Position(0.1, 0.0));
  EXPECT_FLOAT_EQ(cost, 1.0f);
  EXPECT_NEAR(map.atPosition(layers::kStep, grid_map::Position(0.1, 0.0)), 0.3, 1e-4);
}

TEST(Traversability, UnknownCellsStayUnknown)
{
  auto map = makeMap();
  computeTraversability(map, TraversabilityParams{});
  EXPECT_TRUE(std::isnan(atCentre(map, layers::kTraversability)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

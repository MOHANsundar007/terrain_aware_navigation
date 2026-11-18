#include <gtest/gtest.h>

#include <cmath>

#include <nav2_costmap_2d/cost_values.hpp>

#include "terrain_costmap_plugin/traversability_layer.hpp"

using terrain_costmap_plugin::traversabilityToCost;

constexpr double kFree = 0.3;
constexpr double kLethal = 0.95;
constexpr unsigned char kMax = 252;

TEST(CostMapping, UnknownStaysUnknown)
{
  EXPECT_EQ(traversabilityToCost(NAN, kFree, kLethal, kMax), nav2_costmap_2d::NO_INFORMATION);
}

TEST(CostMapping, BelowFreeThresholdIsFree)
{
  EXPECT_EQ(traversabilityToCost(0.0f, kFree, kLethal, kMax), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(traversabilityToCost(0.29f, kFree, kLethal, kMax), nav2_costmap_2d::FREE_SPACE);
}

TEST(CostMapping, LethalAtThreshold)
{
  EXPECT_EQ(traversabilityToCost(0.95f, kFree, kLethal, kMax), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(traversabilityToCost(1.0f, kFree, kLethal, kMax), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostMapping, LinearAndMonotonicInBetween)
{
  const auto lo = traversabilityToCost(0.3f, kFree, kLethal, kMax);
  const auto mid = traversabilityToCost(0.625f, kFree, kLethal, kMax);
  const auto hi = traversabilityToCost(0.94f, kFree, kLethal, kMax);
  EXPECT_EQ(lo, 1);
  EXPECT_NEAR(mid, 126, 1);
  EXPECT_LE(hi, kMax);
  EXPECT_LT(lo, mid);
  EXPECT_LT(mid, hi);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

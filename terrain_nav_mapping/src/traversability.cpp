#include "terrain_nav_mapping/traversability.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace terrain_nav_mapping
{

void computeTraversability(grid_map::GridMap & map, const TraversabilityParams & p)
{
  for (const char * layer : {layers::kSlope, layers::kRoughness, layers::kStep,
      layers::kTraversability})
  {
    if (!map.exists(layer)) {
      map.add(layer);
    }
  }
  map.convertToDefaultStartIndex();

  const grid_map::Matrix & elev = map[layers::kElevation];
  grid_map::Matrix & slope = map[layers::kSlope];
  grid_map::Matrix & rough = map[layers::kRoughness];
  grid_map::Matrix & step = map[layers::kStep];
  grid_map::Matrix & trav = map[layers::kTraversability];

  const float nan = std::numeric_limits<float>::quiet_NaN();
  slope.setConstant(nan);
  rough.setConstant(nan);
  step.setConstant(nan);
  trav.setConstant(nan);

  const int rows = static_cast<int>(elev.rows());
  const int cols = static_cast<int>(elev.cols());
  const double res = map.getResolution();
  const int hp = p.plane_window / 2;
  const int hs = p.step_window / 2;
  const int max_pts = p.plane_window * p.plane_window;

  Eigen::MatrixXd A(max_pts, 3);
  Eigen::VectorXd b(max_pts);

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (!std::isfinite(elev(r, c))) {
        continue;
      }

      float zmin = elev(r, c), zmax = elev(r, c);
      for (int dr = -hs; dr <= hs; ++dr) {
        for (int dc = -hs; dc <= hs; ++dc) {
          const int rr = r + dr, cc = c + dc;
          if (rr < 0 || cc < 0 || rr >= rows || cc >= cols) {continue;}
          const float z = elev(rr, cc);
          if (!std::isfinite(z)) {continue;}
          zmin = std::min(zmin, z);
          zmax = std::max(zmax, z);
        }
      }
      step(r, c) = zmax - zmin;

      int n = 0;
      for (int dr = -hp; dr <= hp; ++dr) {
        for (int dc = -hp; dc <= hp; ++dc) {
          const int rr = r + dr, cc = c + dc;
          if (rr < 0 || cc < 0 || rr >= rows || cc >= cols) {continue;}
          const float z = elev(rr, cc);
          if (!std::isfinite(z)) {continue;}
          A(n, 0) = dr * res;
          A(n, 1) = dc * res;
          A(n, 2) = 1.0;
          b(n) = z;
          ++n;
        }
      }

      double cost_slope = 0.0, cost_rough = 0.0;
      if (n >= p.min_valid_cells) {
        const auto An = A.topRows(n);
        const auto bn = b.head(n);
        const Eigen::Vector3d coef = An.colPivHouseholderQr().solve(bn);
        const double grad = std::hypot(coef(0), coef(1));
        const double s = std::atan(grad);
        const double rms = std::sqrt((An * coef - bn).squaredNorm() / n);
        slope(r, c) = static_cast<float>(s);
        rough(r, c) = static_cast<float>(rms);
        cost_slope = s / p.slope_crit_rad;
        cost_rough = rms / p.roughness_crit;
      }
      const double cost_step = step(r, c) / p.step_crit;

      trav(r, c) = static_cast<float>(
        std::clamp(std::max({cost_slope, cost_rough, cost_step}), 0.0, 1.0));
    }
  }
}

}

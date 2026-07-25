#ifndef PATH_PLANNING_BSPLINE_HPP_
#define PATH_PLANNING_BSPLINE_HPP_

#include <vector>

#include "geometry_types.hpp"

namespace path_planning {

// Fits a clamped, uniform cubic B-spline through control_points (a parametric
// curve x(t)/y(t), so it has no trouble with a vertical-in-local-frame track
// segment the way a y=f(x) polynomial would) and resamples it into points
// spaced spacing_m apart along arc length. The spline passes exactly through
// the first and last control point but only approximates the interior ones
// (a "rubber band" through them), which smooths out cone-position noise
// instead of chasing it. Falls back to returning control_points unchanged if
// there are fewer than 4 (a cubic needs at least 4 control points).
std::vector<Point2D> smooth_and_resample(const std::vector<Point2D>& control_points,
                                         double spacing_m);

}  // namespace path_planning

#endif  // PATH_PLANNING_BSPLINE_HPP_

// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#ifndef PATH_PLANNING_BSPLINE_HPP_
#define PATH_PLANNING_BSPLINE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "geometry_types.hpp"

namespace path_planning {
namespace bspline_detail {

/**
 * Validate coordinates and measure the control polygon.
 *
 * @param points The vector of spline control points.
 * @return The sum of distances between consecutive control points.
 */
inline double control_polygon_length(const std::vector<Point2D>& points) {
  double polygon_length = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y)) {
      throw std::invalid_argument("Spline coordinates must be finite");
    }
    if (i > 0) {
      polygon_length += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
  }
  return polygon_length;
}

/**
 * Build a uniform knot vector with repeated endpoint knots.
 *
 * @param point_count The number of control points.
 * @param degree The spline degree.
 * @return A knot vector that makes the curve preserve its endpoints.
 */
inline std::vector<double> make_clamped_knots(std::size_t point_count, int degree) {
  const std::size_t spans = point_count - degree;
  std::vector<double> knots(point_count + degree + 1, 0.0);
  for (std::size_t i = 1; i < spans; ++i) {
    knots[degree + i] = static_cast<double>(i) / spans;
  }
  std::fill(knots.begin() + point_count, knots.end(), 1.0);

  return knots;
}

/**
 * Evaluate one spline point using de Boor's algorithm.
 *
 * @param t The curve parameter between zero and one.
 * @param points The vector of spline control points.
 * @param knots The clamped knot vector.
 * @param degree The spline degree.
 * @param span The current knot span, updated as the parameter advances.
 * @return The point on the spline at the given parameter.
 */
inline Point2D evaluate_spline(double t, const std::vector<Point2D>& points,
                               const std::vector<double>& knots, int degree, std::size_t& span) {
  if (t >= 1.0) return points.back();
  while (span + 1 < points.size() && t >= knots[span + 1]) ++span;
  std::array<Point2D, 4> work{};
  for (int j = 0; j <= degree; ++j) work[j] = points[span - degree + j];
  // Repeatedly blend neighboring controls until one curve point remains.
  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const std::size_t lo = span - degree + j;
      const double denominator = knots[span + j + 1 - r] - knots[lo];
      const double weight = denominator > 0.0 ? (t - knots[lo]) / denominator : 0.0;
      work[j].x = (1.0 - weight) * work[j - 1].x + weight * work[j].x;
      work[j].y = (1.0 - weight) * work[j - 1].y + weight * work[j].y;
    }
  }
  return work[degree];
}

/**
 * Sample the spline and emit waypoints at approximately equal travel distances.
 *
 * @param points The vector of spline control points.
 * @param knots The clamped knot vector.
 * @param degree The spline degree.
 * @param sample_count The number of curve segments to sample.
 * @param spacing_m The desired waypoint spacing in metres.
 * @param output_capacity The initial capacity to reserve for the output vector.
 * @return The resampled waypoints, including both endpoints.
 */
inline std::vector<Point2D> resample_spline(const std::vector<Point2D>& points,
                                            const std::vector<double>& knots, int degree,
                                            std::size_t sample_count, double spacing_m,
                                            std::size_t output_capacity) {
  std::size_t span = degree;

  std::vector<Point2D> waypoints;
  waypoints.reserve(output_capacity);
  waypoints.push_back(points.front());
  Point2D previous = points.front();
  double distance_traveled = 0.0;
  std::size_t next_waypoint = 1;
  for (std::size_t i = 1; i <= sample_count; ++i) {
    const Point2D current =
        evaluate_spline(static_cast<double>(i) / sample_count, points, knots, degree, span);
    const double segment_length = std::hypot(current.x - previous.x, current.y - previous.y);
    if (segment_length > 0.0) {
      while (next_waypoint * spacing_m <= distance_traveled + segment_length) {
        const double fraction =
            std::clamp((next_waypoint * spacing_m - distance_traveled) / segment_length, 0.0, 1.0);
        Point2D waypoint;
        waypoint.x = (1.0 - fraction) * previous.x + fraction * current.x;
        waypoint.y = (1.0 - fraction) * previous.y + fraction * current.y;
        waypoints.push_back(waypoint);
        ++next_waypoint;
      }
    }
    distance_traveled += segment_length;
    previous = current;
  }
  // Exact endpoint once, including when total distance_traveled is a spacing multiple.
  if (waypoints.back().x != points.back().x || waypoints.back().y != points.back().y) {
    waypoints.push_back(points.back());
  } else {
    waypoints.back() = points.back();
  }
  return waypoints;
}

}  // namespace bspline_detail

/**
 * Smooth control points with a clamped B-spline and preserve both endpoints.
 * Interior points guide the curve; it need not pass through them.
 *
 * @param points The vector of spline control points.
 * @param spacing_m The desired waypoint spacing in metres.
 * @return Smoothed waypoints with approximate spacing and a possibly shorter final segment.
 */
inline std::vector<Point2D> smooth_and_resample(const std::vector<Point2D>& points,
                                                double spacing_m) {
  if (!std::isfinite(spacing_m) || spacing_m <= 0.0) {
    throw std::invalid_argument("Spline spacing must be finite and positive");
  }
  const double length = bspline_detail::control_polygon_length(points);
  if (points.empty()) return {};
  if (length == 0.0) return {points.front()};

  // Bound memory and work for very long paths or tiny spacing.
  constexpr std::size_t max_samples = 1000000;
  const double intervals = std::ceil(length / spacing_m);
  if (!std::isfinite(intervals) || intervals > max_samples / 4 ||
      points.size() > max_samples / 20) {
    throw std::length_error("Spline sampling exceeds the one-million-sample limit");
  }
  const int degree = static_cast<int>(std::min<std::size_t>(3, points.size() - 1));
  const auto knots = bspline_detail::make_clamped_knots(points.size(), degree);
  const auto interval_count = static_cast<std::size_t>(intervals);
  const std::size_t samples = std::max(20 * (points.size() - degree), 4 * interval_count);
  return bspline_detail::resample_spline(points, knots, degree, samples, spacing_m,
                                         interval_count + 2);
}

}  // namespace path_planning
#endif  // PATH_PLANNING_BSPLINE_HPP_

#include "bspline.hpp"

#include <algorithm>
#include <cmath>

namespace path_planning {
namespace {

constexpr int kDegree = 3;

std::vector<double> clamped_knot_vector(int n_control_points, int degree) {
  const int n = n_control_points - 1;
  const int m = n + degree + 1;
  std::vector<double> knots(m + 1);

  for (int i = 0; i <= degree; ++i) {
    knots[i] = 0.0;
    knots[m - i] = 1.0;
  }
  const int n_interior = n - degree;
  for (int j = 1; j <= n_interior; ++j) {
    knots[degree + j] = static_cast<double>(j) / static_cast<double>(n_interior + 1);
  }
  return knots;
}

// De Boor's algorithm: evaluates the clamped B-spline curve at parameter t.
Point2D de_boor(double t, int degree, const std::vector<double>& knots,
               const std::vector<Point2D>& control_points) {
  const int n = static_cast<int>(control_points.size()) - 1;

  int k = degree;
  for (; k < n; ++k) {
    if (t >= knots[k] && t < knots[k + 1]) {
      break;
    }
  }

  std::vector<Point2D> d(degree + 1);
  for (int j = 0; j <= degree; ++j) {
    d[j] = control_points[j + k - degree];
  }

  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const int knot_idx_hi = j + 1 + k - r;
      const int knot_idx_lo = j + k - degree;
      const double denom = knots[knot_idx_hi] - knots[knot_idx_lo];
      const double alpha = denom > 1e-12 ? (t - knots[knot_idx_lo]) / denom : 0.0;
      d[j].x = (1.0 - alpha) * d[j - 1].x + alpha * d[j].x;
      d[j].y = (1.0 - alpha) * d[j - 1].y + alpha * d[j].y;
    }
  }
  return d[degree];
}

}  // namespace

std::vector<Point2D> smooth_and_resample(const std::vector<Point2D>& control_points,
                                         double spacing_m) {
  if (static_cast<int>(control_points.size()) < kDegree + 1 || spacing_m <= 0.0) {
    return control_points;
  }

  const auto knots = clamped_knot_vector(static_cast<int>(control_points.size()), kDegree);

  const int dense_n = static_cast<int>(control_points.size()) * 20;
  std::vector<Point2D> dense(dense_n + 1);
  for (int i = 0; i <= dense_n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(dense_n);
    dense[i] = de_boor(std::min(t, 1.0 - 1e-9), kDegree, knots, control_points);
  }
  dense.back() = control_points.back();  // clamped spline ends exactly on the last point

  std::vector<double> cum_len(dense.size(), 0.0);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    const double dx = dense[i].x - dense[i - 1].x;
    const double dy = dense[i].y - dense[i - 1].y;
    cum_len[i] = cum_len[i - 1] + std::sqrt(dx * dx + dy * dy);
  }
  const double total_len = cum_len.back();
  if (total_len < 1e-6) {
    return dense;
  }

  std::vector<Point2D> out;
  const int n_out = static_cast<int>(total_len / spacing_m) + 1;
  std::size_t seg = 0;
  for (int i = 0; i <= n_out; ++i) {
    const double target = std::min(total_len, i * spacing_m);
    while (seg + 2 < cum_len.size() && cum_len[seg + 1] < target) {
      ++seg;
    }
    const double seg_len = cum_len[seg + 1] - cum_len[seg];
    const double frac = seg_len > 1e-9 ? (target - cum_len[seg]) / seg_len : 0.0;
    Point2D p;
    p.x = dense[seg].x + frac * (dense[seg + 1].x - dense[seg].x);
    p.y = dense[seg].y + frac * (dense[seg + 1].y - dense[seg].y);
    out.push_back(p);
  }
  return out;
}

}  // namespace path_planning

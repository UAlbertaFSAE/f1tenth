// MIT License
//
// Copyright (c) 2026 UAlbertaFSAE
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "bspline.hpp"
#include "filtering.hpp"

using namespace path_planning;

TEST(Planning, StraightTrackProducesOrderedSpacedWaypoints) {
  std::vector<Point2D> cones;
  for (int x = 1; x <= 9; x += 2) {
    cones.push_back({double(x), 1.0, true});
    cones.push_back({double(x), -1.0, false});
  }
  const auto gates = build_filtered_delaunay(cones, 2.1, {});
  std::vector<Point2D> ordered;
  for (const auto& gate : gates) ordered.push_back(gate.mid_pos);
  std::sort(ordered.begin(), ordered.end(),
            [](const auto& a, const auto& b) { return a.x < b.x; });
  ASSERT_EQ(ordered.size(), 5u);
  const auto path = smooth_and_resample(ordered, 0.5);
  ASSERT_EQ(path.size(), 17u);
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_NEAR(path[i].x, 1.0 + 0.5 * i, 1e-10);
    EXPECT_NEAR(path[i].y, 0.0, 1e-10);
  }
}

TEST(Planning, FilteringMergesDuplicatesAndRejectsBehindCar) {
  const std::vector<Point2D> cones{
      {1, 1, true}, {1, -1, false}, {3, 1, true}, {3, -1, false}, {1, 1, true}};
  EXPECT_EQ(build_filtered_delaunay(cones, 2.1, {}).size(), 2u);
  EXPECT_TRUE(build_filtered_delaunay(cones, 2.1, {4, 0, 0}).empty());
  EXPECT_EQ(build_filtered_delaunay(cones, 2.1, {4, 0, 0}, false).size(), 2u);
  EXPECT_TRUE(build_filtered_delaunay(cones, 1.9, {}).empty());
  auto conflicting = cones;
  conflicting.push_back({1, 1, false});
  EXPECT_THROW(build_filtered_delaunay(conflicting, 5, {}), std::invalid_argument);
}

TEST(Planning, SplineHandlesShortRepeatedAndInvalidInput) {
  EXPECT_TRUE(smooth_and_resample({}, 0.5).empty());
  EXPECT_EQ(smooth_and_resample({{1, 2}, {1, 2}}, 0.5).size(), 1u);
  const auto line = smooth_and_resample({{0, 0}, {1, 0}}, 0.5);
  ASSERT_EQ(line.size(), 3u);
  EXPECT_DOUBLE_EQ(line[1].x, 0.5);
  EXPECT_THROW(smooth_and_resample({{NAN, 0}}, 0.5), std::invalid_argument);
  EXPECT_THROW(smooth_and_resample({{0, 0}, {1, 0}}, 0), std::invalid_argument);
  EXPECT_THROW(smooth_and_resample({{0, 0}, {1, 0}}, 1e-300), std::length_error);
}

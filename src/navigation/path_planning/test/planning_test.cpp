// Copyright (c) 2026 UAlbertaFSAE. MIT License.
#include <gtest/gtest.h>

#include "corridor_search.hpp"
using namespace path_planning;
std::vector<BoundaryCone> straight(bool colored = true) {
  std::vector<BoundaryCone> cones;
  for (int i = 0; i < 8; ++i) {
    cones.push_back({i * .6, .46, colored ? 1 : -1});
    cones.push_back({i * .6, -.46, colored ? 0 : -1});
  }
  return cones;
}
TEST(Corridor, StraightAndUnknownCones) {
  for (bool colored : {true, false}) {
    auto cones = straight(colored);
    auto r = search_corridor(cones, {}, .92, .1, {});
    ASSERT_FALSE(r.path.empty()) << r.reason;
    EXPECT_GT(r.path.back().x, 3.5);
    for (const auto& p : r.path) EXPECT_NEAR(p.y, 0, .02);
    EXPECT_TRUE(validate_corridor(r.path, cones, {}, {}).empty());
  }
}
TEST(Corridor, PersistentOffTrackConesDoNotAttractRoute) {
  auto cones = straight();
  for (int i = 0; i < 8; ++i) {
    cones.push_back({i * .6, 1.4, -1});
    cones.push_back({i * .6, 2.32, -1});
  }
  auto r = search_corridor(cones, {}, .92, .1, {});
  ASSERT_FALSE(r.path.empty());
  for (const auto& p : r.path) EXPECT_NEAR(p.y, 0, .02);
}
TEST(Corridor, FootprintRejectsObstructionAndHonorsWidth) {
  std::vector<Point2D> path{{0, 0}, {1, 0}, {2, 0}};
  auto cones = straight();
  cones.push_back({1, .075, -1});
  EXPECT_EQ(validate_corridor(path, cones, {}, {}), "footprint_clearance");
  auto r = search_corridor(cones, {}, .92, .1, {});
  if (!r.path.empty()) {
    EXPECT_TRUE(validate_corridor(r.path, cones, {}, {}).empty());
  }
  CorridorConfig cfg;
  cfg.width = .9;
  EXPECT_EQ(validate_corridor(path, straight(), {}, cfg), "footprint_clearance");
}
TEST(Corridor, InputOrderAndDuplicatesDoNotChangeRoute) {
  auto cones = straight(false);
  auto expected = search_corridor(cones, {}, .92, .1, {});
  cones.insert(cones.end(), cones.begin(), cones.begin() + 2);
  std::reverse(cones.begin(), cones.end());
  auto actual = search_corridor(cones, {}, .92, .1, {});
  ASSERT_EQ(actual.path.size(), expected.path.size());
  for (std::size_t i = 0; i < actual.path.size(); ++i)
    EXPECT_NEAR(distance(actual.path[i], expected.path[i]), 0, 1e-9);
}
TEST(Corridor, BothTurnDirectionsWithoutCamera) {
  for (int sign : {-1, 1}) {
    std::vector<BoundaryCone> cones;
    for (int i = 0; i <= 12; ++i)
      for (int side : {-1, 1}) {
        double a = i * M_PI / 24, r = 2 - side * .46;
        cones.push_back({r * std::sin(a), sign * (2 - r * std::cos(a)), -1});
      }
    auto result = search_corridor(cones, {}, .92, .1, {});
    ASSERT_FALSE(result.path.empty()) << result.reason;
    EXPECT_GT(sign * result.path.back().y, 1.5);
    EXPECT_TRUE(validate_corridor(result.path, cones, {}, {}).empty());
  }
}
TEST(Corridor, InvalidAndInsufficientEvidenceCannotProducePath) {
  EXPECT_TRUE(search_corridor({}, {}, .92, .1, {}).path.empty());
  auto cones = straight();
  cones[0].x = NAN;
  EXPECT_TRUE(search_corridor(cones, {}, .92, .1, {}).path.empty());
  EXPECT_EQ(validate_corridor({{0, 0}, {.1, 0}, {.1, .1}}, {}, {}, {}), "steering_limit");
}

TEST(Corridor, HairpinAndSSequence) {
  std::vector<BoundaryCone> cones;
  for (int i = 0; i <= 16; ++i)
    for (int side : {-1, 1}) {
      double a = i * M_PI / 16, r = 1.5 - side * .46;
      cones.push_back({r * std::sin(a), 1.5 - r * std::cos(a), side == 1 ? 1 : 0});
    }
  auto result = search_corridor(cones, {}, .92, .1, {});
  ASSERT_FALSE(result.path.empty());
  EXPECT_GT(result.path.back().y, 2.5);
  EXPECT_TRUE(validate_corridor(result.path, cones, {}, {}).empty());
  cones.clear();
  for (int i = 0; i <= 12; ++i)
    for (int side : {-1, 1}) {
      double x = i * .5, y = .3 * std::sin(x), yaw = std::atan(.3 * std::cos(x));
      cones.push_back({x - side * .46 * std::sin(yaw), y + side * .46 * std::cos(yaw),
                       i < 2 ? (side == 1 ? 1 : 0) : -1});
    }
  CarPose pose{0, 0, std::atan(.3)};
  result = search_corridor(cones, pose, .92, .1, {});
  ASSERT_FALSE(result.path.empty());
  EXPECT_GT(result.path.back().x, 5);
  EXPECT_TRUE(validate_corridor(result.path, cones, pose, {}).empty());
}
TEST(Corridor, MemoryCannotReuseBlockedPath) {
  auto cones = straight();
  auto first = search_corridor(cones, {}, .92, .1, {});
  ASSERT_FALSE(first.path.empty());
  cones.push_back({.3, 0, -1});
  auto blocked = search_corridor(cones, {}, .92, .1, {}, first.path);
  EXPECT_TRUE(blocked.path.empty());
  auto recovered = search_corridor(straight(), {}, .92, .1, {}, first.path);
  EXPECT_FALSE(recovered.path.empty());
}
TEST(Corridor, EquallyPlausibleBranchesDoNotGuess) {
  std::vector<BoundaryCone> cones{{0, .46, 1}, {0, -.46, 0}, {1, .46, 1}, {1, -.46, 0}};
  for (int i = 2; i < 5; ++i)
    for (int sign : {-1, 1})
      for (int side : {-1, 1}) {
        double heading = sign * std::atan(.6);
        cones.push_back({i - side * .46 * std::sin(heading),
                         sign * .6 * (i - 1) + side * .46 * std::cos(heading), side == 1 ? 1 : 0});
      }
  auto result = search_corridor(cones, {}, .92, .1, {});
  EXPECT_TRUE(result.path.empty());
  EXPECT_EQ(result.reason, "ambiguous_corridor");
  std::vector<Point2D> previous;
  for (const auto& option : result.alternatives)
    if (option.rejection.empty() && option.path.back().y > .5) {
      previous = option.path;
      break;
    }
  ASSERT_FALSE(previous.empty());
  auto selected = search_corridor(cones, {}, .92, .1, {}, previous);
  ASSERT_FALSE(selected.path.empty());
  EXPECT_GT(selected.path.back().y, .5);
  for (auto& cone : cones)
    if (cone.x > 1.3) cone.y += .005;
  auto perturbed = search_corridor(cones, {}, .92, .1, {}, selected.path);
  ASSERT_FALSE(perturbed.path.empty());
  EXPECT_GT(perturbed.path.back().y, .5);
}

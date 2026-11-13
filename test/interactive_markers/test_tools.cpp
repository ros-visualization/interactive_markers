// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "interactive_markers/tools.hpp"

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::Marker;

namespace
{

constexpr double kTolerance = 1e-6;

geometry_msgs::msg::Quaternion makeQuaternion(double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

InteractiveMarker makeInteractiveMarker(float scale = 1.0f)
{
  InteractiveMarker msg;
  msg.name = "marker";
  msg.scale = scale;
  msg.pose.orientation.w = 1.0;
  return msg;
}

InteractiveMarkerControl makeControl(uint8_t interaction_mode, const std::string & name = "")
{
  InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = interaction_mode;
  control.orientation.w = 1.0;
  return control;
}

double radiusYZ(const geometry_msgs::msg::Point & p)
{
  return std::hypot(p.y, p.z);
}

// Every point of a disc must lie on its inner (0.5) or its outer (0.5 * (1 + width)) circle.
void expectPointsOnDisc(const Marker & marker, double width)
{
  const double inner = 0.5;
  const double outer = 0.5 * (1.0 + width);
  for (const auto & p : marker.points) {
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    const double r = radiusYZ(p);
    EXPECT_TRUE(std::abs(r - inner) < kTolerance || std::abs(r - outer) < kTolerance) <<
      "point at radius " << r;
  }
}

}  // namespace

TEST(TestTools, assignDefaultColorFollowsXAxis)
{
  const double s = std::sqrt(0.5);
  Marker marker;

  // Identity: x axis stays along x -> red.
  interactive_markers::assignDefaultColor(marker, makeQuaternion(0, 0, 0, 1));
  EXPECT_FLOAT_EQ(marker.color.r, 1.0f);
  EXPECT_FLOAT_EQ(marker.color.g, 0.0f);
  EXPECT_FLOAT_EQ(marker.color.b, 0.0f);
  EXPECT_FLOAT_EQ(marker.color.a, 0.5f);

  // 90 degrees around z: x axis points along y -> green.
  interactive_markers::assignDefaultColor(marker, makeQuaternion(0, 0, s, s));
  EXPECT_NEAR(marker.color.r, 0.0f, kTolerance);
  EXPECT_NEAR(marker.color.g, 1.0f, kTolerance);
  EXPECT_NEAR(marker.color.b, 0.0f, kTolerance);

  // 90 degrees around y: x axis points along -z -> blue.
  interactive_markers::assignDefaultColor(marker, makeQuaternion(0, s, 0, s));
  EXPECT_NEAR(marker.color.r, 0.0f, kTolerance);
  EXPECT_NEAR(marker.color.g, 0.0f, kTolerance);
  EXPECT_NEAR(marker.color.b, 1.0f, kTolerance);

  // 45 degrees around z: x axis between x and y -> yellow.
  const double half = std::atan(1.0) / 2.0;  // pi / 8
  interactive_markers::assignDefaultColor(
    marker, makeQuaternion(0, 0, std::sin(half), std::cos(half)));
  EXPECT_NEAR(marker.color.r, 1.0f, kTolerance);
  EXPECT_NEAR(marker.color.g, 1.0f, kTolerance);
  EXPECT_NEAR(marker.color.b, 0.0f, kTolerance);
  EXPECT_FLOAT_EQ(marker.color.a, 0.5f);
}

TEST(TestTools, makeArrow)
{
  const InteractiveMarker msg = makeInteractiveMarker(2.0f);
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::MOVE_AXIS);

  interactive_markers::makeArrow(msg, control, 1.0f);
  interactive_markers::makeArrow(msg, control, -2.0f);
  ASSERT_EQ(control.markers.size(), 2u);

  const Marker & forward = control.markers[0];
  EXPECT_EQ(forward.type, Marker::ARROW);
  EXPECT_NEAR(forward.scale.x, 0.3, kTolerance);
  EXPECT_NEAR(forward.scale.y, 0.5, kTolerance);
  EXPECT_NEAR(forward.scale.z, 0.4, kTolerance);
  EXPECT_EQ(forward.pose.orientation, control.orientation);
  EXPECT_FLOAT_EQ(forward.color.r, 1.0f);
  EXPECT_FLOAT_EQ(forward.color.a, 0.5f);
  ASSERT_EQ(forward.points.size(), 2u);
  // inner = 0.5 * |pos|, outer = inner + 0.4, both multiplied by the marker scale.
  EXPECT_NEAR(forward.points[0].x, 1.0, kTolerance);
  EXPECT_NEAR(forward.points[1].x, 1.8, kTolerance);

  const Marker & backward = control.markers[1];
  ASSERT_EQ(backward.points.size(), 2u);
  EXPECT_NEAR(backward.points[0].x, -2.0, kTolerance);
  EXPECT_NEAR(backward.points[1].x, -2.8, kTolerance);
}

TEST(TestTools, makeDiscForMovePlane)
{
  const InteractiveMarker msg = makeInteractiveMarker(2.0f);
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::MOVE_PLANE);

  interactive_markers::makeDisc(msg, control);
  ASSERT_EQ(control.markers.size(), 1u);

  const Marker & disc = control.markers[0];
  EXPECT_EQ(disc.type, Marker::TRIANGLE_LIST);
  EXPECT_DOUBLE_EQ(disc.scale.x, 2.0);
  EXPECT_DOUBLE_EQ(disc.scale.y, 2.0);
  EXPECT_DOUBLE_EQ(disc.scale.z, 2.0);
  EXPECT_FLOAT_EQ(disc.color.r, 1.0f);
  EXPECT_FLOAT_EQ(disc.color.a, 0.5f);
  // 36 steps, 2 triangles per step, a single color for the whole disc.
  EXPECT_EQ(disc.points.size(), 6u * 36u);
  EXPECT_TRUE(disc.colors.empty());
  expectPointsOnDisc(disc, 0.3);
}

TEST(TestTools, makeDiscForRotateAxis)
{
  const InteractiveMarker msg = makeInteractiveMarker();
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::ROTATE_AXIS);

  interactive_markers::makeDisc(msg, control, 0.5);
  ASSERT_EQ(control.markers.size(), 1u);

  const Marker & disc = control.markers[0];
  EXPECT_EQ(disc.points.size(), 6u * 36u);
  ASSERT_EQ(disc.colors.size(), 2u * 36u);
  expectPointsOnDisc(disc, 0.5);

  // The segments alternate between a darker and the full base color.
  EXPECT_FLOAT_EQ(disc.colors[0].r, 0.6f);
  EXPECT_FLOAT_EQ(disc.colors[1].r, 0.6f);
  EXPECT_FLOAT_EQ(disc.colors[2].r, 1.0f);
  EXPECT_FLOAT_EQ(disc.colors[3].r, 1.0f);
  EXPECT_FLOAT_EQ(disc.colors[0].a, 1.0f);
}

TEST(TestTools, makeDiscForMoveRotate)
{
  const InteractiveMarker msg = makeInteractiveMarker();
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::MOVE_ROTATE);

  interactive_markers::makeDisc(msg, control);
  ASSERT_EQ(control.markers.size(), 1u);

  const Marker & disc = control.markers[0];
  EXPECT_EQ(disc.points.size(), 6u * 36u);
  ASSERT_EQ(disc.colors.size(), 2u * 36u);
  expectPointsOnDisc(disc, 0.3);

  // Darker ring segments alternate with arrow heads in the base color.
  EXPECT_FLOAT_EQ(disc.colors[0].r, 0.6f);
  EXPECT_FLOAT_EQ(disc.colors[0].a, 1.0f);
  EXPECT_FLOAT_EQ(disc.colors[2].r, 1.0f);
  EXPECT_FLOAT_EQ(disc.colors[2].a, 0.5f);
}

TEST(TestTools, makeViewFacingButton)
{
  const InteractiveMarker msg = makeInteractiveMarker(2.0f);
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::MENU);
  control.independent_marker_orientation = true;

  interactive_markers::makeViewFacingButton(msg, control, "click me");

  EXPECT_EQ(control.orientation_mode, InteractiveMarkerControl::VIEW_FACING);
  EXPECT_FALSE(control.independent_marker_orientation);
  ASSERT_EQ(control.markers.size(), 1u);

  const Marker & button = control.markers[0];
  EXPECT_EQ(button.type, Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(button.text, "click me");
  EXPECT_NEAR(button.scale.x, 0.5, kTolerance);
  EXPECT_NEAR(button.scale.y, 0.5, kTolerance);
  EXPECT_NEAR(button.scale.z, 0.5, kTolerance);
  EXPECT_NEAR(button.pose.position.x, -0.05, kTolerance);
  EXPECT_NEAR(button.pose.position.z, 2.35, kTolerance);
  EXPECT_FLOAT_EQ(button.color.a, 1.0f);
}

TEST(TestTools, makeTitle)
{
  InteractiveMarker msg = makeInteractiveMarker(2.0f);
  msg.description = "my marker";

  const InteractiveMarkerControl control = interactive_markers::makeTitle(msg);

  EXPECT_EQ(control.interaction_mode, InteractiveMarkerControl::NONE);
  EXPECT_EQ(control.orientation_mode, InteractiveMarkerControl::VIEW_FACING);
  EXPECT_TRUE(control.always_visible);
  EXPECT_DOUBLE_EQ(control.orientation.w, 1.0);
  ASSERT_EQ(control.markers.size(), 1u);

  const Marker & title = control.markers[0];
  EXPECT_EQ(title.type, Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(title.text, "my marker");
  EXPECT_EQ(title.ns, "marker");
  EXPECT_NEAR(title.scale.z, 0.3, kTolerance);
  EXPECT_NEAR(title.pose.position.z, 2.8, kTolerance);
  EXPECT_DOUBLE_EQ(title.pose.orientation.w, 1.0);
  EXPECT_FLOAT_EQ(title.color.a, 1.0f);
}

TEST(TestTools, uniqueifyControlNames)
{
  InteractiveMarker msg = makeInteractiveMarker();
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "a"));
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "b"));
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "a"));
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "a"));

  interactive_markers::uniqueifyControlNames(msg);

  EXPECT_EQ(msg.controls[0].name, "a");
  EXPECT_EQ(msg.controls[1].name, "b");
  EXPECT_EQ(msg.controls[2].name, "a_u0");
  EXPECT_EQ(msg.controls[3].name, "a_u1");
}

TEST(TestTools, uniqueifyControlNamesAvoidsExistingSuffix)
{
  InteractiveMarker msg = makeInteractiveMarker();
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "a"));
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "a_u0"));
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE, "a"));

  interactive_markers::uniqueifyControlNames(msg);

  EXPECT_EQ(msg.controls[0].name, "a");
  EXPECT_EQ(msg.controls[1].name, "a_u0");
  std::set<std::string> names;
  for (const auto & control : msg.controls) {
    names.insert(control.name);
  }
  EXPECT_EQ(names.size(), msg.controls.size());
}

TEST(TestTools, autoCompleteIgnoresMarkerWithoutControls)
{
  // A marker without controls is a delete request and is left untouched.
  InteractiveMarker msg;
  msg.name = "marker";
  msg.pose.orientation = makeQuaternion(0, 0, 0, 0);

  interactive_markers::autoComplete(msg);

  EXPECT_FLOAT_EQ(msg.scale, 0.0f);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.w, 0.0);
}

TEST(TestTools, autoCompleteFillsScaleAndOrientation)
{
  InteractiveMarker msg;
  msg.name = "marker";
  msg.pose.orientation = makeQuaternion(0, 0, 0, 0);
  msg.controls.push_back(makeControl(InteractiveMarkerControl::NONE));

  interactive_markers::autoComplete(msg);

  EXPECT_FLOAT_EQ(msg.scale, 1.0f);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.w, 1.0);
}

TEST(TestTools, autoCompleteNormalizesOrientation)
{
  InteractiveMarker msg = makeInteractiveMarker(3.0f);
  msg.pose.orientation = makeQuaternion(1, 0, 0, 1);
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::NONE);
  control.orientation = makeQuaternion(0, 0, 0, 0);
  msg.controls.push_back(control);

  interactive_markers::autoComplete(msg);

  EXPECT_FLOAT_EQ(msg.scale, 3.0f);
  EXPECT_NEAR(msg.pose.orientation.x, std::sqrt(0.5), kTolerance);
  EXPECT_NEAR(msg.pose.orientation.w, std::sqrt(0.5), kTolerance);
  // An empty control orientation becomes the identity.
  EXPECT_DOUBLE_EQ(msg.controls[0].orientation.w, 1.0);
}

TEST(TestTools, autoCompleteAddsDefaultMarkersPerInteractionMode)
{
  struct Case
  {
    uint8_t mode;
    size_t num_markers;
    int32_t marker_type;
  };
  const Case cases[] = {
    {InteractiveMarkerControl::NONE, 0, 0},
    {InteractiveMarkerControl::BUTTON, 0, 0},
    {InteractiveMarkerControl::MOVE_AXIS, 2, Marker::ARROW},
    {InteractiveMarkerControl::MOVE_PLANE, 1, Marker::TRIANGLE_LIST},
    {InteractiveMarkerControl::ROTATE_AXIS, 1, Marker::TRIANGLE_LIST},
    {InteractiveMarkerControl::MOVE_ROTATE, 1, Marker::TRIANGLE_LIST},
    {InteractiveMarkerControl::MENU, 1, Marker::TEXT_VIEW_FACING},
    {42, 0, 0},  // Unknown interaction mode.
  };

  for (const auto & c : cases) {
    SCOPED_TRACE("interaction_mode " + std::to_string(c.mode));
    InteractiveMarker msg = makeInteractiveMarker();
    InteractiveMarkerControl control = makeControl(c.mode);
    control.description = "menu";

    interactive_markers::autoComplete(msg, control);

    ASSERT_EQ(control.markers.size(), c.num_markers);
    for (const auto & marker : control.markers) {
      EXPECT_EQ(marker.type, c.marker_type);
      EXPECT_EQ(marker.ns, "marker");
    }
  }
}

TEST(TestTools, autoCompleteMenuUsesControlDescription)
{
  InteractiveMarker msg = makeInteractiveMarker();
  msg.controls.push_back(makeControl(InteractiveMarkerControl::MENU, "menu"));
  msg.controls[0].description = "Options";

  interactive_markers::autoComplete(msg);

  const InteractiveMarkerControl & control = msg.controls[0];
  EXPECT_EQ(control.orientation_mode, InteractiveMarkerControl::VIEW_FACING);
  ASSERT_EQ(control.markers.size(), 1u);
  EXPECT_EQ(control.markers[0].text, "Options");
}

TEST(TestTools, autoCompleteKeepsAndCompletesUserMarkers)
{
  InteractiveMarker msg = makeInteractiveMarker();
  msg.name = "box";
  InteractiveMarkerControl control = makeControl(InteractiveMarkerControl::MOVE_AXIS);

  Marker zero_scale;
  zero_scale.type = Marker::CUBE;
  zero_scale.ns = "user_ns";
  zero_scale.pose.orientation = makeQuaternion(0, 0, 0, 0);
  Marker custom;
  custom.type = Marker::SPHERE;
  custom.scale.x = 0.1;
  custom.scale.y = 0.2;
  custom.scale.z = 0.3;
  custom.pose.orientation = makeQuaternion(0, 0, 2, 0);
  control.markers = {zero_scale, custom};

  interactive_markers::autoComplete(msg, control);

  // The user markers are kept: no default arrows are added.
  ASSERT_EQ(control.markers.size(), 2u);

  EXPECT_EQ(control.markers[0].type, Marker::CUBE);
  EXPECT_EQ(control.markers[0].ns, "box");
  EXPECT_DOUBLE_EQ(control.markers[0].scale.x, 1.0);
  EXPECT_DOUBLE_EQ(control.markers[0].scale.y, 1.0);
  EXPECT_DOUBLE_EQ(control.markers[0].scale.z, 1.0);
  EXPECT_DOUBLE_EQ(control.markers[0].pose.orientation.w, 1.0);

  EXPECT_EQ(control.markers[1].type, Marker::SPHERE);
  EXPECT_EQ(control.markers[1].ns, "box");
  EXPECT_DOUBLE_EQ(control.markers[1].scale.x, 0.1);
  EXPECT_DOUBLE_EQ(control.markers[1].scale.y, 0.2);
  EXPECT_DOUBLE_EQ(control.markers[1].scale.z, 0.3);
  EXPECT_DOUBLE_EQ(control.markers[1].pose.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(control.markers[1].pose.orientation.w, 0.0);

  EXPECT_NE(control.markers[0].id, control.markers[1].id);
}

TEST(TestTools, autoCompleteTransparency)
{
  InteractiveMarker msg = makeInteractiveMarker();

  InteractiveMarkerControl transparent = makeControl(InteractiveMarkerControl::MOVE_AXIS);
  interactive_markers::autoComplete(msg, transparent, true);
  ASSERT_EQ(transparent.markers.size(), 2u);
  EXPECT_FLOAT_EQ(transparent.markers[0].color.a, 0.5f);

  InteractiveMarkerControl opaque = makeControl(InteractiveMarkerControl::MOVE_AXIS);
  Marker invisible;
  invisible.color.a = 0.0f;
  Marker translucent;
  translucent.color.a = 0.3f;
  opaque.markers = {invisible, translucent};
  interactive_markers::autoComplete(msg, opaque, false);
  // Semi-transparent markers become opaque, fully transparent ones stay invisible.
  EXPECT_FLOAT_EQ(opaque.markers[0].color.a, 0.0f);
  EXPECT_FLOAT_EQ(opaque.markers[1].color.a, 1.0f);

  InteractiveMarkerControl defaults = makeControl(InteractiveMarkerControl::MOVE_AXIS);
  interactive_markers::autoComplete(msg, defaults, false);
  ASSERT_EQ(defaults.markers.size(), 2u);
  EXPECT_FLOAT_EQ(defaults.markers[0].color.a, 1.0f);
  EXPECT_FLOAT_EQ(defaults.markers[1].color.a, 1.0f);
}

TEST(TestTools, autoCompleteMakesControlNamesUnique)
{
  InteractiveMarker msg = makeInteractiveMarker();
  msg.controls.push_back(makeControl(InteractiveMarkerControl::MOVE_AXIS, "move"));
  msg.controls.push_back(makeControl(InteractiveMarkerControl::MOVE_AXIS, "move"));

  interactive_markers::autoComplete(msg);

  ASSERT_EQ(msg.controls.size(), 2u);
  EXPECT_EQ(msg.controls[0].name, "move");
  EXPECT_EQ(msg.controls[1].name, "move_u0");
  EXPECT_EQ(msg.controls[0].markers.size(), 2u);
  EXPECT_EQ(msg.controls[1].markers.size(), 2u);
}

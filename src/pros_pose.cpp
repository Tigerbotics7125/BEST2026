//  ___            ___
// | _ \_ _ ___ __| _ \___ ___ ___
// |  _/ '_/ _ (_-<  _/ _ (_-</ -_)
// |_| |_| \___/__/_| \___/__/\___|
//

// -----------------------------------------------------------------------------
//   Simple homography-based AprilTag pose estimation
// -----------------------------------------------------------------------------

#include "pros_pose/pros_pose.hpp"

// ━━ External libraries ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#include <cmath>

// ━━ Internal modules ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#include "pros_pose/homography.h"
#include "pros_pose/zarray.h"

// -----------------------------------------------------------------------------
//   Constructor
// -----------------------------------------------------------------------------

ProsPose::ProsPose(double fx, double fy, double cx, double cy, double tag_size)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), tag_size_(tag_size) {}

// -----------------------------------------------------------------------------
//   Pose estimation
// -----------------------------------------------------------------------------

ProsPoseResult ProsPose::estimate(double corners[4][2]) {
  // Standard AprilTag object-space coordinates (Y pointing DOWN)
  float obj_pts[4][2] = {
      {-1.0f, -1.0f}, // Top-Left
      {1.0f, -1.0f},  // Top-Right
      {1.0f, 1.0f},   // Bottom-Right
      {-1.0f, 1.0f},  // Bottom-Left
  };

  zarray_t *corres = zarray_create(sizeof(float[4]));
  for (int i = 0; i < 4; i++) {
    float corr[4] = {obj_pts[i][0], obj_pts[i][1], (float)corners[i][0],
                     (float)corners[i][1]};
    zarray_add(corres, corr);
  }

  matd_t *H = homography_compute(corres, 0);
  zarray_destroy(corres);

  // Guard against homography compute failures (P0)
  if (H == nullptr) {
    return ProsPoseResult{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
  }

  double scale = tag_size_ / 2.0;
  matd_t *M_H = homography_to_pose(H, -fx_, fy_, cx_, cy_);
  matd_destroy(H);

  // Guard against homography decomposition failures (P0)
  if (M_H == nullptr) {
    return ProsPoseResult{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
  }

  MATD_EL(M_H, 0, 3) *= scale;
  MATD_EL(M_H, 1, 3) *= scale;
  MATD_EL(M_H, 2, 3) *= scale;

  matd_t *fix = matd_create(4, 4);
  MATD_EL(fix, 0, 0) = 1;
  MATD_EL(fix, 1, 1) = -1;
  MATD_EL(fix, 2, 2) = -1;
  MATD_EL(fix, 3, 3) = 1;

  matd_t *initial_pose = matd_multiply(fix, M_H);
  matd_destroy(M_H);
  matd_destroy(fix);

  if (initial_pose == nullptr) {
    return ProsPoseResult{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
  }

  ProsPoseResult pose;

  // ━━ 1. Translation extraction ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  pose.x = MATD_EL(initial_pose, 0, 3);
  pose.y = MATD_EL(initial_pose, 1, 3);
  pose.z = MATD_EL(initial_pose, 2, 3);

  // ━━ 2. Rotation matrix elements ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  double r10 = MATD_EL(initial_pose, 1, 0);
  double r11 = MATD_EL(initial_pose, 1, 1);
  double r12 = MATD_EL(initial_pose, 1, 2);
  double r02 = MATD_EL(initial_pose, 0, 2);
  double r22 = MATD_EL(initial_pose, 2, 2);

  // ━━ 3. Camera-Frame Euler angles decomposition ━━━━━━━━━━━━━━━━━━━━
  //   We decompose the rotation matrix into a Yaw-Pitch-Roll (Y-X-Z) sequence.
  //   In standard camera coordinates (Z-forward, X-right, Y-down):
  //     • Yaw (horizontal panning) is rotation about the vertical Y-axis.
  //       Formula: yaw = atan2(-r02, r22)
  //     • Pitch (vertical tilting) is rotation about the lateral X-axis.
  //       Formula: pitch = atan2(-r12, sqrt(r10*r10 + r11*r11))
  //     • Roll (camera roll/spin) is rotation about the optical Z-axis.
  //       Formula: roll = atan2(r10, r11)
  double pitch_rad = std::atan2(-r12, std::sqrt(r10 * r10 + r11 * r11));
  double roll_rad = std::atan2(r10, r11);
  double yaw_rad = std::atan2(-r02, r22);

  // ━━ 4. Unit conversion ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  pose.yaw = yaw_rad * (180.0 / M_PI);
  pose.pitch = pitch_rad * (180.0 / M_PI);
  pose.roll = roll_rad * (180.0 / M_PI);
  pose.is_valid = true;

  matd_destroy(initial_pose);

  return pose;
}

// -----------------------------------------------------------------------------
//   REFERENCES / WORKS CITED
// ----------------------------------------------------------------------------
//
// Sources:
//   • AprilTag 3 - Homography to Pose & Coordinate Ordering
//     https://github.com/AprilRobotics/apriltag
//     Convention: Point 0 = (-s, -s), Point 1 = (s, -s),
//                 Point 2 = (s, s),   Point 3 = (-s, s) [Y-down]
//   • VEX AI Vision & PROS C API for detected AprilTags
//   (aivision_object_tag_s_t)
//     https://pros.cs.purdue.edu/v5/api/cpp/aivision.html
//     Corner sequence matches standard clockwise sequence starting top-left.
//
//   • VEX AI Vision comparison
//     https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor

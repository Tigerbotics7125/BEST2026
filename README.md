# BEST2026
This is the code for the Byte to Bite game.
# pros_pose

Simple homography-based AprilTag pose estimation for VEX V5, defaults are measured for AI Vision Sensor and Override

## Installation

```bash
pros conduct fetch <path_to_zip_from_releases>
pros conduct apply pros_pose
```

## Usage

```cpp
#include "main.h"
#include "pros_pose/pros_pose.hpp"

void opcontrol() {
    ProsPose pose_estimator;  // defaults: VEX AI Vision (320x240, 74° HFOV, 18mm tags)
    pros::Vision vision_sensor(1);

    while (true) {
        pros::vision_object_s_t tag = vision_sensor.get_by_sig(0, 1);

        if (tag.signature != VISION_OBJECT_ERR_SIG) {
            double corners[4][2] = {
                {tag.left_coord, tag.top_coord},
                {tag.left_coord + tag.width, tag.top_coord},
                {tag.left_coord + tag.width, tag.top_coord + tag.height},
                {tag.left_coord, tag.top_coord + tag.height}
            };

            ProsPoseResult result = pose_estimator.estimate(corners);

            if (result.is_valid) {
                // result.x, result.y, result.z in meters
                // result.yaw, result.pitch, result.roll in degrees
            }
        }

        pros::delay(20);
    }
}
```

## Constructor Parameters

| Parameter  | Default | Description                                                    |
| ---------- | ------- | -------------------------------------------------------------- |
| `fx`       | 212.3   | Focal length X in pixels: `(sensor_width / 2) / tan(HFOV / 2)` |
| `fy`       | 195.82  | Focal length Y in pixels                                       |
| `cx`       | 160.0   | Principal point X (optical center)                             |
| `cy`       | 120.0   | Principal point Y (optical center)                             |
| `tag_size` | 0.018   | Physical tag width in meters (18 mm)                           |

focal lengths calculated based on [This](https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor)

## Output (ProsPoseResult)

| Field      | Unit    | Description                      |
| ---------- | ------- | -------------------------------- |
| `x`        | meters  | Translation X relative to camera |
| `y`        | meters  | Translation Y relative to camera |
| `z`        | meters  | Translation Z relative to camera |
| `yaw`      | degrees | Rotation about Y axis            |
| `pitch`    | degrees | Rotation about X axis            |
| `roll`     | degrees | Rotation about Z axis            |
| `is_valid` | bool    | True if estimation succeeded     |

## Credits

Core math from [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) (BSD 2-Clause).

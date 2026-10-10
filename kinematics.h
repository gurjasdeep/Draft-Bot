#pragma once

/*
CURRENT LIMITS AS IT SEEMS:
Bottom-left: (-84, 200)
Top-right: (142, 264)
Size: 226 × 64 mm
TODO: test this physically.
*/

// Distance between the two fixed motor shafts.
//
// Example:
//     150 mm
//
constexpr float BASE_DISTANCE = 42.5f;

// Minimum clearance height used to set the left-link angle limit.
constexpr float LINK_WIDTH = 20.0f;

// Maximum left-link angle, measured counterclockwise from +X.
constexpr float LEFT_HOME_ANGLE_DEGREES = 180.0f;

// Minimum right-link angle, measured counterclockwise from +X.
constexpr float RIGHT_HOME_ANGLE_DEGREES = 0.0f;


// Motor → passive joint.
//
// This is the first link on each side.
//
constexpr float L1 = 100.0f;


// Passive joint → central revolute joint.
//
// This is the normal second link length.
//
constexpr float L2 = 132.0f;


// Distance from the central revolute joint to the pen.
//
// This is the extra part of the second link extending beyond C.
//
constexpr float PEN_OFFSET = 68.0f;


// Small tolerance used when dealing with floating-point
// calculations.
//
// Computers cannot represent most decimal numbers exactly.
// Because of that, something that mathematically should be
// zero might become:
//
//     -0.0000001
//
// instead.
//
constexpr float EPSILON = 0.00001f;





// ============================================================
//                       POINT STRUCT
// ============================================================

struct Point2D
{
    float x;
    float y;
};



// ============================================================
//                    IK RESULT STRUCT
// ============================================================
//
// Besides the motor angles, we return the intermediate
// positions.
//
// This is extremely useful while developing the robot.
//
// You can print these values over Serial and compare them
// against your CAD model.
//


struct IKResult
{
    bool valid;

    // Left and right motor angles.
    float leftAngle;
    float rightAngle;

    // Intermediate joint positions.
    Point2D leftElbow;
    Point2D rightElbow;
    Point2D center;
};

IKResult inverseKinematics(float penX, float penY);
bool forwardKinematics(
    float leftAngle,
    float rightAngle,
    Point2D& penPoint
);

#include "kinematics.h"
#include <math.h>


// ============================================================
//                 ROBOT GEOMETRY
// ============================================================
//
// Coordinate system:
//
//
//                         +Y
//                          ↑
//                          |
//                          |
//                          |
//          P ●             |
//           /              |
//          /               |
//         C ●              |
//        /   \             |
//       /     \            |
//      B       E           |
//     /         \          |
//    /           \         |
//   A             D        |
//   ●-------------●-------→ +X
//
//
// A = LEFT motor
// D = RIGHT motor
//
// B = LEFT passive revolute joint
// E = RIGHT passive revolute joint
//
// C = CENTRAL revolute joint
//
// P = PEN TIP
//
//
// All distances are millimetres.
//
// All angles returned by this implementation are in RADIANS.
//
// ============================================================




// ============================================================
//            INVERSE KINEMATICS FOR THE 5-BAR
// ============================================================
//
// Input:
//
//     penX
//     penY
//
// These are the desired coordinates of the PEN TIP.
//
// Output:
//
//     leftAngle
//     rightAngle
//
// These are the required motor shaft angles.
//
// ============================================================
//
// IMPORTANT:
//
// The pen is NOT located directly at the central joint.
//
// Instead:
//
//
//      B ───────── C ─────── P
//       \         /          ↑
//        \       /           │
//         \     /            │
//          \   /             PEN_OFFSET
//
// Therefore we cannot simply perform ordinary 5-bar IK on
// (penX, penY).
//
// We must account for the extension beyond C.
//
// ============================================================

IKResult inverseKinematics(float penX, float penY)
{
    IKResult result{};
    result.valid = false;

    // ========================================================
    // 1. FIXED MOTOR POSITIONS
    // ========================================================

    const Point2D leftMotor  = {0.0f, 0.0f};
    const Point2D rightMotor = {BASE_DISTANCE, 0.0f};

    // Total distance from the left elbow to the pen.
    // B -> P = L2 + PEN_OFFSET

    const float totalLength = L2 + PEN_OFFSET;

    // ========================================================
    // 2. CALCULATE DISTANCE AND DIRECTION TO THE PEN
    // ========================================================

    const float penDistance = sqrtf(
        penX * penX +
        penY * penY
    );

    // Reject impossible positions.
    if (penDistance < EPSILON)
        return result;

    if (penDistance > L1 + totalLength + EPSILON)
        return result;

    if (penDistance < fabsf(L1 - totalLength) - EPSILON)
        return result;

    const float penDirection = atan2f(penY, penX);

    // ========================================================
    // 3. CALCULATE THE LEFT MOTOR ANGLE
    // ========================================================
    //
    // Triangle:
    //
    // A = left motor
    // B = left elbow
    // P = pen
    //
    // AB = L1
    // BP = L2 + PEN_OFFSET
    // AP = penDistance
    //
    // Use the cosine rule to calculate the angle
    // between A -> P and A -> B.
    // ========================================================

    float cosLeftOffset =
        (
            L1 * L1 +
            penDistance * penDistance -
            totalLength * totalLength
        )
        /
        (
            2.0f * L1 * penDistance
        );

    // Protect acosf() against floating-point errors.

    cosLeftOffset = fmaxf(
        -1.0f,
        fminf(1.0f, cosLeftOffset)
    );

    const float leftOffset = acosf(cosLeftOffset);

    // Select the required left elbow configuration.
    constexpr float ELBOW_SIGN = +1.0f;

    result.leftAngle =
        penDirection + ELBOW_SIGN * leftOffset;

    // ========================================================
    // 4. CALCULATE THE LEFT ELBOW COORDINATES
    // ========================================================

    result.leftElbow.x =
        L1 * cosf(result.leftAngle);

    result.leftElbow.y =
        L1 * sinf(result.leftAngle);

    // ========================================================
    // 5. CALCULATE THE ANGLE OF THE LEFT L2 LINK
    // ========================================================
    //
    // B -> C -> P are collinear.
    //
    // Therefore:
    //
    // angle(B -> C) = angle(B -> P)
    //
    // Calculate the direction from the left elbow
    // directly to the requested pen position.
    // ========================================================

    const float BPx =
        penX - result.leftElbow.x;

    const float BPy =
        penY - result.leftElbow.y;

    const float BPdistance =
        sqrtf(BPx * BPx + BPy * BPy);

    if (BPdistance < EPSILON)
        return result;

    const float thetaL2 = atan2f(BPy, BPx);

    // ========================================================
    // 6. CALCULATE THE PEN OFFSET: DELTA X AND DELTA Y
    // ========================================================
    //
    // The pen extends beyond the central joint C.
    //
    // C -------- P
    //      E
    //
    // E = PEN_OFFSET
    //
    // deltaX = E * cos(thetaL2)
    // deltaY = E * sin(thetaL2)
    //
    // ========================================================

    const float deltaX =
        PEN_OFFSET * cosf(thetaL2);

    const float deltaY =
        PEN_OFFSET * sinf(thetaL2);

    // ========================================================
    // 7. CALCULATE THE CENTRAL JOINT
    // ========================================================
    //
    // P = C + (deltaX, deltaY)
    //
    // Therefore:
    //
    // Cx = Px - deltaX
    // Cy = Py - deltaY
    //
    // This is your offset-correction method.
    // ========================================================

    result.center.x = penX - deltaX;
    result.center.y = penY - deltaY;

    // ========================================================
    // 8. VALIDATE THE LEFT L2 LINK
    // ========================================================

    const float leftL2x =
        result.center.x - result.leftElbow.x;

    const float leftL2y =
        result.center.y - result.leftElbow.y;

    const float leftL2distance =
        sqrtf(leftL2x * leftL2x + leftL2y * leftL2y);

    if (fabsf(leftL2distance - L2) > 0.01f)
        return result;

    // ========================================================
    // 9. SOLVE THE RIGHT ARM
    // ========================================================
    //
    // The central joint is now known.
    //
    // Solve the right two-link arm using the cosine rule.
    // ========================================================

    const float dx =
        result.center.x - rightMotor.x;

    const float dy =
        result.center.y - rightMotor.y;

    const float rightDistance =
        sqrtf(dx * dx + dy * dy);

    if (rightDistance < EPSILON)
        return result;

    if (rightDistance > L1 + L2 + EPSILON)
        return result;

    if (rightDistance < fabsf(L1 - L2) - EPSILON)
        return result;

    const float rightDirection =
        atan2f(dy, dx);

    float cosRightOffset =
        (
            L1 * L1 +
            rightDistance * rightDistance -
            L2 * L2
        )
        /
        (
            2.0f * L1 * rightDistance
        );

    cosRightOffset = fmaxf(
        -1.0f,
        fminf(1.0f, cosRightOffset)
    );

    const float rightOffset =
        acosf(cosRightOffset);

    // Select the required right elbow configuration.
    constexpr float RIGHT_ELBOW_SIGN = -1.0f;

    result.rightAngle =
        rightDirection +
        RIGHT_ELBOW_SIGN * rightOffset;

    // ========================================================
    // 10. CALCULATE THE RIGHT ELBOW
    // ========================================================

    result.rightElbow.x =
        rightMotor.x +
        L1 * cosf(result.rightAngle);

    result.rightElbow.y =
        rightMotor.y +
        L1 * sinf(result.rightAngle);

    // ========================================================
    // 11. VALIDATE THE RIGHT L2 LINK
    // ========================================================

    const float rightL2x =
        result.center.x - result.rightElbow.x;

    const float rightL2y =
        result.center.y - result.rightElbow.y;

    const float rightL2distance =
        sqrtf(rightL2x * rightL2x + rightL2y * rightL2y);

    if (fabsf(rightL2distance - L2) > 0.01f)
        return result;

    // ========================================================
    // 12. VALIDATE THE PEN OFFSET
    // ========================================================

    const float actualPenX =
        result.center.x + deltaX;

    const float actualPenY =
        result.center.y + deltaY;

    const float penError =
        sqrtf(
            (actualPenX - penX) * (actualPenX - penX) +
            (actualPenY - penY) * (actualPenY - penY)
        );

    if (penError > 0.01f)
        return result;

    // ========================================================
    // 13. SUCCESS
    // ========================================================

    result.valid = true;

    return result;
}

bool forwardKinematics(
    float leftAngle,
    float rightAngle,
    Point2D& penPoint
)
{
    const Point2D leftElbow =
    {
        L1 * cos(leftAngle),
        L1 * sin(leftAngle)
    };

    const Point2D rightElbow =
    {
        BASE_DISTANCE + L1 * cos(rightAngle),
        L1 * sin(rightAngle)
    };

    const float dx = rightElbow.x - leftElbow.x;
    const float dy = rightElbow.y - leftElbow.y;
    const float elbowDistance = sqrt(dx * dx + dy * dy);

    if (elbowDistance < EPSILON ||
        elbowDistance > 2.0f * L2 + EPSILON)
    {
        return false;
    }

    const float lineAngle = atan2f(dy, dx);
    float halfAngleCosine = elbowDistance / (2.0f * L2);
    if (halfAngleCosine > 1.0f)
    {
        halfAngleCosine = 1.0f;
    }

    const float halfAngle = acosf(halfAngleCosine);

    const Point2D centers[] =
    {
        {
            leftElbow.x + L2 * cosf(lineAngle + halfAngle),
            leftElbow.y + L2 * sinf(lineAngle + halfAngle)
        },
        {
            leftElbow.x + L2 * cosf(lineAngle - halfAngle),
            leftElbow.y + L2 * sinf(lineAngle - halfAngle)
        }
    };

    bool found = false;
    float bestAngleError = 0.0f;

    for (int i = 0; i < 2; ++i)
    {
        const float directionX =
            (centers[i].x - leftElbow.x) / L2;
        const float directionY =
            (centers[i].y - leftElbow.y) / L2;

        const Point2D candidatePen =
        {
            centers[i].x + PEN_OFFSET * directionX,
            centers[i].y + PEN_OFFSET * directionY
        };

        const IKResult candidate =
            inverseKinematics(candidatePen.x, candidatePen.y);
        if (!candidate.valid)
        {
            continue;
        }

        const float leftError = atan2(
            sin(candidate.leftAngle - leftAngle),
            cos(candidate.leftAngle - leftAngle)
        );
        const float rightError = atan2(
            sin(candidate.rightAngle - rightAngle),
            cos(candidate.rightAngle - rightAngle)
        );
        const float angleError =
            leftError * leftError + rightError * rightError;

        if (!found || angleError < bestAngleError)
        {
            penPoint = candidatePen;
            bestAngleError = angleError;
            found = true;
        }
    }

    return found;
}

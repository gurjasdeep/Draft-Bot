#include "kinematics.h"
#include <stdint.h>
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

    // Assume failure initially.
    //
    // Only set this to true after every geometric calculation
    // succeeds.
    //
    result.valid = false;


    // ========================================================
    // FIXED MOTOR POSITIONS
    // ========================================================
    //
    // We choose the coordinate system so that:
    //
    //     Left motor  = (0, 0)
    //
    // and:
    //
    //     Right motor = (BASE_DISTANCE, 0)
    //
    // This makes the mathematics considerably simpler.
    //
    // ========================================================

    const Point2D leftMotor =
    {
        0.0f,
        0.0f
    };


    const Point2D rightMotor =
    {
        BASE_DISTANCE,
        0.0f
    };


    // ========================================================
    // THE IMPORTANT GEOMETRIC RELATIONSHIP
    // ========================================================
    //
    // The left second link is extended beyond the central
    // joint:
    //
    //
    //     B -------- C -------- P
    //
    //       L2       OFFSET
    //
    //
    // Therefore:
    //
    //     B → P = L2 + PEN_OFFSET
    //
    //
    // Let:
    //
    //     totalLeftLength = L2 + PEN_OFFSET
    //
    // ========================================================

    const float totalLeftLength =
        L2 + PEN_OFFSET;


    // ========================================================
    // WE NOW NEED TO FIND THE CENTRAL JOINT C
    // ========================================================
    //
    // This is the slightly non-obvious part.
    //
    // Normally, in a standard 5-bar, the central joint C
    // would simply be the requested (x,y) position.
    //
    // But in our robot:
    //
    //             P = requested position
    //
    //             C = actual 5-bar central joint
    //
    // So C is somewhere between B and P.
    //
    //
    // Because:
    //
    //     BC = L2
    //
    // and:
    //
    //     BP = L2 + PEN_OFFSET
    //
    //
    // the fraction from B to P at which C occurs is:
    //
    //
    //                 L2
    //     lambda = -----------
    //                L2 + E
    //
    //
    // where E = PEN_OFFSET.
    //
    // ========================================================

    const float lambda =
        L2 / totalLeftLength;


    // ========================================================
    // CIRCLE #1
    // ========================================================
    //
    // We can mathematically transform the left-side constraint
    // into a circle for C.
    //
    // The resulting circle has:
    //
    //
    //       center = lambda * P
    //
    //
    // because the left motor is at (0,0).
    //
    //
    // Its radius is:
    //
    //
    //       L1 * PEN_OFFSET
    //       -----------------
    //       L2 + PEN_OFFSET
    //
    //
    // This is the mathematical consequence of requiring:
    //
    //     A → B = L1
    //
    // while:
    //
    //     B → C = L2
    //
    //     C → P = PEN_OFFSET
    //
    // ========================================================

    Point2D circle1Center;

    circle1Center.x =
        lambda * penX;

    circle1Center.y =
        lambda * penY;


    const float circle1Radius =
        L1 * PEN_OFFSET / totalLeftLength;


    // ========================================================
    // CIRCLE #2
    // ========================================================
    //
    // The central joint C is also connected to the right
    // passive joint E by a link of length L2.
    //
    // The right passive joint E is connected to the right
    // motor by L1.
    //
    //
    // Therefore C must lie at a distance that can be reached
    // from the right motor through those two links.
    //
    // However, because E is a passive joint, C is not simply
    // constrained to a circle of radius L2 around the motor.
    //
    // Instead, for a fixed right motor and two links L1/L2,
    // C can lie anywhere in the normal two-link workspace.
    //
    // Therefore we need a different formulation.
    //
    // --------------------------------------------------------
    //
    // A cleaner approach is to use the fact that C itself is
    // the endpoint of a standard two-link arm on the right.
    //
    // The right arm can reach C whenever:
    //
    //     |L1 - L2| <= distance(rightMotor,C)
    //                         <= L1 + L2
    //
    // So rather than intersecting two circles directly, we
    // first solve the left-side geometry for possible C
    // positions, then check whether the right arm can reach C.
    //
    // ========================================================


    // ========================================================
    // PARAMETRIC SOLUTION
    // ========================================================
    //
    // We need one more geometric relationship.
    //
    // Because C lies on the line between B and P, we can
    // parameterize the unknown direction from B to P.
    //
    // Let that direction be:
    //
    //     u = (cos(theta), sin(theta))
    //
    //
    // Then:
    //
    //     B = P - (L2 + E) * u
    //
    // and:
    //
    //     C = P - E * u
    //
    //
    // B must be exactly L1 away from the left motor:
    //
    //
    //     |B| = L1
    //
    //
    // Therefore:
    //
    //     |P - (L2 + E)u| = L1
    //
    //
    // This gives us the possible directions u.
    //
    // ========================================================


    // --------------------------------------------------------
    // Distance from the LEFT motor to the PEN.
    // --------------------------------------------------------

    const float penDistance =
        sqrt(
            penX * penX +
            penY * penY
        );


    // ========================================================
    // REACHABILITY CHECK FOR THE LEFT SIDE
    // ========================================================
    //
    // We need to determine whether a circle of radius L1
    // around the left motor intersects a circle of radius
    // (L2 + E) around the pen.
    //
    //
    // Those two circles must intersect.
    //
    // Therefore:
    //
    //     |(L2+E) - L1| <= penDistance
    //
    // and:
    //
    //     penDistance <= (L2+E) + L1
    //
    // ========================================================

    if (penDistance >
        L1 + totalLeftLength + EPSILON)
    {
        // Pen is too far away.
        return result;
    }


    if (penDistance <
        fabs(totalLeftLength - L1) - EPSILON)
    {
        // Pen is too close for the required geometry.
        return result;
    }
    

    // ========================================================
    // SPECIAL CASE
    // ========================================================
    //
    // If the pen is exactly at the left motor, atan2() would
    // have no meaningful direction.
    //
    // ========================================================

    if (penDistance < EPSILON)
    {
        return result;
    }


    // ========================================================
    // FIND THE ANGLE FROM LEFT MOTOR TO PEN
    // ========================================================
    //
    // This gives us the direction of the vector:
    //
    //     LEFT MOTOR → PEN
    //
    // ========================================================

    const float penDirection =
        atan2(
            penY,
            penX
        );


    // ========================================================
    // USE THE COSINE RULE
    // ========================================================
    //
    // We now have a triangle:
    //
    //
    //                 P
    //                / \
    //               /   \
    //      L1+E+L2 /     \ penDistance
    //             /       \
    //            /         \
    //           A-----------B
    //
    //
    // More specifically, B is the left passive joint.
    //
    // The sides are:
    //
    //     A → B = L1
    //
    //     B → P = L2 + E
    //
    //     A → P = penDistance
    //
    //
    // We need the angle between:
    //
    //     A → P
    //
    // and:
    //
    //     A → B
    //
    //
    // Using the cosine rule:
    //
    //
    //       (A→B)^2 + (A→P)^2 - (B→P)^2
    // cos = --------------------------------
    //              2(A→B)(A→P)
    //
    // ========================================================

    float cosLeftOffset =
    (
        L1 * L1 +
        penDistance * penDistance -
        totalLeftLength * totalLeftLength
    )
    /
    (
        2.0f *
        L1 *
        penDistance
    );


    // ========================================================
    // FLOATING-POINT SAFETY
    // ========================================================
    //
    // Mathematically cosLeftOffset must be between -1 and 1.
    //
    // Due to floating-point rounding we might get:
    //
    //     1.0000001
    //
    // instead of:
    //
    //     1.0
    //
    // That would make acos() return NaN.
    //
    // Therefore clamp it.
    //
    // ========================================================

    if (cosLeftOffset > 1.0f)
        cosLeftOffset = 1.0f;

    if (cosLeftOffset < -1.0f)
        cosLeftOffset = -1.0f;


    // ========================================================
    // ANGLE BETWEEN:
    //
    //     LEFT MOTOR → PEN
    //
    // and:
    //
    //     LEFT MOTOR → LEFT ELBOW
    //
    // ========================================================

    const float leftOffset =
        acos(cosLeftOffset);


    // ========================================================
    // ASSEMBLY BRANCH
    // ========================================================
    //
    // There are normally two possible positions for the
    // left elbow B:
    //
    //
    //                    B1
    //                   ●
    //                  /
    //                 /
    // A ●-------------● P
    //                 \
    //                  \
    //                   ●
    //                    B2
    //
    //
    // One is the "above" solution and the other is the
    // "below" solution.
    //
    // For your drawing robot, you will select whichever
    // corresponds to the physical configuration you build.
    //
    // +1 = one assembly configuration
    // -1 = the mirrored configuration
    //
    // ========================================================

    constexpr float ELBOW_SIGN = +1.0f;


    // ========================================================
    // LEFT MOTOR ANGLE
    // ========================================================
    //
    // The motor's first link points from:
    //
    //     LEFT MOTOR → LEFT ELBOW
    //
    //
    // Its direction is:
    //
    //     penDirection + leftOffset
    //
    // or:
    //
    //     penDirection - leftOffset
    //
    // depending on the assembly branch.
    //
    // ========================================================

    result.leftAngle =
        penDirection +
        ELBOW_SIGN * leftOffset;


    // ========================================================
    // CALCULATE LEFT ELBOW POSITION
    // ========================================================
    //
    // Once we know the left motor angle:
    //
    //     Bx = L1 * cos(theta)
    //     By = L1 * sin(theta)
    //
    // ========================================================

    result.leftElbow.x =
        L1 * cos(result.leftAngle);

    result.leftElbow.y =
        L1 * sin(result.leftAngle);


    // ========================================================
    // CALCULATE CENTRAL JOINT C
    // ========================================================
    //
    // The line B → P is the extended second link.
    //
    // Its total length is:
    //
    //     L2 + PEN_OFFSET
    //
    //
    // We need to travel from B toward P by L2 to reach C.
    //
    // First calculate the unit vector:
    //
    //     B → P
    //
    // ========================================================

    float BPx =
        penX - result.leftElbow.x;

    float BPy =
        penY - result.leftElbow.y;


    // Normalize B → P.

    float BPdistance =
        sqrt(
            BPx * BPx +
            BPy * BPy
        );


    if (BPdistance < EPSILON)
    {
        return result;
    }


    float BPunitX =
        BPx / BPdistance;

    float BPunitY =
        BPy / BPdistance;


    // ========================================================
    // MOVE L2 MILLIMETRES FROM B TOWARD P
    // ========================================================
    //
    // This gives the central revolute joint C.
    //
    // ========================================================

    result.center.x =
        result.leftElbow.x +
        BPunitX * L2;

    result.center.y =
        result.leftElbow.y +
        BPunitY * L2;


    // ========================================================
    // NOW SOLVE THE RIGHT ARM
    // ========================================================
    //
    // The right arm is now a normal 2-link planar arm:
    //
    //
    //                    C
    //                   ●
    //                  / \
    //               L2/   \ L2
    //                /     \
    //               ●       ●
    //               E       ?
    //                \
    //                 \ L1
    //                  \
    //                   ●
    //                   D
    //
    //
    // D = right motor
    // E = right elbow
    //
    // C = known from the previous calculation.
    //
    // ========================================================


    // Vector:
    //
    //     RIGHT MOTOR → C
    //
    float dx =
        result.center.x -
        rightMotor.x;

    float dy =
        result.center.y -
        rightMotor.y;


    // Distance:
    //
    //     RIGHT MOTOR → C
    //
    float rightDistance =
        sqrt(
            dx * dx +
            dy * dy
        );


    // ========================================================
    // RIGHT ARM REACHABILITY
    // ========================================================
    //
    // A two-link arm with lengths L1 and L2 can reach a
    // distance d only if:
    //
    //
    //     |L1-L2| <= d <= L1+L2
    //
    // ========================================================

    if (rightDistance >
        L1 + L2 + EPSILON)
    {
        return result;
    }


    if (rightDistance <
        fabs(L1 - L2) - EPSILON)
    {
        return result;
    }


    if (rightDistance < EPSILON)
    {
        return result;
    }


    // ========================================================
    // ANGLE FROM RIGHT MOTOR TO CENTRAL JOINT
    // ========================================================

    float rightDirection =
        atan2(
            dy,
            dx
        );


    // ========================================================
    // COSINE RULE FOR RIGHT ARM
    // ========================================================
    //
    // Triangle:
    //
    //
    //                    C
    //                   ●
    //                  / \
    //                 /   \
    //                /     \
    //               /       \
    //              ●---------●
    //              D
    //
    //
    // Sides:
    //
    //     D → E = L1
    //     E → C = L2
    //     D → C = rightDistance
    //
    //
    // We calculate the angle between:
    //
    //     D → C
    //
    // and:
    //
    //     D → E
    //
    // ========================================================

    float cosRightOffset =
    (
        L1 * L1 +
        rightDistance * rightDistance -
        L2 * L2
    )
    /
    (
        2.0f *
        L1 *
        rightDistance
    );


    // Clamp floating-point errors.

    if (cosRightOffset > 1.0f)
        cosRightOffset = 1.0f;

    if (cosRightOffset < -1.0f)
        cosRightOffset = -1.0f;


    float rightOffset =
        acos(cosRightOffset);


    // ========================================================
    // RIGHT ELBOW ASSEMBLY BRANCH
    // ========================================================
    //
    // Just like the left arm, the right arm has two possible
    // configurations.
    //
    // We normally want the configuration where the right elbow
    // is on the appropriate side of the arm.
    //
    // ========================================================

    constexpr float RIGHT_ELBOW_SIGN = -1.0f;


    // ========================================================
    // RIGHT MOTOR ANGLE
    // ========================================================
    //
    // Depending on which assembly branch we select:
    //
    //     theta = direction + offset
    //
    // or:
    //
    //     theta = direction - offset
    //
    // ========================================================

    result.rightAngle =
        rightDirection +
        RIGHT_ELBOW_SIGN * rightOffset;


    // ========================================================
    // CALCULATE RIGHT ELBOW POSITION
    // ========================================================

    result.rightElbow.x =
        rightMotor.x +
        L1 * cos(result.rightAngle);

    result.rightElbow.y =
        rightMotor.y +
        L1 * sin(result.rightAngle);


    // ========================================================
    // FINAL GEOMETRY VALIDATION
    // ========================================================
    //
    // At this point we can verify that:
    //
    //     leftElbow → center = L2
    //
    //     rightElbow → center = L2
    //
    //     center → pen = PEN_OFFSET
    //
    //
    // This catches mistakes in branch selection or numerical
    // calculations.
    //
    // ========================================================

    float leftSecondLength =
        sqrt(
            pow(
                result.center.x -
                result.leftElbow.x,
                2
            )
            +
            pow(
                result.center.y -
                result.leftElbow.y,
                2
            )
        );


    float rightSecondLength =
        sqrt(
            pow(
                result.center.x -
                result.rightElbow.x,
                2
            )
            +
            pow(
                result.center.y -
                result.rightElbow.y,
                2
            )
        );


    float penLength =
        sqrt(
            pow(
                penX -
                result.center.x,
                2
            )
            +
            pow(
                penY -
                result.center.y,
                2
            )
        );


    // Check all three expected lengths.

    if (fabs(leftSecondLength - L2) > 0.01f)
        return result;

    if (fabs(rightSecondLength - L2) > 0.01f)
        return result;

    if (fabs(penLength - PEN_OFFSET) > 0.01f)
        return result;


    // ========================================================
    // EVERYTHING IS VALID
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

    const float halfDistance = elbowDistance / 2.0f;
    float heightSquared = L2 * L2 - halfDistance * halfDistance;
    if (heightSquared < 0.0f)
    {
        heightSquared = 0.0f;
    }

    const float height = sqrt(heightSquared);
    const float midpointX = (leftElbow.x + rightElbow.x) / 2.0f;
    const float midpointY = (leftElbow.y + rightElbow.y) / 2.0f;
    const float perpendicularX = -dy / elbowDistance;
    const float perpendicularY = dx / elbowDistance;

    const Point2D centers[] =
    {
        {
            midpointX + height * perpendicularX,
            midpointY + height * perpendicularY
        },
        {
            midpointX - height * perpendicularX,
            midpointY - height * perpendicularY
        }
    };

    bool found = false;
    float bestAngleError = 0.0f;

    for (uint8_t i = 0; i < 2; ++i)
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

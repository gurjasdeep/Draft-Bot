#include "kinematics1.h"
#include <math.h>




IKResult inverseKinematics(float penX, float penY)
{
    IKResult result{};

    result.valid = false;



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


   

    const float totalLeftLength =
        L2 + PEN_OFFSET;



    const float lambda =L2 / totalLeftLength;

    Point2D circle1Center;

    circle1Center.x =lambda * penX;

    circle1Center.y =lambda * penY;


    const float circle1Radius =L1 * PEN_OFFSET / totalLeftLength;



    const float penDistance =sqrt(penX * penX +penY * penY);


    if (penDistance >L1 + totalLeftLength + EPSILON)
    {
        return result;
    }


    if (penDistance < fabs(totalLeftLength - L1) - EPSILON)
    {
        return result;
    }


   
    if (penDistance < EPSILON)
    {
        return result;
    }


    const float penDirection =atan2(penY,penX);



    float cosLeftOffset =(L1 * L1 +penDistance * penDistance -totalLeftLength * totalLeftLength)/( 2.0f *L1 *penDistance);


    if (cosLeftOffset > 1.0f)
        cosLeftOffset = 1.0f;

    if (cosLeftOffset < -1.0f)
        cosLeftOffset = -1.0f;

    const float leftOffset =acos(cosLeftOffset);


    constexpr float ELBOW_SIGN = +1.0f;



    result.leftAngle =penDirection + ELBOW_SIGN * leftOffset;


    

    result.leftElbow.x =L1 * cos(result.leftAngle);

    result.leftElbow.y =L1 * sin(result.leftAngle);


    float BPx =penX - result.leftElbow.x;

    float BPy =penY - result.leftElbow.y;


    // Normalize B → P.

    float BPdistance =sqrt(BPx * BPx +BPy * BPy);


    if (BPdistance < EPSILON)
    {
        return result;
    }


    float BPunitX = BPx / BPdistance;
    float BPunitY = BPy / BPdistance;
   

    result.center.x =result.leftElbow.x + BPunitX * L2;

    result.center.y =result.leftElbow.y +BPunitY * L2;



    float dx =result.center.x -rightMotor.x;

    float dy =result.center.y - rightMotor.y;


  
    float rightDistance =sqrt(dx * dx +dy * dy);



    if (rightDistance > L1 + L2 + EPSILON)
    {
        return result;
    }


    if (rightDistance < fabs(L1 - L2) - EPSILON)
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

    float rightDirection =atan2(dy,dx);
    float cosRightOffset =(L1 * L1 +rightDistance * rightDistance -L2 * L2)/(2.0f *L1 *rightDistance);


    // Clamp floating-point errors.

    if (cosRightOffset > 1.0f)
        cosRightOffset = 1.0f;

    if (cosRightOffset < -1.0f)
        cosRightOffset = -1.0f;


    float rightOffset =cos(cosRightOffset);
     constexpr float RIGHT_ELBOW_SIGN = -1.0f;


    // ========================================================
    // RIGHT MOTOR ANGLE
    // ========================================================
    result.rightAngle =rightDirection +RIGHT_ELBOW_SIGN * rightOffset;
    // ========================================================
    // CALCULATE RIGHT ELBOW POSITION
    // ========================================================

    result.rightElbow.x =rightMotor.x +L1 * cos(result.rightAngle);
    result.rightElbow.y =rightMotor.y + L1 * sin(result.rightAngle);

    float leftSecondLength =sqrt(pow(result.center.x -result.leftElbow.x,2)+ pow(result.center.y -result.leftElbow.y, 2));
    float rightSecondLength =sqrt(pow(result.center.x -result.rightElbow.x,2)+pow(result.center.y - result.rightElbow.y,2));
    float penLength = sqrt(pow(penX -result.center.x,2 )+pow(penY -result.center.y,2));

    if (fabs(leftSecondLength - L2) > 0.01f)
        return result;

    if (fabs(rightSecondLength - L2) > 0.01f)
        return result;

    if (fabs(penLength - PEN_OFFSET) > 0.01f)
        return result;


   
    result.valid = true;

    return result;
}
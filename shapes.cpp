#include "shapes.h"
#include <math.h>



#ifndef PI
#define PI 3.14159265358979323846f
#endif


// ============================================================
// generateSquare()
// ============================================================

uint16_t generateSquare(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float size
)
{
    // A square needs 5 points:
    //
    // 4 corners
    // +
    // first point repeated to close the shape
    //
    if (buffer == nullptr || maxPoints < 5)
    {
        return 0;
    }


    // Half the side length.
    //
    // We use this because the square is centered at
    // (centerX, centerY).
    //
    // Example:
    //
    // size = 100
    //
    // halfSize = 50
    //
    // Therefore the square extends:
    //
    // centerX - 50  →  centerX + 50
    //
    // and similarly for Y.

    const float halfSize = size / 2.0f;


    // Top-left

    buffer[0] =
    {
        centerX - halfSize,
        centerY + halfSize
    };


    // Top-right

    buffer[1] =
    {
        centerX + halfSize,
        centerY + halfSize
    };


    // Bottom-right

    buffer[2] =
    {
        centerX + halfSize,
        centerY - halfSize
    };


    // Bottom-left

    buffer[3] =
    {
        centerX - halfSize,
        centerY - halfSize
    };


    // Return to the starting point.
    //
    // This closes the square.

    buffer[4] = buffer[0];


    return 5;
}


// ============================================================
// generateTriangle()
// ============================================================

uint16_t generateTriangle(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float size
)
{
    // Three vertices + first vertex repeated.

    if (buffer == nullptr || maxPoints < 4)
    {
        return 0;
    }


    // ========================================================
    // EQUILATERAL TRIANGLE GEOMETRY
    // ========================================================
    //
    // For an equilateral triangle:
    //
    // height = sqrt(3) / 2 * side
    //
    // We place the triangle with one vertex pointing upward.
    //
    //                 A
    //                 ●
    //                / \
    //               /   \
    //              /     \
    //             ●-------●
    //             B       C
    //
    // ========================================================

    const float height =
        sqrt(3.0f) * size / 2.0f;


    // We want the geometric centre of the triangle to be
    // approximately at (centerX, centerY).
    //
    // For an equilateral triangle whose base is horizontal:
    //
    // centroid Y = (topY + bottomY + bottomY) / 3
    //
    // This gives the following convenient coordinates.

    const float topY =
        centerY + (2.0f / 3.0f) * height;

    const float bottomY =
        centerY - (1.0f / 3.0f) * height;


    // Top vertex.

    buffer[0] =
    {
        centerX,
        topY
    };


    // Bottom-left vertex.

    buffer[1] =
    {
        centerX - size / 2.0f,
        bottomY
    };


    // Bottom-right vertex.

    buffer[2] =
    {
        centerX + size / 2.0f,
        bottomY
    };


    // Return to the top.

    buffer[3] = buffer[0];


    return 4;
}


// ============================================================
// generatePentagon()
// ============================================================

uint16_t generatePentagon(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float size
)
{
    if (buffer == nullptr || maxPoints < 6)
    {
        return 0;
    }

    if (size <= 0.0f)
    {
        return 0;
    }

    const float radius =
        size / (2.0f * sinf(PI / 5.0f));

    for (uint16_t i = 0; i < 5; i++)
    {
        const float angle =
            -PI / 2.0f + (static_cast<float>(i) * 2.0f * PI / 5.0f);

        buffer[i] =
        {
            centerX + radius * cosf(angle),
            centerY + radius * sinf(angle)
        };
    }

    buffer[5] = buffer[0];
    return 6;
}


// ============================================================
// generateCircle()
// ============================================================

uint16_t generateCircle(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float radius,
    uint16_t segments
)
{
    // We need one extra point because we repeat the first
    // point at the end to close the circle.

    if (buffer == nullptr)
    {
        return 0;
    }


    if (segments < 3)
    {
        return 0;
    }


    if (maxPoints < segments + 1)
    {
        return 0;
    }


    // ========================================================
    // GENERATE THE CIRCLE
    // ========================================================
    //
    // Parametric equation of a circle:
    //
    //     x = centerX + radius * cos(theta)
    //
    //     y = centerY + radius * sin(theta)
    //
    //
    // theta goes from:
    //
    //     0 → 2π
    //
    // ========================================================

    for (uint16_t i = 0; i < segments; i++)
    {
        // Convert the current segment number into an angle.
        //
        // Example with 4 segments:
        //
        // i = 0 → 0°
        // i = 1 → 90°
        // i = 2 → 180°
        // i = 3 → 270°
        //
        // The final point at 360° is added separately because
        // it is identical to the first point.

        const float theta =
            (2.0f * PI * i) / segments;


        buffer[i].x =
            centerX +
            radius * cos(theta);


        buffer[i].y =
            centerY +
            radius * sin(theta);
    }


    // Close the circle by returning to the first point.

    buffer[segments] = buffer[0];


    return segments + 1;
}

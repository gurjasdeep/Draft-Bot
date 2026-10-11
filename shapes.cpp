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
    float height
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
    // For an equilateral triangle, base width = 2 * height / sqrt(3).
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

    const float baseWidth =
        2.0f * height / sqrt(3.0f);


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
        centerX - baseWidth / 2.0f,
        bottomY
    };


    // Bottom-right vertex.

    buffer[2] =
    {
        centerX + baseWidth / 2.0f,
        bottomY
    };


    // Return to the top.

    buffer[3] = buffer[0];


    return 4;
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

uint16_t generateRobo(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float letterHeight,
    bool* penUpBefore
)
{
    // Each letter is a pen stroke in a simple single-line font.
    // Coordinates are normalized to a 0.6-by-1.0 letter cell.
    static const int8_t letterPoints[][2] =
    {
        // R
        {0, 0}, {0, 100}, {60, 100}, {60, 55}, {0, 55}, {60, 0},
        // O
        {0, 50}, {0, 100}, {60, 100}, {60, 0}, {0, 0}, {0, 50},
        // B
        {0, 0}, {0, 100}, {40, 100}, {60, 85}, {60, 65}, {40, 50},
        {0, 50}, {40, 50}, {60, 35}, {60, 15}, {40, 0}, {0, 0},
        // O
        {0, 50}, {0, 100}, {60, 100}, {60, 0}, {0, 0}, {0, 50},
        // C
        {60, 100}, {0, 100}, {0, 0}, {60, 0},
        // O
        {0, 50}, {0, 100}, {60, 100}, {60, 0}, {0, 0}, {0, 50},
        // N
        {0, 0}, {0, 100}, {60, 0}, {60, 100}
    };
    constexpr uint16_t POINT_COUNT =
        sizeof(letterPoints) / sizeof(letterPoints[0]);
    constexpr uint16_t LETTER_STARTS[] = {0, 6, 12, 24, 30, 34, 40};
    constexpr float LETTER_WIDTH = 0.6f;
    constexpr float LETTER_ADVANCE = 0.8f;
    constexpr uint16_t LETTER_COUNT =
        sizeof(LETTER_STARTS) / sizeof(LETTER_STARTS[0]);
    const float textWidth =
        ((LETTER_COUNT - 1) * LETTER_ADVANCE + LETTER_WIDTH) * letterHeight;
    const float startX = centerX - textWidth / 2.0f;
    const float baselineY = centerY - letterHeight / 2.0f;

    if (buffer == nullptr || penUpBefore == nullptr ||
        maxPoints < POINT_COUNT)
    {
        return 0;
    }

    uint16_t letterIndex = 0;
    for (uint16_t i = 0; i < POINT_COUNT; ++i)
    {
        if (letterIndex + 1 < LETTER_COUNT &&
            i == LETTER_STARTS[letterIndex + 1])
        {
            ++letterIndex;
        }

        const uint16_t pointInLetter = i - LETTER_STARTS[letterIndex];
        buffer[i] =
        {
            startX +
                (letterIndex * LETTER_ADVANCE +
                 (letterPoints[i][0] / 100.0f) * LETTER_WIDTH) *
                    letterHeight,
            baselineY + (letterPoints[i][1] / 100.0f) * letterHeight
        };
        penUpBefore[i] = pointInLetter == 0 && letterIndex != 0;
    }

    return POINT_COUNT;
}

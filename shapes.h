#pragma once

#include <stdint.h>

// ============================================================
// A single point in the robot's drawing coordinate system.
//
// Units: millimetres
//
// x = horizontal position
// y = vertical position
// ============================================================

struct PathPoint
{
    float x;
    float y;
};


// ============================================================
// Generate a square.
//
// The square is centered around (centerX, centerY).
//
// size = length of one side in millimetres.
//
// The final point is the same as the first point so that the
// square is completely closed.
//
// Example:
//
//          (x0,y0) -------- (x1,y0)
//             |                 |
//             |                 |
//             |                 |
//          (x0,y1) -------- (x1,y1)
//
// Returns:
//     Number of points written into the buffer.
//
// Returns 0 if the buffer is too small.
// ============================================================

uint16_t generateSquare(PathPoint* buffer,uint16_t maxPoints,float centerX,float centerY,float size);


// ============================================================
// Generate an equilateral triangle.
//
// The triangle is centered approximately around
// (centerX, centerY).
//
// size = side length in millimetres.
//
// The final point is the same as the first point.
//
// Returns:
//     Number of points written.
//
// Returns 0 if the buffer is too small.
// ============================================================

uint16_t generateTriangle(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float size
);


// ============================================================
// Generate a circle.
//
// centerX / centerY = circle centre
// radius             = circle radius
// segments           = number of straight-line points used
//
// More segments = smoother circle.
//
// For example:
//
//     radius = 30 mm
//     segments = 100
//
// produces a 30 mm radius circle using 100 segments.
//
// The final point is the same as the first point.
//
// Returns:
//     Number of points written.
//
// Returns 0 if the buffer is too small or segments is invalid.
// ============================================================

uint16_t generateCircle(
    PathPoint* buffer,
    uint16_t maxPoints,
    float centerX,
    float centerY,
    float radius,
    uint16_t segments
);

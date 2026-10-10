#include <Arduino.h>
#include <Servo.h>
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kinematics.h"
#include "pins.h"
#include "shapes.h"
#include "stepper.h"


// ============================================================
// SERIAL COMMAND CONFIGURATION
// ============================================================

// Maximum length of one command.
//
// Examples:
//
// SHAPE CIRCLE
// MOVE 50.5 30.2
// STEP L +100
// HOME
// HOME TEST
// PEN DOWN
//
// 64 bytes is more than enough for these commands.
constexpr uint16_t COMMAND_BUFFER_SIZE = 64;
constexpr uint16_t CIRCLE_SEGMENTS = 36;
constexpr uint16_t MAX_SHAPE_POINTS = CIRCLE_SEGMENTS + 1;

constexpr uint16_t MOTOR_STEPS_PER_REVOLUTION = 200;
constexpr uint8_t MOTOR_MICROSTEPS = 8;

constexpr float DEFAULT_SHAPE_CENTER_X = 21.2f;
constexpr float DEFAULT_SHAPE_CENTER_Y = 200.0f;
constexpr float DEFAULT_SHAPE_SIZE = 50.0f;
constexpr float DEFAULT_CIRCLE_RADIUS = DEFAULT_SHAPE_SIZE / 2.0f;
constexpr float MAX_CARTESIAN_SEGMENT_MM = 1.0f;

constexpr float LEFT_HOME_ANGLE_DEGREES = 180.0f;
constexpr float RIGHT_HOME_ANGLE_DEGREES = 0.0f;
constexpr LimitPolarity RIGHT_LIMIT_POLARITY = LimitPolarity::ACTIVE_LOW;
constexpr uint8_t PEN_UP_ANGLE = 90;
constexpr uint8_t PEN_DOWN_ANGLE = 0;
constexpr unsigned long PEN_SETTLE_TIME_MS = 400;


// Buffer in which we accumulate incoming characters.
char commandBuffer[COMMAND_BUFFER_SIZE];

// Number of characters currently stored in the buffer.
uint16_t commandLength = 0;


// ============================================================
// ROBOT STATE
// ============================================================

enum class PenState
{
    UP,
    DOWN
};

PenState penState = PenState::UP;
bool robotHomed = false;

Stepper leftMotor(
    LEFT_PULSE_PIN,
    LEFT_DIR_PIN,
    LEFT_ENABLE_PIN,
    MOTOR_STEPS_PER_REVOLUTION,
    MOTOR_MICROSTEPS,
    LEFT_LIMIT_PIN
);

Stepper rightMotor(
    RIGHT_PULSE_PIN,
    RIGHT_DIR_PIN,
    RIGHT_ENABLE_PIN,
    MOTOR_STEPS_PER_REVOLUTION,
    MOTOR_MICROSTEPS,
    RIGHT_LIMIT_PIN
);

Servo penServo;

enum class MotionType
{
    NONE,
    MOVE,
    STEP,
    SHAPE
};

enum class ShapeMotionPhase
{
    LIFTING_FOR_TRAVEL,
    MOVING_TO_START,
    LOWERING_PEN,
    DRAWING,
    RAISING_PEN,
    RETURNING_HOME
};

MotionType activeMotion = MotionType::NONE;
ShapeMotionPhase shapeMotionPhase = ShapeMotionPhase::MOVING_TO_START;
PathPoint shapePath[MAX_SHAPE_POINTS];
uint16_t shapePointCount = 0;
uint16_t shapePointIndex = 0;
const char* activeShapeName = nullptr;
char activeStepMotor = '\0';
unsigned long shapePhaseStartedAt = 0;
Point2D cartesianPathStart = {};
Point2D cartesianPathTarget = {};
uint16_t cartesianSegmentCount = 0;
uint16_t cartesianSegmentIndex = 0;
bool cartesianPathActive = false;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void processSerial();
void processCommand(char* command);
bool parseMoveCoordinate(const char*& cursor, float& coordinate);

void handleShape(const char* shapeName);
void handleMove(float x, float y);
bool parseStepCommand(char* command, char& motorName, long& steps);
void handleStep(char motorName, long steps);
void handlePen(const char* state);
void updateMotion();
bool setMotorTarget(float x, float y);
void setMotorAngles(const IKResult& result);
void startNextCartesianSegment();
void setPen(PenState state);
void finishShape();

void sendOK();
void sendError(const char* message);


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Give the PC a little time to establish the serial
    // connection. This is not strictly necessary for every
    // board, but is useful during USB serial development.
    delay(100);

    leftMotor.begin();
    rightMotor.begin(RIGHT_LIMIT_POLARITY);

    penServo.attach(SERVO_PIN);
    setPen(PenState::UP);

    Serial.println("READY");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // Check whether the PC has sent us anything.
    //
    // This function does NOT block waiting for serial data.
    //
    // Keep processing serial input while motor motion advances.
    processSerial();
    updateMotion();
}


// ============================================================
// PROCESS SERIAL DATA
// ============================================================
//
// The PC sends commands such as:
//
//     SHAPE CIRCLE\n
//     MOVE 50 30\n
//     PEN DOWN\n
//
// We read one character at a time.
//
// Everything before '\n' is considered one command.
//
// ============================================================

void processSerial()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read();


        // ----------------------------------------------------
        // NEWLINE = END OF COMMAND
        // ----------------------------------------------------

        if (c == '\n')
        {
            Serial.println();
            // Terminate the C string.
            //
            // Example:
            //
            // "MOVE 50 30"
            //
            // becomes:
            //
            // ['M']['O']['V']['E'][' ']['5']['0']...
            //                                  ['\0']
            //
            commandBuffer[commandLength] = '\0';


            // Ignore an empty line.
            if (commandLength > 0)
            {
                processCommand(commandBuffer);
            }


            // Prepare the buffer for the next command.
            commandLength = 0;
        }


        // ----------------------------------------------------
        // CARRIAGE RETURN
        // ----------------------------------------------------
        //
        // Some computers send:
        //
        //     "\r\n"
        //
        // instead of simply:
        //
        //     "\n"
        //
        // We don't want '\r' to become part of the command.
        //
        else if (c == '\r')
        {
            continue;
        }


        // ----------------------------------------------------
        // NORMAL CHARACTER
        // ----------------------------------------------------

        else
        {
            Serial.print(c);
            // Only add the character if there is still room.
            //
            // We reserve one byte for '\0'.
            //
            if (commandLength < COMMAND_BUFFER_SIZE - 1)
            {
                commandBuffer[commandLength] = c;
                commandLength++;
            }
            else
            {
                // Command is too long.
                //
                // Discard the current command rather than
                // overflowing the buffer.
                commandLength = 0;

                sendError("COMMAND_TOO_LONG");
            }
        }
    }
}


// ============================================================
// PROCESS ONE COMPLETE COMMAND
// ============================================================
//
// Examples:
//
//     SHAPE CIRCLE
//     MOVE 50 30
//     PEN DOWN
//
// ============================================================

void processCommand(char* command)
{
    char commandType[16];


    // --------------------------------------------------------
    // Extract the first word.
    //
    // Example:
    //
    // "MOVE 50 30"
    //
    // gives:
    //
    // commandType = "MOVE"
    // --------------------------------------------------------

    int parsed =
        sscanf(
            command,
            "%15s",
            commandType
        );


    if (parsed != 1)
    {
        sendError("INVALID_COMMAND");
        return;
    }


    // ========================================================
    // SHAPE
    // ========================================================
    //
    // Expected:
    //
    //     SHAPE SQUARE
    //     SHAPE TRIANGLE
    //     SHAPE CIRCLE
    //
    // ========================================================

    if (strcmp(commandType, "SHAPE") == 0)
    {
        char shapeName[20];


        if (sscanf(
                command,
                "%15s %19s",
                commandType,
                shapeName
            ) != 2)
        {
            sendError("INVALID_SHAPE_COMMAND");
            return;
        }


        handleShape(shapeName);

        return;
    }


    // ========================================================
    // MOVE
    // ========================================================
    //
    // Expected:
    //
    //     MOVE 50 30
    //     MOVE 50.5 30.2
    //
    // ========================================================

    if (strcmp(commandType, "MOVE") == 0)
    {
        float x;
        float y;
        const char *cursor = command;

        while (*cursor == ' ' || *cursor == '\t')
        {
            ++cursor;
        }

        cursor += strlen(commandType);


        if (!parseMoveCoordinate(cursor, x) ||
            !parseMoveCoordinate(cursor, y))
        {
            sendError("INVALID_MOVE_COMMAND");
            return;
        }

        while (*cursor == ' ' || *cursor == '\t')
        {
            ++cursor;
        }

        if (*cursor != '\0')
        {
            sendError("INVALID_MOVE_COMMAND");
            return;
        }

        handleMove(x, y);

        return;
    }

    if (strcmp(commandType, "STEP") == 0)
    {
        char motorName;
        long steps;

        if (!parseStepCommand(command, motorName, steps))
        {
            sendError("INVALID_STEP_COMMAND");
            return;
        }

        handleStep(motorName, steps);
        return;
    }

    if (strcmp(commandType, "HOME") == 0)
    {
        if (activeMotion != MotionType::NONE)
        {
            sendError("MOTION_BUSY");
            return;
        }

        char homeMode[16] = {};
        const int parsedHomeMode =
            sscanf(
                command,
                "%15s %15s",
                commandType,
                homeMode
            );

        // Allow a manual "software home" for bench testing when
        // limit switches are not installed yet.
        if (parsedHomeMode == 2 &&
            (
                strcmp(homeMode, "TEST") == 0 ||
                strcmp(homeMode, "FAKE") == 0 ||
                strcmp(homeMode, "FORCE") == 0 ||
                strcmp(homeMode, "SIMULATE") == 0
            ))
        {
            setPen(PenState::UP);
            leftMotor.setAngle(LEFT_HOME_ANGLE_DEGREES);
            rightMotor.setAngle(RIGHT_HOME_ANGLE_DEGREES);
            robotHomed = true;
            Serial.println("OK HOME");
            return;
        }

        if (parsedHomeMode != 1)
        {
            sendError("INVALID_HOME_COMMAND");
            return;
        }

        setPen(PenState::UP);
        robotHomed = false;
        const HomingResult leftHome =
            leftMotor.home(HomingDirection::HOME_LEFT);
        if (leftHome != HomingResult::SUCCESS)
        {
            sendError(
                leftHome == HomingResult::TIMEOUT
                    ? "HOME_TIMEOUT_LEFT"
                    : "HOME_LIMIT_NOT_CONFIGURED_LEFT"
            );
            return;
        }

        const HomingResult rightHome =
            rightMotor.home(
                HomingDirection::HOME_RIGHT,
                RIGHT_LIMIT_POLARITY
            );
        if (rightHome != HomingResult::SUCCESS)
        {
            sendError(
                rightHome == HomingResult::TIMEOUT
                    ? "HOME_TIMEOUT_RIGHT"
                    : "HOME_LIMIT_NOT_CONFIGURED_RIGHT"
            );
            return;
        }

        leftMotor.setAngle(LEFT_HOME_ANGLE_DEGREES);
        rightMotor.setAngle(RIGHT_HOME_ANGLE_DEGREES);
        robotHomed = true;

        Serial.println("OK HOME");
        return;
    }

    // ========================================================
    // PEN
    // ========================================================
    //
    // Expected:
    //
    //     PEN UP
    //     PEN DOWN
    //
    // ========================================================

    if (strcmp(commandType, "PEN") == 0)
    {
        char state[10];


        if (sscanf(
                command,
                "%15s %9s",
                commandType,
                state
            ) != 2)
        {
            sendError("INVALID_PEN_COMMAND");
            return;
        }

        handlePen(state);

        return;
    }


    // ========================================================
    // UNKNOWN COMMAND
    // ========================================================

    sendError("UNKNOWN_COMMAND");
}

bool parseStepCommand(char* command, char& motorName, long& steps)
{
    const char* cursor = command;

    while (*cursor == ' ' || *cursor == '\t')
    {
        ++cursor;
    }

    cursor += strlen("STEP");

    if (*cursor != ' ' && *cursor != '\t')
    {
        return false;
    }

    while (*cursor == ' ' || *cursor == '\t')
    {
        ++cursor;
    }

    if (*cursor != 'L' && *cursor != 'R')
    {
        return false;
    }

    motorName = *cursor++;

    if (*cursor != ' ' && *cursor != '\t')
    {
        return false;
    }

    while (*cursor == ' ' || *cursor == '\t')
    {
        ++cursor;
    }

    if (*cursor != '+' && *cursor != '-')
    {
        return false;
    }

    errno = 0;
    char* end;
    const long parsedSteps = strtol(cursor, &end, 10);

    if (end == cursor ||
        errno == ERANGE ||
        parsedSteps == 0 ||
        (*end != '\0' && *end != ' ' && *end != '\t'))
    {
        return false;
    }

    while (*end == ' ' || *end == '\t')
    {
        ++end;
    }

    if (*end != '\0')
    {
        return false;
    }

    steps = parsedSteps;
    return true;
}

bool parseMoveCoordinate(const char*& cursor, float& coordinate)
{
    while (*cursor == ' ' || *cursor == '\t')
    {
        ++cursor;
    }

    if (*cursor == '\0')
    {
        return false;
    }

    char* end;
    const double parsedValue = strtod(cursor, &end);

    if (end == cursor ||
        (*end != '\0' && *end != ' ' && *end != '\t') ||
        parsedValue != parsedValue ||
        parsedValue > FLT_MAX ||
        parsedValue < -FLT_MAX)
    {
        return false;
    }

    coordinate = static_cast<float>(parsedValue);
    cursor = end;
    return true;
}


// ============================================================
// HANDLE SHAPE COMMAND
// ============================================================

void handleShape(const char* shapeName)
{
    if (strcmp(shapeName, "SQUARE") != 0 &&
        strcmp(shapeName, "TRIANGLE") != 0 &&
        strcmp(shapeName, "CIRCLE") != 0)
    {
        sendError("UNKNOWN_SHAPE");
        return;
    }

    if (!robotHomed)
    {
        sendError("NOT_HOMED");
        return;
    }

    if (activeMotion != MotionType::NONE)
    {
        sendError("MOTION_BUSY");
        return;
    }

    const float centerX = DEFAULT_SHAPE_CENTER_X;
    const float centerY = DEFAULT_SHAPE_CENTER_Y;
    uint16_t generatedPoints = 0;

    if (strcmp(shapeName, "SQUARE") == 0)
    {
        generatedPoints = generateSquare(
            shapePath,
            MAX_SHAPE_POINTS,
            centerX,
            centerY,
            DEFAULT_SHAPE_SIZE
        );
        activeShapeName = "SQUARE";
    }
    else if (strcmp(shapeName, "TRIANGLE") == 0)
    {
        generatedPoints = generateTriangle(
            shapePath,
            MAX_SHAPE_POINTS,
            centerX,
            centerY,
            DEFAULT_SHAPE_SIZE
        );
        activeShapeName = "TRIANGLE";
    }
    else if (strcmp(shapeName, "CIRCLE") == 0)
    {
        generatedPoints = generateCircle(
            shapePath,
            MAX_SHAPE_POINTS,
            centerX,
            centerY,
            DEFAULT_CIRCLE_RADIUS,
            CIRCLE_SEGMENTS
        );
        activeShapeName = "CIRCLE";
    }
    if (generatedPoints == 0)
    {
        sendError("SHAPE_GENERATION_FAILED");
        activeShapeName = nullptr;
        return;
    }

    for (uint16_t i = 0; i < generatedPoints; ++i)
    {
        const IKResult result =
            inverseKinematics(shapePath[i].x, shapePath[i].y);

        if (!result.valid)
        {
            sendError("UNREACHABLE_SHAPE");
            activeShapeName = nullptr;
            return;
        }
    }

    shapePointCount = generatedPoints;
    shapePointIndex = 0;
    shapeMotionPhase = ShapeMotionPhase::LIFTING_FOR_TRAVEL;
    activeMotion = MotionType::SHAPE;
    setPen(PenState::UP);
    shapePhaseStartedAt = millis();

    Serial.print("OK SHAPE ");
    Serial.print(activeShapeName);
    Serial.println(" STARTED");
}


// ============================================================
// HANDLE MOVE COMMAND
// ============================================================

void handleMove(float x, float y)
{
    if (!robotHomed)
    {
        sendError("NOT_HOMED");
        return;
    }

    if (activeMotion != MotionType::NONE)
    {
        sendError("MOTION_BUSY");
        return;
    }

    if (!setMotorTarget(x, y))
    {
        sendError("UNREACHABLE");
        return;
    }

    activeMotion = MotionType::MOVE;
    Serial.println("OK MOVE STARTED");
}

void handleStep(char motorName, long steps)
{
    if (activeMotion != MotionType::NONE)
    {
        sendError("MOTION_BUSY");
        return;
    }

    Stepper& motor = motorName == 'L' ? leftMotor : rightMotor;
    const long currentPosition = motor.getPosition();

    if ((steps > 0 && currentPosition > LONG_MAX - steps) ||
        (steps < 0 && currentPosition < LONG_MIN - steps))
    {
        sendError("STEP_POSITION_OVERFLOW");
        return;
    }

    motor.moveTo(currentPosition + steps);
    activeStepMotor = motorName;
    activeMotion = MotionType::STEP;

    Serial.print("OK STEP ");
    Serial.print(motorName);
    Serial.println(" STARTED");
}


// ============================================================
// HANDLE PEN COMMAND
// ============================================================

void handlePen(const char* state)
{
    if (activeMotion != MotionType::NONE)
    {
        sendError("MOTION_BUSY");
        return;
    }

    if (strcmp(state, "UP") == 0)
    {
        setPen(PenState::UP);
        Serial.println("OK PEN UP");
    }
    else if (strcmp(state, "DOWN") == 0)
    {
        setPen(PenState::DOWN);
        Serial.println("OK PEN DOWN");
    }
    else
    {
        sendError("INVALID_PEN_STATE");
    }
}

bool setMotorTarget(float x, float y)
{
    const IKResult targetResult = inverseKinematics(x, y);
    if (!targetResult.valid)
    {
        return false;
    }

    const float degreesToRadians =
        3.14159265358979323846f / 180.0f;
    if (!forwardKinematics(
            leftMotor.getAngle() * degreesToRadians,
            rightMotor.getAngle() * degreesToRadians,
            cartesianPathStart
        ))
    {
        return false;
    }

    const float deltaX = x - cartesianPathStart.x;
    const float deltaY = y - cartesianPathStart.y;
    const float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    const float segmentCount = ceilf(
        distance / MAX_CARTESIAN_SEGMENT_MM
    );

    cartesianSegmentCount =
        segmentCount < 1.0f ? 1 : static_cast<uint16_t>(segmentCount);
    cartesianSegmentIndex = 0;
    cartesianPathTarget = {x, y};

    IKResult firstSegmentResult = {};
    for (uint16_t segment = 1; segment <= cartesianSegmentCount; ++segment)
    {
        const float fraction =
            static_cast<float>(segment) / cartesianSegmentCount;
        const IKResult result = inverseKinematics(
            cartesianPathStart.x + deltaX * fraction,
            cartesianPathStart.y + deltaY * fraction
        );
        if (!result.valid)
        {
            cartesianPathActive = false;
            return false;
        }

        if (segment == 1)
        {
            firstSegmentResult = result;
        }
    }

    cartesianPathActive = true;
    cartesianSegmentIndex = 1;
    setMotorAngles(firstSegmentResult);
    return true;
}

void setMotorAngles(const IKResult& result)
{
    const float degreesToRadians =
        3.14159265358979323846f / 180.0f;
    const float leftCurrentAngle = leftMotor.getAngle();
    const float rightCurrentAngle = rightMotor.getAngle();
    const float leftDelta = result.leftAngle -
        leftCurrentAngle * degreesToRadians;
    const float rightDelta = result.rightAngle -
        rightCurrentAngle * degreesToRadians;

    leftMotor.moveToAngle(
        leftCurrentAngle +
        radiansToDegrees(atan2f(sinf(leftDelta), cosf(leftDelta)))
    );
    rightMotor.moveToAngle(
        rightCurrentAngle +
        radiansToDegrees(atan2f(sinf(rightDelta), cosf(rightDelta)))
    );
}

void startNextCartesianSegment()
{
    ++cartesianSegmentIndex;
    const float fraction =
        static_cast<float>(cartesianSegmentIndex) / cartesianSegmentCount;
    const float x = cartesianPathStart.x +
        (cartesianPathTarget.x - cartesianPathStart.x) * fraction;
    const float y = cartesianPathStart.y +
        (cartesianPathTarget.y - cartesianPathStart.y) * fraction;
    const IKResult result = inverseKinematics(x, y);

    if (!result.valid)
    {
        cartesianPathActive = false;
        activeMotion = MotionType::NONE;
        setPen(PenState::UP);
        activeShapeName = nullptr;
        sendError("UNREACHABLE_PATH");
        return;
    }

    setMotorAngles(result);
}

void setPen(PenState state)
{
    penState = state;
    penServo.write(state == PenState::UP ? PEN_UP_ANGLE : PEN_DOWN_ANGLE);
}

void updateMotion()
{
    leftMotor.update();
    rightMotor.update();

    if (activeMotion == MotionType::NONE)
    {
        return;
    }

    if (leftMotor.isMoving() || rightMotor.isMoving())
    {
        return;
    }

    if (cartesianPathActive)
    {
        if (cartesianSegmentIndex < cartesianSegmentCount)
        {
            startNextCartesianSegment();
            return;
        }

        cartesianPathActive = false;
    }

    if (activeMotion == MotionType::MOVE)
    {
        activeMotion = MotionType::NONE;
        Serial.println("DONE MOVE");
        return;
    }

    if (activeMotion == MotionType::STEP)
    {
        activeMotion = MotionType::NONE;
        Serial.print("DONE STEP ");
        Serial.println(activeStepMotor);
        activeStepMotor = '\0';
        return;
    }

    if (shapeMotionPhase == ShapeMotionPhase::LIFTING_FOR_TRAVEL)
    {
        if (millis() - shapePhaseStartedAt < PEN_SETTLE_TIME_MS)
        {
            return;
        }

        shapeMotionPhase = ShapeMotionPhase::MOVING_TO_START;
        if (!setMotorTarget(shapePath[0].x, shapePath[0].y))
        {
            activeMotion = MotionType::NONE;
            activeShapeName = nullptr;
            sendError("UNREACHABLE_SHAPE");
        }
        return;
    }

    if (shapeMotionPhase == ShapeMotionPhase::RAISING_PEN)
    {
        if (millis() - shapePhaseStartedAt < PEN_SETTLE_TIME_MS)
        {
            return;
        }

        shapeMotionPhase = ShapeMotionPhase::RETURNING_HOME;
        leftMotor.moveToAngle(LEFT_HOME_ANGLE_DEGREES);
        rightMotor.moveToAngle(RIGHT_HOME_ANGLE_DEGREES);
        return;
    }

    if (shapeMotionPhase == ShapeMotionPhase::RETURNING_HOME)
    {
        activeMotion = MotionType::NONE;
        Serial.print("DONE SHAPE ");
        Serial.println(activeShapeName);
        activeShapeName = nullptr;
        return;
    }

    if (shapeMotionPhase == ShapeMotionPhase::MOVING_TO_START)
    {
        setPen(PenState::DOWN);
        shapeMotionPhase = ShapeMotionPhase::LOWERING_PEN;
        shapePhaseStartedAt = millis();
        return;
    }

    if (shapeMotionPhase == ShapeMotionPhase::LOWERING_PEN)
    {
        if (millis() - shapePhaseStartedAt < PEN_SETTLE_TIME_MS)
        {
            return;
        }

        shapeMotionPhase = ShapeMotionPhase::DRAWING;
        shapePointIndex = 1;
    }
    else
    {
        ++shapePointIndex;
    }

    if (shapePointIndex >= shapePointCount)
    {
        finishShape();
        return;
    }

    if (!setMotorTarget(
            shapePath[shapePointIndex].x,
            shapePath[shapePointIndex].y
        ))
    {
        setPen(PenState::UP);
        activeMotion = MotionType::NONE;
        activeShapeName = nullptr;
        sendError("UNREACHABLE_SHAPE");
    }
}

void finishShape()
{
    setPen(PenState::UP);
    shapeMotionPhase = ShapeMotionPhase::RAISING_PEN;
    shapePhaseStartedAt = millis();
}


// ============================================================
// SEND SUCCESS RESPONSE
// ============================================================

void sendOK()
{
    Serial.println("OK");
}


// ============================================================
// SEND ERROR RESPONSE
// ============================================================

void sendError(const char* message)
{
    Serial.print("ERROR ");
    Serial.println(message);
}
>>>>>>> gurjas

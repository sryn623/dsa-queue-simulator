// src/common/types.h
#ifndef TYPES_H
#define TYPES_H

// Lane identification
enum class LaneId {
    AL1_INCOMING, AL2_PRIORITY, AL3_FREELANE,
    BL1_INCOMING, BL2_PRIORITY, BL3_FREELANE,
    CL1_INCOMING, CL2_PRIORITY, CL3_FREELANE,
    DL1_INCOMING, DL2_PRIORITY, DL3_FREELANE
};

// Direction enum for vehicle movement
enum class Direction {
    NORTH,
    SOUTH,
    EAST,
    WEST
};

#endif // TYPES_H
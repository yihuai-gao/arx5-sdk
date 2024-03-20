#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#ifndef MINGW
// struct can_frame
#include <linux/can.h>
//#include <linux/can/raw.h>
#else
#include <stdint.h>

// Workaround for absent linux headers: Explicit struct definition
struct can_frame
{
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t __pad;
    uint8_t __res0;
    uint8_t __res1;
    uint8_t data[8] __attribute__((aligned(8)));
};
#endif

/**
 * Holds the content of one CAN frame
 * Struct delcared in <linux/can.h>
 */
typedef struct can_frame can_frame_t;

#endif

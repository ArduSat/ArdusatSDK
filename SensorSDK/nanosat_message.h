/*
*   @file nanosat_message.h
*   @version 1.0
*   @name NanoSatisfi Inc.
*
*   @section LICENSE
*
*   see LICENSE and NOTICE files.
*
*   @section DESCRIPTION
*   Message passing schema between OnboardCommLayer (I2C wrapper) and Payload
*   supervisor. This interface exposes system calls to user space and sensor
*   access. The main interaction players are Payload Supervisor(assv) and
*   experiment nodes.
*
*   @todo Put examples here.
*   @code Example
*   @endcode
*/
#ifndef NANOSAT_MESSAGE_H
#define NANOSAT_MESSAGE_H

#include <inttypes.h>

#define NODE_COMM_MAX_I2C_BUFFER_SIZE 32
#define NODE_COMM_MAX_BUFFER_SIZE 24
#define NODE_COMM_MESSAGE_PREFIX 0xAB

/******************************************************************************
 * message types
 ******************************************************************************/
enum message_type{
    APPEND,         //append to the experiment file
    READ,           //read a value from a sensor
    SET,            //set the value of sensor register
    EXIT,           //signal the end of an experiment instance
    SENSOR_REG,     //register the use of a sensor by a node
    CAM,            //camera message (to supervisor)
    CLOCK,          //clock message (to supervisor)
    SUN,            //sun message (to supervisor)
    TELEMETRY,      //telemetry request
    CONTROL_PITCH,  //change pitch
    CONTROL_YAW,    //change yaw
    CONTROL_ROLL    //change roll
};

/******************************************************************************
 * message put on wire
 ******************************************************************************/
typedef struct __attribute__((packed)) {
    uint8_t prefix;
    uint8_t type;
    uint8_t node_addr; // Address of sender
    uint8_t len;
    uint8_t buf[NODE_COMM_MAX_BUFFER_SIZE];
    uint16_t checksum;
} nanosat_message_t;

//-----------------------------------------------------------
// Encode a Fletcher checksum in the last 2 bytes of buffer
// @see http://en.wikipedia.org/wiki/Fletcher's_checksum
//-----------------------------------------------------------
//inline uint16_t fletcher_encode(uint8_t buffer[], long count );

//-------------------------------------------------------------
// Decode Fletcher Checksum. Returns zero if buffer error-free
// @see http://en.wikipedia.org/wiki/Fletcher's_checksum
//-------------------------------------------------------------
//inline long fletcher_decode(uint8_t buffer[], long count );

#endif /* NANOSAT_MESSAGE_H */

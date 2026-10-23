#pragma once

#include <stdint.h>      // Per uint8_t, uint32_t
#include <stdbool.h>     // Per il tipo bool
#include "driver/uart.h" // Per uart_read_bytes e le funzioni ESP-IDF
#include "id_type.h"


//*PAYLOADS DEFINITIONS


typedef struct{
    uint32_t millis;
}PayloadG4;

//todo just for mockup
typedef struct{
    int num1;
    float num2;
}PayloadCommand02;
//todo _just for mockup

typedef enum{
    type_StM,
    type_StM_ack,
    type_MtS,
    type_MtS_ack,
}HandshakeType;
typedef struct{
    HandshakeType handshake_type;
} PayloadHandshake;


typedef struct{ //max value: 127
    module_id_t my_slave_id;
    module_id_t my_id;
    module_id_t my_master_id;
}PayloadReport;


typedef struct{
    float radians; 
    float speed;
    float acceleration;
    float jerk;
    bool relative;
    bool send_ack;
}PayloadServo;

typedef struct{
    module_id_t sender_id;
}PayloadServoAck;

typedef struct{
    float curr_radians;
    float curr_speed;
    float curr_acc;
}PayloadCinematicUpdate;

typedef struct{ //massimo 255 messaggi
    uint8_t group_number;
} PayloadGroup;


//con le union alloca sempre i byte x il messaggio + lungo
typedef union{ 
    PayloadG4 payload_g4;
    PayloadCommand02 payload_debug; //todo just for mockup
    PayloadHandshake payload_handshake;
    PayloadReport payload_report;
    PayloadServo payload_servo;
    PayloadServoAck payload_servo_ack;
    PayloadCinematicUpdate payload_cinematic_update;
    PayloadGroup payload_group;
}Payload;


//*MSG DEFINIFION
typedef enum{
    type_g4,
    type_debug, //todo just for mockup
    type_handshake,
    type_report,
    type_servo,
    type_servo_ack,
    type_cinematic_update,
    type_group,
}MsgType;
typedef struct __attribute__((packed)){
    uint8_t header;
    module_id_t sender_id;
    module_id_t target_id;
    MsgType type;

    Payload payload;

    uint32_t footer;
}Msg;

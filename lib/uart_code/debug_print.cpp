#include <stdio.h>
#include <string.h>

//definiti da me:
#include "msg_structs.h"
#include "utils_uart_comms.h"


//* _______________________________________PRINT DEBUG INFO

const char* get_role_name(int role) {
    if (role == 0) {
        return "MASTER";
    } else if (role == 1) {
        return "SLAVE";
    } else {
        return "UNKNOWN";
    }
}




#include <stdio.h>
#include <stdio.h>

static const char* msg_type_to_str(int t){
  switch(t){
    case type_g4: return "COMMAND_G4";
    case type_debug: return "COMMAND_debug";
    case type_handshake:  return "HANDSHAKE";
    case type_report:     return "REPORT";
    case type_servo:      return "SERVO";
    case type_servo_ack:  return "SERVO_ACK";
    default:              return "UNKNOWN_TYPE";
  }
}

static const char* handshake_type_to_str(int h){
  switch(h){
    case type_StM:     return "StM";
    case type_StM_ack: return "StM_ack";
    case type_MtS:     return "MtS";
    case type_MtS_ack: return "MtS_ack";
    default:           return "UNKNOWN_HANDSHAKE";
  }
}

void print_msg_struct(Msg* msg){

  if(msg == nullptr){
    printf("PRINTING STRUCT MESSAGE: NULL\n\n");
    return;
  }

  printf("PRINTING STRUCT MESSAGE:\n");
  printf("HEAP PT: %p\n", (void*)msg);
  printf("Sender: %d\n", msg->sender_id);
  printf("Target: %d\n", msg->target_id);
  printf("Type: %s\n", msg_type_to_str(msg->type));
  printf("Header: %d\n", msg->header);
  printf("Footer: %lu\n", (unsigned long)msg->footer);

  switch(msg->type){

    case type_g4:
      printf("Payload (COMMAND_01)\n");
      printf("millis: %ld\n", msg->payload.payload_g4.millis);
      break;

    case type_debug:
      printf("Payload (COMMAND_debug)\n");
      printf("num1: %d\n", msg->payload.payload_debug.num1);
      printf("num2: %f\n", msg->payload.payload_debug.num2);
      break;

    case type_handshake:
      printf("Payload (HANDSHAKE)\n");
      printf("Handshake Type: %s\n",
             handshake_type_to_str(
               msg->payload.payload_handshake.handshake_type));
      break;

    case type_report:
      printf("Payload (REPORT)\n");
      printf("my_slave_id: %d\n", msg->payload.payload_report.my_slave_id);
      printf("my_id: %d\n", msg->payload.payload_report.my_id);
      printf("my_master_id: %d\n", msg->payload.payload_report.my_master_id);
      break;

    case type_servo:
      printf("Payload (SERVO)\n");
      printf("servo radians: %f\n", msg->payload.payload_servo.radians);
      break;

    case type_servo_ack:
      printf("Payload (SERVO)\n");
      printf("servo radians: %f\n", msg->payload.payload_servo.radians);
      break;

    default:
      printf("ERROR: unknown message type\n");
  }
  printf("__PRINTING STRUCT MESSAGE.\n");
}
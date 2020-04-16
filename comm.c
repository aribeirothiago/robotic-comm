#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define SPEED 6
#define TIME_STEP 64
#define COMMUNICATION_CHANNEL 1

typedef enum { EMITTER, RECEIVER } robot_types;

int main() {
  WbDeviceTag ps0, ps7, communication, left_motor, right_motor;
  robot_types robot_type;
  int message_printed = 0; /* used to avoid printing continuously the communication state */
  double left_speed, right_speed;

  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /*
   * as we are using the same controller for the emitter and the reciever,
   * we need to differentiate them
   */
  if (strncmp(wb_robot_get_name(), "MyBot emitter", 13) == 0) {
    robot_type = EMITTER;
    communication = wb_robot_get_device("emitter");

    /* if the cannel is not the right one, change it */
    const int channel = wb_emitter_get_channel(communication);
    if (channel != COMMUNICATION_CHANNEL) {
      wb_emitter_set_channel(communication, COMMUNICATION_CHANNEL);
    }
  } else if (strncmp(wb_robot_get_name(), "MyBot receiver", 14) == 0) {
    robot_type = RECEIVER;
    communication = wb_robot_get_device("receiver");
    wb_receiver_enable(communication, TIME_STEP);
  } else {
    printf("Unrecognized robot name '%s'. Exiting...\n", wb_robot_get_name());
    wb_robot_cleanup();
    return 0;
  }

  /* get a handler to the distance sensors and enable them */
  ps0 = wb_robot_get_device("ps0");
  ps7 = wb_robot_get_device("ps7");
  wb_distance_sensor_enable(ps0, TIME_STEP);
  wb_distance_sensor_enable(ps7, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * the emitter simply sends the message but the receiver
     * has to check if there is something before it can reads the buffer.
     */
    if (robot_type == EMITTER) {
      /* send null-terminated message */
      const char *message = "Hello!";
      wb_emitter_send(communication, message, strlen(message) + 1);
    } else {
      /* is there at least one packet in the receiver's queue ? */
      if (wb_receiver_get_queue_length(communication) > 0) {
        /* read current packet's data */
        const char *buffer = wb_receiver_get_data(communication);

        if (message_printed != 1) {
          /* print null-terminated message */
          printf("Communicating: received \"%s\"\n", buffer);
          message_printed = 1;
        }
        /* fetch next packet */
        wb_receiver_next_packet(communication);
      } else {
        if (message_printed != 2) {
          printf("Communication broken!\n");
          message_printed = 2;
        }
      }
    }

    const double ps0_value = wb_distance_sensor_get_value(ps0);
    const double ps7_value = wb_distance_sensor_get_value(ps7);

    if (ps7_value > 500) {
      /*
       * If both distance sensors are detecting something, this means that
       * we are facing a wall. In this case we need to move backwards.
       */
      if (ps0_value > 200) {
        if (robot_type == EMITTER) {
          left_speed = -SPEED;
          right_speed = -SPEED / 2;
        } else {
          left_speed = -SPEED / 2;
          right_speed = -SPEED;
        }
      } else {
        /*
         * we turn proportionnaly to the sensors value because the
         * closer we are from the wall, the more we need to turn.
         */
        left_speed = -ps7_value / 100;
        right_speed = (ps0_value / 100) + 0.5;
      }
    } else if (ps0_value > 500) {
      left_speed = (ps7_value / 100) + 0.5;
      right_speed = -ps0_value / 100;
    } else {
      /*
       * if nothing was detected we can move forward at maximal speed.
       */
      left_speed = SPEED;
      right_speed = SPEED;
    }

    /* set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}

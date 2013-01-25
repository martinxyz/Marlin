/*
  stepper_maxy.c - alternative stepper irq for Marlin

  Copyright (C) 2013 Martin Renold <martinxyz@gmx.ch>
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stepper_maxy.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#ifdef ADVANCE
#error stepper_maxy is not compatible with ADVANCE
#endif

bool stepper_maxy_in_control = false;
volatile static uint32_t max_stepper_irq_micros = 0;

// how big one full microstep is (for current_subposition)
#define SUBPOSITION_MICROSTEP (((int16_t)1)<<13)
// fractional position (sub-microsteps)
static int16_t current_subposition[NUM_AXIS] = { 0, 0, 0, 0 };
// velocity (substeps per interrupt)
static int16_t current_velocity[NUM_AXIS] = { 0, 0, 0, 0 };
static int16_t current_acceleration[NUM_AXIS] = { 0, 0, 0, 0 };
static int16_t current_jerk[NUM_AXIS] = { 0, 0, 0, 0 };

const int step_queue_max = 1000;
static bool step_queue[1000];

typedef struct {
  uint16_t duration;
  int16_t jerk;
} motion_segment;

#define MOTION_BUFFER_SIZE 32
static motion_segment motion_buffer[MOTION_BUFFER_SIZE];
volatile static int motion_buffer_read = 0;
volatile static int motion_buffer_write = 0;

void stepper_maxy_enable()
{
  SERIAL_ECHO_START;
  SERIAL_ECHO("max_stepper_irq_micros ");
  SERIAL_ECHOLN(max_stepper_irq_micros);
  SERIAL_ECHO_START;
  SERIAL_ECHOLN(micros());
  volatile uint16_t t0 = micros();
  volatile uint16_t t1 = micros();
  volatile uint16_t t3;
  t3 = t1-t0;
  SERIAL_ECHO_START;
  SERIAL_ECHO("micros() duration ");
  SERIAL_ECHOLN(t3);
  if (stepper_maxy_in_control) return;

  SERIAL_ECHO_START;
  SERIAL_ECHOLN("stepper_maxy taking over");
  uint16_t duration = micros() - t0;
  SERIAL_ECHO_START;
  SERIAL_ECHOLN(duration);
  SERIAL_ECHOLN(" micros");
  if (!stepper_maxy_in_control) {
    // process all pending blocks
    st_synchronize();
  }

  // DEBUG: clean start
  memset(current_subposition, 0, sizeof(current_subposition));
  memset(current_velocity, 0, sizeof(current_velocity));
  memset(current_acceleration, 0, sizeof(current_acceleration));
  memset(current_acceleration, 0, sizeof(current_acceleration));
  motion_buffer_read = 0;
  motion_buffer_write = 0;

  // enable irq
  stepper_maxy_in_control = true;

  // enable all axes
  enable_e0();
  enable_e1();
  enable_e2(); 
  enable_x();
  enable_y();
  enable_z();
}

void stepper_maxy_disable()
{
  if (!stepper_maxy_in_control) return;
  stepper_maxy_in_control = false;
  if (motion_buffer_write != motion_buffer_read) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("stepper_maxy killed and disabled");
  } else {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("stepper_maxy finished and disabled");
  }
  /*
  // the analog of st_synchronize()
  while (motion_buffer_write != motion_buffer_read) {
    manage_heater();
    manage_inactivity();
    lcd_update();
  }
  */
}

void stepper_maxy_test(int x, int y, int z, int e, int f)
{
  SERIAL_ECHO_START;
  SERIAL_ECHOLN("stepper_maxy_test {");
  //current_velocity[X_AXIS] = x;
  //current_velocity[Y_AXIS] = y;
  //current_velocity[Z_AXIS] = z;
  //current_velocity[E_AXIS] = e;
  if (x || y || z || e) {
    stepper_maxy_enable();
    if (motion_buffer_write == motion_buffer_read) {
      static bool reverse = false;
      memset(motion_buffer, 0, sizeof(motion_buffer));
motion_buffer[0] = {.duration=10000, .jerk=0};
motion_buffer[1] = {.duration=144, .jerk=103};
motion_buffer[2] = {.duration=78, .jerk=0};
motion_buffer[3] = {.duration=144, .jerk=-103};
motion_buffer[4] = {.duration=4634, .jerk=0};
motion_buffer[5] = {.duration=144, .jerk=-103};
motion_buffer[6] = {.duration=78, .jerk=0};
motion_buffer[7] = {.duration=0, .jerk=103};
motion_buffer[8] = {.duration=0, .jerk=0};
motion_buffer[9] = {.duration=0, .jerk=0};
motion_buffer[10] = {.duration=16000, .jerk=0};
motion_buffer[11] = {.duration=144, .jerk=-103};
motion_buffer[12] = {.duration=78, .jerk=0};
motion_buffer[13] = {.duration=144, .jerk=103};
motion_buffer[14] = {.duration=4634, .jerk=0};
motion_buffer[15] = {.duration=144, .jerk=103};
motion_buffer[16] = {.duration=78, .jerk=0};
motion_buffer[17] = {.duration=0, .jerk=-103};
motion_buffer[18] = {.duration=0, .jerk=0};
motion_buffer[19] = {.duration=0, .jerk=0};
      /*
        if (reverse) {
        for (int i=0; i<9; i++) {
        motion_buffer[9+i] = motion_buffer[i];
        motion_buffer[9+i].jerk = -motion_buffer[i].jerk;
        }
        motion_buffer_write = 2*9;
        } else {
        motion_buffer_write = 9;
        }
      */
      SERIAL_ECHO_START;
      SERIAL_ECHOLN("starting motion now");
      motion_buffer_write = 21; // go
      reverse = !reverse;
    } else {
      SERIAL_ECHO_START;
      SERIAL_ECHOLN("stepper_maxy busy, request ignored");
    }
  } else {
    stepper_maxy_disable();
  }
  SERIAL_ECHO_START;
  SERIAL_ECHOLN("stepper_maxy_test }");
}

void stepper_maxy_irq()
{
  uint32_t stepper_t0 = micros();
  //OCR1A = 2000; // irq every 1ms
  OCR1A = 100; // irq every 50us
  //OCR1A = 200; // irq every 100us

  const int16_t accel_scale    = 1<<9;
  const int16_t velocity_scale = 1;

  const int ax = X_AXIS;
  //const int ax = E_AXIS;

  if (motion_buffer_read != motion_buffer_write) {
    motion_segment * ms = &motion_buffer[motion_buffer_read];

    static bool start_of_segment = true;
    if (start_of_segment) {
      start_of_segment = false;
      current_jerk[ax] = ms->jerk;

      current_velocity[ax] += (ms->jerk*5+17)/32 / accel_scale; // jerk/6 (approximated)
      current_acceleration[ax] += ms->jerk/2;
      if (ms->duration == 0) {
        // stop everything
        current_jerk[ax] = 0;
        current_velocity[ax] = 0;
        current_acceleration[ax] = 0;
        current_subposition[ax] = 0;
      }
    }
    if (ms->duration == 0) {
      motion_buffer_read = (motion_buffer_read + 1) % MOTION_BUFFER_SIZE;
      start_of_segment = true;
    } else {
      ms->duration--;
    }
  }

  // motion
  //for (int i=0; i<NUM_AXIS; i++) {
  {
    // this takes 61us!!!!
    const int i = ax;
    current_subposition[i]  += (current_velocity[i] + velocity_scale/2)/velocity_scale; // + (current_acceleration[i]+accel_scale/4*velocity_scale)/(accel_scale/2*velocity_scale);
    current_velocity[i]     += (current_acceleration[i]+accel_scale/2)/accel_scale;
    current_acceleration[i] += current_jerk[i];

    // Formula: position     += velocity + acceleration/2 + jerk/6
    //          velocity     += acceleration + jerk/2
    //          acceleration += jerk
    //
    // max. sensible subposition = ca. +/- 4 microsteps (xy ca. 2000mm/s, extruder ca. 100mm/s)
    // max. sensible velocity = ca. +/- 1000 mm/s
    // max. sensible acceleration? usually 1g...10g = 10000mm/s2 = steps/tick
  }

  // set stepper direction
  if (current_subposition[X_AXIS] > 0) {
    WRITE(X_DIR_PIN, !INVERT_X_DIR);
  } else {
    WRITE(X_DIR_PIN, INVERT_X_DIR);
  }

  // set stepper direction
  if (current_subposition[Y_AXIS] > 0) {
    WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
  } else {
    WRITE(Y_DIR_PIN, INVERT_Y_DIR);
  }

  // set stepper direction
  if (current_subposition[Z_AXIS] > 0) {
    WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
  } else {
    WRITE(Z_DIR_PIN, INVERT_Z_DIR);
  }

  // set stepper direction
  if (current_subposition[E_AXIS] > 0) {
    WRITE(E0_DIR_PIN, !INVERT_E0_DIR);
  } else {
    WRITE(E0_DIR_PIN, INVERT_E0_DIR);
  }

  // 200ns A4988 setup time (one nop is 62.5ns)
  // (not a concern, since we go through all axes again now)
  //asm volatile ("nop\nnop\nnop\nnop\n");

  if (current_subposition[X_AXIS] >= SUBPOSITION_MICROSTEP/2) {
    WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN); // start of step pulse
    current_subposition[X_AXIS] -= SUBPOSITION_MICROSTEP;
  } else if (current_subposition[X_AXIS] < -SUBPOSITION_MICROSTEP/2) {
    WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN); // start of step pulse
    current_subposition[X_AXIS] += SUBPOSITION_MICROSTEP;
  }

  if (current_subposition[Y_AXIS] >= SUBPOSITION_MICROSTEP/2) {
    WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN); // start of step pulse
    current_subposition[Y_AXIS] -= SUBPOSITION_MICROSTEP;
  } else if (current_subposition[Y_AXIS] < -SUBPOSITION_MICROSTEP/2) {
    WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN); // start of step pulse
    current_subposition[Y_AXIS] += SUBPOSITION_MICROSTEP;
  }

  if (current_subposition[Z_AXIS] >= SUBPOSITION_MICROSTEP/2) {
    WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN); // start of step pulse
    current_subposition[Z_AXIS] -= SUBPOSITION_MICROSTEP;
  } else if (current_subposition[Z_AXIS] < -SUBPOSITION_MICROSTEP/2) {
    WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN); // start of step pulse
    current_subposition[Z_AXIS] += SUBPOSITION_MICROSTEP;
  }

  if (current_subposition[E_AXIS] >= SUBPOSITION_MICROSTEP/2) {
    WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN); // start of step pulse
    current_subposition[E_AXIS] -= SUBPOSITION_MICROSTEP;
  } else if (current_subposition[E_AXIS] < -SUBPOSITION_MICROSTEP/2) {
    WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN); // start of step pulse
    current_subposition[E_AXIS] += SUBPOSITION_MICROSTEP;
  }

  // 1us A4988 minimum HIGH pulse width
  asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");

  // end of step pulse (if a step pulse was made)
  WRITE(X_STEP_PIN,  INVERT_X_STEP_PIN);
  WRITE(Y_STEP_PIN,  INVERT_Y_STEP_PIN);
  WRITE(Z_STEP_PIN,  INVERT_Z_STEP_PIN);
  WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);

  /*
    #if Y_MAX_PIN > -1
    bool y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
    if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
    endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
    endstop_y_hit=true;
    step_events_completed = current_block->step_event_count;
    }
    old_y_max_endstop = y_max_endstop;
    #endif
  */

  uint32_t stepper_duration = micros() - stepper_t0;
  if (stepper_duration > max_stepper_irq_micros) {
    max_stepper_irq_micros = stepper_duration;
  }
}

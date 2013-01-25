/*
  stepper_maxy.h - alternative stepper motor driver

  Copyright (c) 2013 Martin Renold

  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted, provided that the above
  copyright notice and this permission notice appear in all copies.
 
  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#ifndef stepper_maxy
#define stepper_maxy

#include "Marlin.h"

extern bool stepper_maxy_in_control; 

typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long substeps_per_tick_x, steps_y, steps_z, steps_e;  // Step count along each axis
  long ticks;
} block_maxy_t;

// timer interrupt handler, called from the real handler in stepper.cpp 
void stepper_maxy_irq();
void stepper_maxy_test(int x, int y, int z, int e, int f);

#endif

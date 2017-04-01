/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Grbl driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of Grbl

  Copyright (c) 2016 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// Increase Buffers to make use of extra SRAM
//#define RX_BUFFER_SIZE		256
//#define TX_BUFFER_SIZE		128
//#define BLOCK_BUFFER_SIZE	36
// LINE_BUFFER_SIZE must be a multiple of 4 - 1 (see settings.h - EEPROM writes has to be word-aligned)

//

#define EEPROMOFFSET 0


// Define step pulse output pins. NOTE: Routed to RGB led on Tiva C LaunchPad.
#define STEP_PERIPH	SYSCTL_PERIPH_GPIOF
#define STEP_PORT	GPIO_PORTF_BASE
#define X_STEP_PIN	GPIO_PIN_1
#define Y_STEP_PIN	GPIO_PIN_2
#define Z_STEP_PIN	GPIO_PIN_3
#define HWSTEP_MASK (X_STEP_PIN|Y_STEP_PIN|Z_STEP_PIN) // All direction bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PERIPH	SYSCTL_PERIPH_GPIOC
#define DIRECTION_PORT		GPIO_PORTC_BASE
#define X_DIRECTION_PIN		GPIO_PIN_5
#define Y_DIRECTION_PIN		GPIO_PIN_6
#define Z_DIRECTION_PIN		GPIO_PIN_7
#define HWDIRECTION_MASK    (X_DIRECTION_PIN|Y_DIRECTION_PIN|Z_DIRECTION_PIN) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PERIPH	SYSCTL_PERIPH_GPIOE
#define STEPPERS_DISABLE_PORT   GPIO_PORTE_BASE
#define STEPPERS_DISABLE_PIN    GPIO_PIN_0

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
#define LIMIT_PERIPH SYSCTL_PERIPH_GPIOA
#define LIMIT_PORT   GPIO_PORTA_BASE
#define X_LIMIT_PIN  GPIO_PIN_2
#define Y_LIMIT_PIN  GPIO_PIN_3
#define Z_LIMIT_PIN  GPIO_PIN_4
#define HWLIMIT_MASK   (X_LIMIT_PIN|Y_LIMIT_PIN|Z_LIMIT_PIN) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PERIPH	SYSCTL_PERIPH_GPIOC
#define SPINDLE_ENABLE_PORT		GPIO_PORTB_BASE
#define SPINDLE_ENABLE_PIN		GPIO_PIN_0

#define SPINDLE_DIRECTION_PERIPH	SYSCTL_PERIPH_GPIOC
#define SPINDLE_DIRECTION_PORT		GPIO_PORTB_BASE
#define SPINDLE_DIRECTION_PIN		GPIO_PIN_1

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PERIPH	SYSCTL_PERIPH_GPIOA
#define COOLANT_FLOOD_PORT		GPIO_PORTA_BASE
#define COOLANT_FLOOD_PIN		GPIO_PIN_5

#define COOLANT_MIST_PERIPH	SYSCTL_PERIPH_GPIOA
#define COOLANT_MIST_PORT	GPIO_PORTA_BASE
#define COOLANT_MIST_PIN	GPIO_PIN_6

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PERIPH		SYSCTL_PERIPH_GPIOD
#define CONTROL_PORT		GPIO_PORTD_BASE
#define RESET_PIN			GPIO_PIN_0
#define FEED_HOLD_PIN		GPIO_PIN_1
#define CYCLE_START_PIN		GPIO_PIN_2
#define SAFETY_DOOR_PIN		GPIO_PIN_3
#define HWCONTROL_MASK		(RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN|SAFETY_DOOR_PIN)
#define CONTROL_INVERT_MASK HWCONTROL_MASK // May be re-defined to only invert certain control pins.

// Define probe switch input pin.
#define PROBE_PERIPH	SYSCTL_PERIPH_GPIOA
#define PROBE_PORT		GPIO_PORTA_BASE
#define PROBE_PIN		GPIO_PIN_7
#define PROBE_MASK		PROBE_PIN

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  #define PWM_MAX_VALUE			1000	// For 10Khz PWM frequency
  #define SPINDLEPPERIPH		SYSCTL_PERIPH_GPIOE
  #define SPINDLEPPORT			GPIO_PORTE_BASE
  #define SPINDLEPPIN			GPIO_PIN_4
  #define SPINDLEPWM			PWM1_BASE
  #define SPINDLEPWM_GEN		PWM_GEN_1
  #define SPINDLEPWM_OUT		PWM_OUT_2
  #define SPINDLEPWM_OUT_BIT	PWM_OUT_2_BIT
  #define SPINDLEPWM_MAP		GPIO_PE4_M1PWM2
#endif // End of VARIABLE_SPINDLE

  // Variable spindle configuration below. Do not change unless you know what you are doing.
  // NOTE: Only used when variable spindle is enabled.
  #define SPINDLE_PWM_MAX_VALUE     255 // Don't change. 328p fast PWM mode fixes top value as 255.
  #ifndef SPINDLE_PWM_MIN_VALUE
    #define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
  #endif
  #define SPINDLE_PWM_OFF_VALUE     0
  #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)

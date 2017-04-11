/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of Grbl

  Copyright (c) 2017 Terje Io
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

#include "grbl.h"

#include "tiva.h"
#include "driver.h"
#include "eeprom.h"
#include "serial.h"
#include "usermcodes.h"

// TODO: handle F_CPU in a portable way - it is not used as CPU frequency but rather ticks per counter increment/decrement for timer(s)
// This implementation prescales step counter by four for a F_CPU of 20MHz to avoid overflow in Grbl code
// However, the best solution may be to change cycles_per_tick in stepper.c to 32 bit and handle prescaling in the driver?

// prescale step counter to 20Mhz (80 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 3

static uint32_t ms_delayCycles, us_delayCycles;
static uint8_t step_port_invert_mask, dir_port_invert_mask, pulse_time;

#ifdef VARIABLE_SPINDLE
    static bool pwmEnabled = false;
    static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
#endif

#ifdef STEP_PULSE_DELAY
static uint8_t next_step_outbits, step_port_invert_mask, dir_port_invert_mask;
#endif

#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
	static uint16_t step_prescaler[4] = {
		STEPPER_DRIVER_PRESCALER,
		STEPPER_DRIVER_PRESCALER,
		STEPPER_DRIVER_PRESCALER + 8,
		STEPPER_DRIVER_PRESCALER + 64
	};
#endif

// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert_mask;

// Interrupt handler prototypes

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void limit_isr (void);
static void control_isr (void);
#ifdef ENABLE_SOFTWARE_DEBOUNCE
static void software_debounce_isr (void);
#endif

// NOTE: driver_delay_ms may be called with 0, causes a hang on Tiva if passed on...

static void driver_delay_ms (uint16_t ms) {
	if(ms > 0)
		SysCtlDelay(ms_delayCycles * ms) ;
}

static void driver_delay_us (uint32_t us) {
	SysCtlDelay(us < 1 ? us_delayCycles / 10 : --us);
}

// Enable/disable steppers, called from st_wake_up() and st_go_idle()
static void stepperEnable (bool on) {

	if (bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE))
		on = !on; // Apply pin invert.

    GPIOPinWrite(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_PIN, on ? STEPPERS_DISABLE_PIN : 0);
}

// Sets up for a step pulse and forces a stepper driver interrupt, called from st_wake_up()
// NOTE: delay and pulse_time are # of microseconds
static void stepperWakeUp (uint8_t delay) {

#ifdef STEP_PULSE_DELAY
	pulse_time += delay;
	TimerMatchSet(TIMER2_BASE, TIMER_A, pulse_time - delay);
#endif
	TimerLoadSet(TIMER2_BASE, TIMER_A, pulse_time - 1);

	TimerLoadSet(TIMER1_BASE, TIMER_A, 5000); 	// dummy...
	TimerEnable(TIMER1_BASE, TIMER_A);
	IntPendSet(INT_TIMER1A); 					// force immediate Timer1 interrupt
}

// Disables stepper driver interrupts, called from st_go_idle()
static void stepperGoIdle (void) {
	TimerDisable(TIMER1_BASE, TIMER_A);
}

// Sets up stepper driver interrupt timeout, called from stepper_driver_interrupt_handler()
static void stepperCyclesPerTick (uint32_t cycles_per_tick) {
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint16_t prescaler;
    // Compute step timing and timer prescalar for normal step generation.
    if (cycles_per_tick < (1UL << 16)) // < 65536  (4.1ms @ 16MHz)
      prescaler = step_prescaler[1]; // prescaler: 0
    else if (cycles_per_tick < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
      prescaler = step_prescaler[2]; // prescaler: 8
      cycles_per_tick = cycles_per_tick >> 3;
    } else {
      prescaler = step_prescaler[3]; // prescaler: 64
      cycles_per_tick = cycles_per_tick >> 6;
    }
	TimerPrescaleSet(TIMER1_BASE, TIMER_A, prescaler);
#endif
	TimerLoadSet(TIMER1_BASE, TIMER_A, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xffff /*Just set the slowest speed possible.*/);
}

// Set stepper pulse output pins, called from st_reset()
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z, needs to be mapped to physical pins by bit shifting or other means
//       other means may be by bit-banding or lookup table...
inline static void stepperSetStepOutputs (uint8_t step_outbits) {
	GPIOPinWrite(STEP_PORT, HWSTEP_MASK, (step_outbits ^ step_port_invert_mask) << 1);
}

// Set stepper direction output pins, called from st_reset()
// NOTE1: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z, needs to be mapped to physical pins by bit shifting or other means
//        other means may be by bit-banding or lookup table...
inline static void stepperSetDirOutputs (uint8_t dir_outbits) {
	GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, (dir_outbits ^ dir_port_invert_mask) << 5);
}

// Sets stepper direction and pulse pins and starts a step pulse, called from stepper_driver_interrupt_handler()
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStart (uint8_t dir_outbits, uint8_t step_outbits, uint32_t spindle_pwm) {

    stepperSetDirOutputs(step_outbits);

#ifdef STEP_PULSE_DELAY
	next_step_outbits = (step_outbits ^ step_port_invert_mask) << 1; // Store out_bits
#else  // Normal operation
	stepperSetStepOutputs(step_outbits);
#endif

	TimerEnable(TIMER2_BASE, TIMER_A);
}

// Disable limit pins interrupt, called from mc_homing_cycle()
static void limitsEnable (bool on) {
    if (on && bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE))
        GPIOIntEnable(LIMIT_PORT, HWLIMIT_MASK); // Enable Pin Change Interrupt
    else
        GPIOIntDisable(LIMIT_PORT, HWLIMIT_MASK); // Disable Pin Change Interrupt
}

// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
inline static uint8_t limitsGetState() {

	uint8_t pins = (uint8_t)GPIOPinRead(LIMIT_PORT, HWLIMIT_MASK);

	if (bit_isfalse(settings.flags, BITFLAG_INVERT_LIMIT_PINS))
		pins ^= HWLIMIT_MASK;

	return pins >> 2;
}

inline static uint8_t systemGetState (void) {

    uint8_t flags = GPIOPinRead(CONTROL_PORT, HWCONTROL_MASK) ^ CONTROL_INVERT_MASK;

	return
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
	        (flags & SAFETY_DOOR_PIN ? CONTROL_PIN_INDEX_SAFETY_DOOR : 0) |
#endif
	        (flags & RESET_PIN ? CONTROL_PIN_INDEX_RESET : 0) |
	         (flags & FEED_HOLD_PIN ? CONTROL_PIN_INDEX_FEED_HOLD : 0) |
	          (flags & CYCLE_START_PIN ? CONTROL_PIN_INDEX_CYCLE_START : 0);
}

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigureInvertMask(bool is_probe_away)
{

  probe_invert_mask = bit_isfalse(settings.flags, BITFLAG_INVERT_PROBE_PIN) ? PROBE_MASK : 0;

  if (is_probe_away)
	  probe_invert_mask ^= PROBE_MASK;
}

// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
bool probeGetState (void) {
    return (((uint8_t)GPIOPinRead(PROBE_PORT, PROBE_MASK)) ^ probe_invert_mask) != 0;
}

// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
uint32_t spindleComputePWMValue(float rpm) // 328p PWM register is 8-bit.
{
  uint32_t pwm_value;
  rpm *= (0.010f*sys.spindle_speed_ovr); // Scale by spindle speed override value.
  // Calculate PWM register value based on rpm max/min settings and programmed rpm.
  if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
    // No PWM range possible. Set simple on/off spindle control pin state.
    sys.spindle_speed = settings.rpm_max;
    pwm_value = SPINDLE_PWM_MAX_VALUE;
  } else if (rpm <= settings.rpm_min) {
    if (rpm == 0.0) { // S0 disables spindle
      sys.spindle_speed = 0.0;
      pwm_value = SPINDLE_PWM_OFF_VALUE;
    } else { // Set minimum PWM output
      sys.spindle_speed = settings.rpm_min;
      pwm_value = SPINDLE_PWM_MIN_VALUE;
    }
  } else {
    // Compute intermediate PWM value with linear spindle speed model.
    // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
    sys.spindle_speed = rpm;
    pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
  }
  return(pwm_value);
}

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetState(uint8_t state, float rpm) {

#ifdef VARIABLE_SPINDLE

	if (state == SPINDLE_DISABLE || rpm == 0.0f) {
#else
	if (state == SPINDLE_DISABLE) {
#endif
#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
  #ifdef INVERT_SPINDLE_ENABLE_PIN
	GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, SPINDLE_ENABLE_PIN);
  #else
	GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, 0);
  #endif
#else
  #ifdef INVERT_SPINDLE_ENABLE_PIN
	GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, SPINDLE_ENABLE_PIN);
  #else
	GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, 0);
  #endif
#endif
#ifdef VARIABLE_SPINDLE
	pwmEnabled = false;
	PWMGenDisable(SPINDLEPWM, SPINDLEPWM_GEN);
#endif

	} else {

#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
		GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, state == SPINDLE_ENABLE_CW ? 0 : SPINDLE_DIRECTION_PIN);
#endif

	}
}

static uint8_t spindleGetState (void) {

    uint8_t state = SPINDLE_STATE_DISABLE;

 #ifdef VARIABLE_SPINDLE

  #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
  // No spindle direction output pin.
   #ifdef INVERT_SPINDLE_ENABLE_PIN
    if (!GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN))
        state = SPINDLE_STATE_CW;
   #else
    if (GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN))
        state = SPINDLE_STATE_CW);
   #endif
  #else
    if (pwmEnabled) // Check if PWM is enabled.
        state = GPIOPinRead(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) ? SPINDLE_STATE_CCW : SPINDLE_STATE_CW;
  #endif

 #else
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if (!GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN))
  #else
    if (GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN))
  #endif
    state = GPIOPinRead(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) ? SPINDLE_STATE_CCW : SPINDLE_STATE_CW;

 #endif

    return state;
}

inline static uint32_t spindleSetSpeed (uint32_t pwm_value) {

    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
          spindleSetState(SPINDLE_DISABLE, SPINDLE_PWM_OFF_VALUE);
      } else {
          pwmEnabled = true;
          PWMGenEnable(SPINDLEPWM, SPINDLEPWM_GEN);
 		#ifdef INVERT_SPINDLE_ENABLE_PIN
			GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, SPINDLE_ENABLE_PIN);
		#else
			GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, 0);
		#endif
      }
    #else
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
          pwmEnabled = false;
          PWMGenDisable(SPINDLEPWM, SPINDLEPWM_GEN); // Disable PWM. Output voltage is zero.
      } else {
          pwmEnabled = true;
          PWMGenPeriodSet(SPINDLEPWM, SPINDLEPWM_GEN, SPINDLE_PWM_MAX_VALUE); // 2KHz on Arduino
          PWMPulseWidthSet(SPINDLEPWM, SPINDLEPWM_OUT, pwm_value);
          PWMGenEnable(SPINDLEPWM, SPINDLEPWM_GEN); // Ensure PWM output is enabled.
      }
    #endif

    return pwm_value;
}

// Start/stop coolant (and mist if enabled), called by coolant_run() and protocol_execute_realtime()
static void coolantSetState (uint8_t mode)
{

	switch(mode) {

		case COOLANT_FLOOD_ENABLE:
		  #ifdef INVERT_COOLANT_FLOOD_PIN
			GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, 0);
		  #else
			GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, COOLANT_FLOOD_PIN);
		  #endif
			break;

		case COOLANT_MIST_ENABLE:
		  #ifdef INVERT_COOLANT_MIST_PIN
			GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, 0);
		  #else
			GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, COOLANT_MIST_PIN);
		  #endif
			break;

		default:
		#ifdef INVERT_COOLANT_FLOOD_PIN
			GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, COOLANT_FLOOD_PIN);
		#else
			GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, 0);
		#endif

		#ifdef INVERT_COOLANT_MIST_PIN
			GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, COOLANT_MIST_PIN);
		#else
			GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, 0);
		#endif
			break;

	}
}

static uint8_t coolantGetState (void) {

	uint8_t state = 0;

  #ifdef INVERT_COOLANT_FLOOD_PIN
	state = GPIOPinRead(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN) == 0 ? COOLANT_STATE_FLOOD : 0;
  #else
	state = GPIOPinRead(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN) ? COOLANT_STATE_FLOOD : 0;
  #endif

  #ifdef INVERT_COOLANT_MIST_PIN
	state |= GPIOPinRead(COOLANT_MIST_PORT, COOLANT_MIST_PIN) == 0 ? COOLANT_STATE_MIST : 0;
  #else
	state |= GPIOPinRead(COOLANT_MIST_PORT, COOLANT_MIST_PIN) ? COOLANT_STATE_MIST : 0;
  #endif

    return state;

}

static uint16_t serialRxBuffer (void) {
    return RX_BUFFER_SIZE;
}

// Callback to inform settings has been changed, called by settings_store_global_setting() and local mcu_init()
// Used here to set assorted helper variables
void settings_changed (settings_t *settings) {
/*
	if(bit_istrue(settings->flags, BITFLAG_HARD_LIMIT_ENABLE)) {
		GPIOIntClear(CONTROL_PORT, HWCONTROL_MASK);					// Clear any pending interrupt
		GPIOIntEnable(LIMIT_PORT, HWLIMIT_MASK);                   	// and enable Pin Change Interrupt
	} else
		GPIOIntDisable(LIMIT_PORT, HWLIMIT_MASK);						// Disable Pin Change Interrupt
*/
	// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.

    pulse_time = settings->pulse_microseconds;
    step_port_invert_mask = settings->step_invert_mask;
    dir_port_invert_mask = settings->dir_invert_mask;

//    curr_dir_outbits = 0xFF; // "forget" current dir flags
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable) TODO: switch to bit-banding?
static void bitsSetAtomic (volatile uint8_t *value, uint8_t bits) {
	IntMasterDisable();
	*value |= bits;
	IntMasterEnable();
}

static void bitsClearAtomic (volatile uint8_t *value, uint8_t bits) {
	IntMasterDisable();
	*value &= ~bits;
	IntMasterEnable();
}

// Initializes MCU peripherals for Grbl use TODO: rename?
static void mcu_init (void) {

  // System init

//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
#ifdef ENABLE_SOFTWARE_DEBOUNCE
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
#endif
#ifdef VARIABLE_SPINDLE
	SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
#endif

    SysCtlDelay(26); // wait a bit for peripherals to wake up

  // Control pins init

	GPIOPinTypeGPIOInput(CONTROL_PORT, HWCONTROL_MASK);
	GPIOIntRegister(CONTROL_PORT, &control_isr); 		     // Register a call-back funcion for interrupt
  #ifdef DISABLE_CONTROL_PIN_PULL_UP
	GPIOPadConfigSet(CONTROL_PORT, HWCONTROL_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD); //Enable weak pull-downs
	GPIOIntTypeSet(CONTROL_PORT, HWCONTROL_MASK, GPIO_RISING_EDGE); // Enable specific pins of the Pin Change Interrupt
  #else
	GPIOPadConfigSet(CONTROL_PORT, HWCONTROL_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Enable weak pull-ups
	GPIOIntTypeSet(CONTROL_PORT, HWCONTROL_MASK, GPIO_FALLING_EDGE); // Enable specific pins of the Pin Change Interrupt
  #endif
	GPIOIntClear(CONTROL_PORT, HWCONTROL_MASK);					 // Clear any pending interrupt
	GPIOIntEnable(CONTROL_PORT, HWCONTROL_MASK);                   // and enable Pin Change Interrupt

  // Stepper init

	GPIOPinTypeGPIOOutput(STEP_PORT, HWSTEP_MASK);
	GPIOPadConfigSet(STEP_PORT, HWSTEP_MASK, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

	GPIOPinTypeGPIOOutput(DIRECTION_PORT, HWDIRECTION_MASK);
	GPIOPadConfigSet(DIRECTION_PORT, HWDIRECTION_MASK, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

	GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_PIN);

	// Configure stepper driver timer
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_PERIODIC);
	IntPrioritySet(INT_TIMER1A, 0x20); // lower priority than for Timer2 (which resets the step-dir signal)
	TimerControlStall(TIMER1_BASE, TIMER_A, true); //timer1 will stall in debug mode
	TimerIntRegister(TIMER1_BASE, TIMER_A, stepper_driver_isr);
	TimerIntClear(TIMER1_BASE, 0xFFFF);
	IntPendClear(INT_TIMER1A);
	TimerPrescaleSet(TIMER1_BASE, TIMER_A, STEPPER_DRIVER_PRESCALER); // 20 MHz clock
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Configure step pulse timer
//	TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
	IntPrioritySet(INT_TIMER2A, 0x00); // highest priority - higher than for Timer1 (which sets the step-dir output)
	TimerControlStall(TIMER2_BASE, TIMER_A, true); //timer2 will stall in debug mode
	TimerIntRegister(TIMER2_BASE, TIMER_A, stepper_pulse_isr);
	TimerIntClear(TIMER2_BASE, 0xFFFF);
	IntPendClear(INT_TIMER2A);
	TimerPrescaleSet(TIMER2_BASE, TIMER_A, 79); // for 1uS per count
#ifdef STEP_PULSE_DELAY
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);
#else
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
#endif

#ifdef ENABLE_SOFTWARE_DEBOUNCE
    IntPrioritySet(INT_TIMER3A, 0x40); // lower priority than for Timer2 (which resets the step-dir signal)
	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
	TimerControlStall(TIMER3_BASE, TIMER_A, true); //timer2 will stall in debug mode
	TimerIntRegister(TIMER3_BASE, TIMER_A, software_debounce_isr);
	TimerIntClear(TIMER3_BASE, 0xFFFF);
	IntPendClear(INT_TIMER3A);
	TimerPrescaleSet(TIMER3_BASE, TIMER_A, 79); // configure for 1us per count
	TimerLoadSet(TIMER3_BASE, TIMER_A, 32000);  // and for a total of 32ms
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
#endif

  // Limits init

	GPIOPinTypeGPIOInput(LIMIT_PORT, HWLIMIT_MASK);
	GPIOIntRegister(LIMIT_PORT, limit_isr); 		     // Register a call-back funcion for interrupt
#ifdef DISABLE_LIMIT_PIN_PULL_UP
	GPIOPadConfigSet(LIMIT_PORT, HWLIMIT_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD); //Enable weak pull-downs
	GPIOIntTypeSet(CONTROL_PORT, HWCONTROL_MASK, GPIO_RISING_EDGE); // Enable specific pins of the Pin Change Interrupt
#else
	GPIOPadConfigSet(LIMIT_PORT, HWLIMIT_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Enable weak pull-ups
	GPIOIntTypeSet(CONTROL_PORT, HWCONTROL_MASK, GPIO_FALLING_EDGE); // Enable specific pins of the Pin Change Interrupt
#endif

  // Probe init

	GPIOPinTypeGPIOInput(PROBE_PORT, PROBE_MASK);
#ifdef DISABLE_PROBE_PIN_PULL_UP
	GPIOPadConfigSet(LIMIT_PORT, PROBE_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD); //Enable weak pull-downs
#else
	GPIOPadConfigSet(PROBE_PORT, PROBE_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Enable weak pull-ups
#endif

  // Spindle init

	GPIOPinTypeGPIOOutput(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
	GPIOPinTypeGPIOOutput(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);

#ifdef VARIABLE_SPINDLE
    GPIOPinConfigure(SPINDLEPWM_MAP);
    GPIOPinTypePWM(SPINDLEPPORT, SPINDLEPPIN);
    GPIOPadConfigSet(SPINDLEPPORT, SPINDLEPPIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    PWMGenConfigure(SPINDLEPWM, SPINDLEPWM_GEN, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_GEN_SYNC_LOCAL);
	PWMGenPeriodSet(SPINDLEPWM, SPINDLEPWM_GEN, PWM_MAX_VALUE);
    PWMOutputState(SPINDLEPWM, SPINDLEPWM_OUT_BIT, true);

    pwm_gradient = SPINDLE_PWM_RANGE / (settings.rpm_max - settings.rpm_min);

#endif

  // Coolant init

	GPIOPinTypeGPIOOutput(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
#ifdef ENABLE_M7
	GPIOPinTypeGPIOOutput(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
#endif

  // Set defaults

    setSerialReceiveCallback(hal.protocol_process_realtime);
    spindleSetState(SPINDLE_DISABLE, SPINDLE_PWM_OFF_VALUE);
    coolantSetState(COOLANT_STATE_DISABLE);
    stepperSetDirOutputs(0);

}

// Initialize HAL pointers
// NOTE: Grbl is not yet configured (from EEPROM data), mcu_init() will be called when done
bool driver_init (void) {

	ms_delayCycles = SysCtlClockGet() / 3000; // tics per ms
	us_delayCycles = ms_delayCycles / 1000; // tics per us

	// Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
	EEPROMInit();

	serialInit();

	hal.initMCU = &mcu_init;
	hal.f_step_timer = 20000000;
	hal.delay_ms = &driver_delay_ms;
	hal.delay_us = &driver_delay_us;
	hal.stepper_wake_up = &stepperWakeUp;
	hal.stepper_go_idle = &stepperGoIdle;
	hal.stepper_enable = &stepperEnable;
	hal.stepper_set_outputs = &stepperSetStepOutputs;
	hal.stepper_set_directions = &stepperSetDirOutputs;
	hal.stepper_cycles_per_tick = &stepperCyclesPerTick;
	hal.stepper_pulse_start = &stepperPulseStart;
	hal.limits_enable = &limitsEnable;
	hal.limits_get_state = &limitsGetState;
	hal.coolant_set_state = &coolantSetState;
	hal.coolant_get_state = &coolantGetState;
	hal.probe_get_state = &probeGetState;
	hal.probe_configure_invert_mask = &probeConfigureInvertMask;
	hal.spindle_set_state = &spindleSetState;
	hal.spindle_get_state = &spindleGetState;
	hal.spindle_set_speed = &spindleSetSpeed;
	hal.spindle_compute_pwm_value = &spindleComputePWMValue;
	hal.system_control_get_state = &systemGetState;
    hal.serial_read = &serialGetC;
    hal.serial_write = (void (*)(uint8_t))&serialPutC;
    hal.serial_write_string = &serialWriteS;
    hal.serial_get_rx_buffer_size = &serialRxBuffer;
    hal.serial_get_rx_buffer_available = &serialRxFree;
    hal.serial_reset_read_buffer = &serialRxFlush;
    hal.serial_cancel_read_buffer = &serialRxCancel;
	hal.eeprom_get_char = &eepromGetChar;
	hal.eeprom_put_char = &eepromPutChar;
	hal.memcpy_to_eeprom_with_checksum = &eepromWriteBlockWithChecksum;
	hal.memcpy_from_eeprom_with_checksum = &eepromReadBlockWithChecksum;
	hal.settings_changed = &settings_changed;
	hal.set_bits_atomic = &bitsSetAtomic;
	hal.clear_bits_atomic = &bitsClearAtomic;
    hal.userdefined_mcode_check = &userMCodeCheck;
    hal.userdefined_mcode_validate = &userMCodeValidate;
    hal.userdefined_mcode_execute = &userMCodeExecute;
    hal.hasEEPROM = true;

// no need to move version check before init - compiler will fail any mismatch for existing entries
	return hal.version == 1;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void) {

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

	hal.stepper_interrupt_callback();

}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/
// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
// NOTE: TivaC has a shared interrupt for match and timeout
static void stepper_pulse_isr (void) {

#ifdef STEP_PULSE_DELAY
	uint32_t iflags = TimerIntStatus(TIMER2_BASE, true);
	TimerIntClear(TIMER2_BASE, iflags); // clear interrupt flags
	GPIOPinWrite(STEP_PORT, HWSTEP_MASK, iflags & TIMER_TIMA_MATCH ? next_step_outbits : step_port_invert_mask);
#else
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
	GPIOPinWrite(STEP_PORT, HWSTEP_MASK, step_port_invert_mask);
#endif
}

#ifdef ENABLE_SOFTWARE_DEBOUNCE
static void software_debounce_isr (void) {

	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

	uint8_t state = limitsGetState();

	if(state) //TODO: add check for limit swicthes having same state as when limit_isr were invoked?
		hal.limit_interrupt_callback(limitsGetState());
}
#endif

static void limit_isr (void) {

	uint32_t iflags = GPIOIntStatus(LIMIT_PORT, true) & HWLIMIT_MASK;

	if(iflags) {
		GPIOIntClear(LIMIT_PORT, iflags);
#ifdef ENABLE_SOFTWARE_DEBOUNCE
		// TODO: disable interrups here and reenable in software_debounce_isr?
        TimerLoadSet(TIMER3_BASE, TIMER_A, 32000);  // 32ms
        TimerEnable(TIMER3_BASE, TIMER_A);
#else
		hal.limit_interrupt_callback(limitsGetState());
#endif
	}
}

static void control_isr (void) {
// No debounce??
	uint8_t iflags = GPIOIntStatus(CONTROL_PORT, true) & HWCONTROL_MASK;

	if(iflags) {
		GPIOIntClear(CONTROL_PORT, iflags);
		hal.control_interrupt_callback(systemGetState());
	}
}


/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of Grbl

  Copyright (c) 2016-2017 Terje Io
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
#include "keypad.h"

// prescale step counter to 20Mhz (80 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 3

static volatile uint32_t ms_count = 1; // NOTE: initial value 1 is for "resetting" systick timer
static uint8_t pulse_time, pulse_delay;
static bool pwmEnabled = false, IOInitDone = false;
static axes_signals_t step_port_invert_mask, dir_port_invert_mask, next_step_outbits;
static spindle_pwm_t spindle_pwm;
static void (*delayCallback)(void) = 0;

static uint16_t step_prescaler[3] = {
    STEPPER_DRIVER_PRESCALER,
    STEPPER_DRIVER_PRESCALER + 8,
    STEPPER_DRIVER_PRESCALER + 64
};

// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert_mask;

// Interrupt handler prototypes

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void limit_isr (void);
static void limit_isr_debounced (void);
static void control_isr (void);
static void software_debounce_isr (void);
static void systick_isr (void);

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
	if((ms_count = ms) > 0) {
		SysTickEnable();
		if(!(delayCallback = callback))
			while(ms_count);
	} else if(callback)
		callback();
}

// Enable/disable steppers, called from st_wake_up() and st_go_idle()
static void stepperEnable (bool on)
{
	if (settings.flags.invert_st_enable)
		on = !on; // Apply pin invert.

    GPIOPinWrite(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_PIN, on ? STEPPERS_DISABLE_PIN : 0);
}

// Sets up for a step pulse and forces a stepper driver interrupt, called from st_wake_up()
// NOTE: delay and pulse_time are # of microseconds
static void stepperWakeUp ()
{
    if(pulse_delay) {
        pulse_time += pulse_delay;
        TimerMatchSet(TIMER2_BASE, TIMER_A, pulse_time - pulse_delay);
    }

    // Enable stepper drivers.
    stepperEnable(true);

    TimerLoadSet(TIMER2_BASE, TIMER_A, pulse_time - 1);
	TimerLoadSet(TIMER1_BASE, TIMER_A, 5000); 	// dummy...
	TimerEnable(TIMER1_BASE, TIMER_A);
	IntPendSet(INT_TIMER1A); 					// force immediate Timer1 interrupt
}

// Disables stepper driver interrupts, called from st_go_idle()
static void stepperGoIdle (void)
{
	TimerDisable(TIMER1_BASE, TIMER_A);
}

// Sets up stepper driver interrupt timeout, called from stepper_driver_interrupt_handler() - AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
	TimerLoadSet(TIMER1_BASE, TIMER_A, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFF /*Just set the slowest speed possible.*/);
}

// Sets up stepper driver interrupt timeout, called from stepper_driver_interrupt_handler() - "Normal" version
static void stepperCyclesPerTickPrescaled (uint32_t cycles_per_tick)
{
	uint32_t prescaler;
	// Compute step timing and timer prescalar for normal step generation.
	if (cycles_per_tick < (1UL << 16)) // < 65536  (4.1ms @ 16MHz)
		prescaler = step_prescaler[0]; // prescaler: 0
	else if (cycles_per_tick < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
		prescaler = step_prescaler[1]; // prescaler: 8
		cycles_per_tick = cycles_per_tick >> 3;
	} else {
		prescaler = step_prescaler[2]; // prescaler: 64
		cycles_per_tick = cycles_per_tick >> 6;
	}
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, prescaler);
    TimerLoadSet(TIMER1_BASE, TIMER_A, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFF /*Just set the slowest speed possible.*/);
}

// Set stepper pulse output pins, called from st_reset()
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z, needs to be mapped to physical pins by bit shifting or other means
//       other means may be by bit-banding or lookup table...
inline static void stepperSetStepOutputs (axes_signals_t step_outbits)
{
	GPIOPinWrite(STEP_PORT, HWSTEP_MASK, (step_outbits.value ^ step_port_invert_mask.value) << 1);
}

// Set stepper direction output pins, called from st_reset()
// NOTE1: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z, needs to be mapped to physical pins by bit shifting or other means
//        other means may be by bit-banding or lookup table...
inline static void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
	GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, (dir_outbits.value ^ dir_port_invert_mask.value) << 5);
}

// Sets stepper direction and pulse pins and starts a step pulse, called from stepper_driver_interrupt_handler()
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStart (axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm)
{
    stepperSetDirOutputs(dir_outbits);
	stepperSetStepOutputs(step_outbits);
	TimerEnable(TIMER2_BASE, TIMER_A);
}

static void stepperPulseStartDelayed (axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm)
{
    stepperSetDirOutputs(dir_outbits);
    next_step_outbits = step_outbits; // Store out_bits
    TimerEnable(TIMER2_BASE, TIMER_A);
}

// Disable limit pins interrupt, called from mc_homing_cycle()
static void limitsEnable (bool on)
{
    if (on && settings.flags.hard_limit_enable)
        GPIOIntEnable(LIMIT_PORT, HWLIMIT_MASK); // Enable Pin Change Interrupt
    else
        GPIOIntDisable(LIMIT_PORT, HWLIMIT_MASK); // Disable Pin Change Interrupt
}

// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
inline static axes_signals_t limitsGetState()
{
    axes_signals_t signals;

    signals.value = (uint8_t)(GPIOPinRead(LIMIT_PORT, HWLIMIT_MASK) >> 2);

	if (settings.limit_invert_mask.value)
	    signals.value ^= settings.limit_invert_mask.value;

	return signals;
}

inline static control_signals_t systemGetState (void)
{
    uint8_t flags = GPIOPinRead(CONTROL_PORT, HWCONTROL_MASK);
    control_signals_t signals = {0};

    if(flags & RESET_PIN)
        signals.reset = on;
    else if(flags & SAFETY_DOOR_PIN)
        signals.safety_door_ajar = on;
    else if(flags & FEED_HOLD_PIN)
        signals.feed_hold = on;
    else if(flags & CYCLE_START_PIN)
        signals.cycle_start = on;

    if(settings.control_invert_mask.value)
        signals.value ^= settings.control_invert_mask.value;

	return signals;
}

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigureInvertMask(bool is_probe_away)
{

  probe_invert_mask = settings.flags.invert_probe_pin ? 0 : PROBE_PIN;

  if (is_probe_away)
	  probe_invert_mask ^= PROBE_PIN;
}

// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
bool probeGetState (void)
{
    return (((uint8_t)GPIOPinRead(PROBE_PORT, PROBE_PIN)) ^ probe_invert_mask) != 0;
}

// Static spindle (off, on cw & on ccw)

inline static void spindleOff ()
{
    if(hal.driver_cap.spindle_dir)
        GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, settings.spindle_invert_mask.on ? SPINDLE_DIRECTION_PIN : 0);
    else
        GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN,  settings.spindle_invert_mask.on ? SPINDLE_ENABLE_PIN : 0);
}

inline static void spindleOn ()
{
    if(hal.driver_cap.spindle_dir)
        GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, settings.spindle_invert_mask.on ? 0 : SPINDLE_DIRECTION_PIN);
    else
        GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle_invert_mask.on ? 0 : SPINDLE_ENABLE_PIN );
}

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetState (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    if (!state.on)
        spindleOff();
    else {
        if(!hal.driver_cap.spindle_dir)
            GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, state.ccw ? SPINDLE_DIRECTION_PIN : 0);
    }
}

// Variable spindle

// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
static uint32_t spindleComputePWMValue (float rpm, uint8_t speed_ovr)
{
    uint32_t pwm_value;

    rpm *= (0.010f * speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_speed = settings.rpm_max;
        pwm_value = spindle_pwm.max_value - 1;
    } else if (rpm <= settings.rpm_min) {
        if (rpm == 0.0f) { // S0 disables spindle
            sys.spindle_speed = 0.0f;
            pwm_value = spindle_pwm.off_value;
        } else { // Set minimum PWM output
            sys.spindle_speed = settings.rpm_min;
            pwm_value = spindle_pwm.min_value;
        }
    } else {
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_speed = rpm;
        pwm_value = (uint32_t)floorf((rpm - settings.rpm_min) * spindle_pwm.pwm_gradient) + spindle_pwm.min_value;
        if(pwm_value >= spindle_pwm.max_value)
        	pwm_value = spindle_pwm.max_value - 1;
    }

    return pwm_value;
}

static uint32_t spindleSetSpeed (uint32_t pwm_value)
{

    if (pwm_value == hal.spindle_pwm_off) {
        pwmEnabled = false;
        if(settings.flags.spindle_disable_with_zero_speed)
            spindleOff();
        PWMGenDisable(SPINDLEPWM, SPINDLEPWM_GEN); // Disable PWM. Output voltage is zero.
     } else {
        if(!pwmEnabled)
            spindleOn();
        pwmEnabled = true;
        PWMGenPeriodSet(SPINDLEPWM, SPINDLEPWM_GEN, spindle_pwm.period);
        PWMPulseWidthSet(SPINDLEPWM, SPINDLEPWM_OUT, pwm_value);
        PWMGenEnable(SPINDLEPWM, SPINDLEPWM_GEN); // Ensure PWM output is enabled.
    }

    return pwm_value;
}

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetStateVariable (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    if (!state.on || rpm == 0.0f) {

        if(hal.driver_cap.spindle_dir)
            GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, settings.spindle_invert_mask.on ? SPINDLE_DIRECTION_PIN : 0);
        else
            GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN,  settings.spindle_invert_mask.on ? SPINDLE_ENABLE_PIN : 0);

        pwmEnabled = false;
        PWMGenDisable(SPINDLEPWM, SPINDLEPWM_GEN);

    } else {

        if(hal.driver_cap.spindle_dir)
            GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, settings.spindle_invert_mask.on ? 0 : SPINDLE_DIRECTION_PIN);
        else {
            GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, state.ccw ? SPINDLE_DIRECTION_PIN : 0);
            GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN,  settings.spindle_invert_mask.on ? 0 : SPINDLE_ENABLE_PIN);
        }
    }
}

static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    if(hal.driver_cap.spindle_dir) {
        if(GPIOPinRead(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) == settings.spindle_invert_mask.on ? 0 : SPINDLE_DIRECTION_PIN)
            state.on = on;
    } else if((GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) == settings.spindle_invert_mask.on ? 0 : SPINDLE_ENABLE_PIN) || pwmEnabled) {
        state.on = on;
        state.ccw = GPIOPinRead(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) == settings.spindle_invert_mask.ccw ? 0 : SPINDLE_DIRECTION_PIN;
    }

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled), called by coolant_run() and protocol_execute_realtime()
static void coolantSetState (coolant_state_t mode)
{

	if(mode.flood)
		GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, settings.coolant_invert_mask.flood ? 0 : COOLANT_FLOOD_PIN);
	else
		GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, settings.coolant_invert_mask.flood ? COOLANT_FLOOD_PIN : 0);

	if(mode.mist)
		GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, settings.coolant_invert_mask.mist ? 0 : COOLANT_MIST_PIN);
	else
		GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, settings.coolant_invert_mask.mist ? COOLANT_MIST_PIN : 0);

}

static coolant_state_t coolantGetState (void)
{
	coolant_state_t state = {0};

	state.flood = settings.coolant_invert_mask.flood
	               ? GPIOPinRead(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN) == 0
                   : GPIOPinRead(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN) != 0;

	state.mist  = settings.coolant_invert_mask.mist
	               ? GPIOPinRead(COOLANT_MIST_PORT, COOLANT_MIST_PIN) == 0
	               : GPIOPinRead(COOLANT_MIST_PORT, COOLANT_MIST_PIN) != 0;

    return state;

}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint8_t *ptr, uint8_t bits)
{
	IntMasterDisable();
	*ptr |= bits;
	IntMasterEnable();
}

static uint8_t bitsClearAtomic (volatile uint8_t *ptr, uint8_t bits)
{
    IntMasterDisable();
    uint8_t prev = *ptr;
	*ptr &= ~bits;
	IntMasterEnable();
	return prev;
}

static uint8_t valueSetAtomic (volatile uint8_t *ptr, uint8_t value)
{
    IntMasterDisable();
    uint8_t prev = *ptr;
    *ptr = value;
    IntMasterEnable();
    return prev;
}

// Callback to inform settings has been changed, called by settings_store_global_setting() and local driver_setup()
// Used here to set assorted helper variables
static void settings_changed (settings_t *settings)
{
/*
	if(bit_istrue(settings->flags, BITFLAG_HARD_LIMIT_ENABLE)) {
		GPIOIntClear(CONTROL_PORT, HWCONTROL_MASK);					// Clear any pending interrupt
		GPIOIntEnable(LIMIT_PORT, HWLIMIT_MASK);                   	// and enable Pin Change Interrupt
	} else
		GPIOIntDisable(LIMIT_PORT, HWLIMIT_MASK);						// Disable Pin Change Interrupt
*/
	// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.

    pulse_time = settings->pulse_microseconds;
    pulse_delay = settings->pulse_delay_microseconds;
    step_port_invert_mask = settings->step_invert_mask;
    dir_port_invert_mask = settings->dir_invert_mask;

    spindle_pwm.period = (uint32_t)(10000000 / settings->spindle_pwm_freq);
    spindle_pwm.off_value = (uint32_t)(spindle_pwm.period * settings->spindle_pwm_off_value / 100.0f);
    spindle_pwm.min_value = (uint32_t)(spindle_pwm.period * settings->spindle_pwm_min_value / 100.0f);
    spindle_pwm.max_value = (uint32_t)(spindle_pwm.period * settings->spindle_pwm_max_value / 100.0f);
    spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->rpm_max - settings->rpm_min);

    hal.spindle_pwm_off = spindle_pwm.off_value;

    if(IOInitDone) {

    	if(hal.driver_cap.variable_spindle)
    		PWMGenPeriodSet(SPINDLEPWM, SPINDLEPWM_GEN, spindle_pwm.period);


    }

    //    curr_dir_outbits = 0xFF; // "forget" current dir flags
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  // System init

#ifndef BACKCHANNEL
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
#endif
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    SysCtlDelay(26); // wait a bit for peripherals to wake up

    /******************
     *  Stepper init  *
     ******************/

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
	TimerIntClear(TIMER2_BASE, 0xFFFF);
	IntPendClear(INT_TIMER2A);
	TimerPrescaleSet(TIMER2_BASE, TIMER_A, 79); // for 1uS per count

	if(hal.driver_cap.step_pulse_delay) {
	    TimerIntRegister(TIMER2_BASE, TIMER_A, stepper_pulse_isr_delayed);
	    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);
	    hal.stepper_pulse_start = &stepperPulseStartDelayed;
	} else {
	    TimerIntRegister(TIMER2_BASE, TIMER_A, stepper_pulse_isr);
	    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	}

   /****************************
    *  Software debounce init  *
    ****************************/

	if(hal.driver_cap.software_debounce) {
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	    SysCtlDelay(26); // wait a bit for peripherals to wake up
	    IntPrioritySet(INT_TIMER3A, 0x40); // lower priority than for Timer2 (which resets the step-dir signal)
        TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
        TimerControlStall(TIMER3_BASE, TIMER_A, true); //timer2 will stall in debug mode
        TimerIntRegister(TIMER3_BASE, TIMER_A, software_debounce_isr);
        TimerIntClear(TIMER3_BASE, 0xFFFF);
        IntPendClear(INT_TIMER3A);
        TimerPrescaleSet(TIMER3_BASE, TIMER_A, 79); // configure for 1us per count
        TimerLoadSet(TIMER3_BASE, TIMER_A, 32000);  // and for a total of 32ms
        TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    }

   /***********************
    *  Control pins init  *
    ***********************/

	GPIOPinTypeGPIOInput(CONTROL_PORT, HWCONTROL_MASK);
    GPIOIntRegister(CONTROL_PORT, &control_isr);             // Register interrupt handler

    GPIOPadConfigSet(CONTROL_PORT, CYCLE_START_PIN, GPIO_STRENGTH_2MA, settings->control_disable_pullup_mask.cycle_start ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(CONTROL_PORT, FEED_HOLD_PIN, GPIO_STRENGTH_2MA, settings->control_disable_pullup_mask.feed_hold ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(CONTROL_PORT, RESET_PIN, GPIO_STRENGTH_2MA, settings->control_disable_pullup_mask.reset ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(CONTROL_PORT, SAFETY_DOOR_PIN, GPIO_STRENGTH_2MA, settings->control_disable_pullup_mask.safety_door_ajar ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(CONTROL_PORT, CYCLE_START_PIN, settings->control_invert_mask.cycle_start ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    GPIOIntTypeSet(CONTROL_PORT, FEED_HOLD_PIN, settings->control_invert_mask.feed_hold ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    GPIOIntTypeSet(CONTROL_PORT, RESET_PIN, settings->control_invert_mask.reset ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    GPIOIntTypeSet(CONTROL_PORT, SAFETY_DOOR_PIN, settings->control_invert_mask.safety_door_ajar ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);

    GPIOIntClear(CONTROL_PORT, HWCONTROL_MASK);     // Clear any pending interrupt
    GPIOIntEnable(CONTROL_PORT, HWCONTROL_MASK);    // and enable pin change interrupt

   /*********************
    *  Limit pins init  *
    *********************/

	GPIOPinTypeGPIOInput(LIMIT_PORT, HWLIMIT_MASK);
	GPIOIntRegister(LIMIT_PORT, hal.driver_cap.software_debounce ? limit_isr_debounced : limit_isr); // Register a call-back funcion for interrupt

	// Configure pullup/pulldown
    GPIOPadConfigSet(LIMIT_PORT, X_LIMIT_PIN, GPIO_STRENGTH_2MA, settings->limit_disable_pullup_mask.x ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(LIMIT_PORT, Y_LIMIT_PIN, GPIO_STRENGTH_2MA, settings->limit_disable_pullup_mask.y ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(LIMIT_PORT, Z_LIMIT_PIN, GPIO_STRENGTH_2MA, settings->limit_disable_pullup_mask.z ? GPIO_PIN_TYPE_STD_WPD : GPIO_PIN_TYPE_STD_WPU);

    // Configure interrupts
    GPIOIntTypeSet(LIMIT_PORT, X_LIMIT_PIN, settings->limit_invert_mask.x ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    GPIOIntTypeSet(LIMIT_PORT, Y_LIMIT_PIN, settings->limit_invert_mask.y ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    GPIOIntTypeSet(LIMIT_PORT, Z_LIMIT_PIN, settings->limit_invert_mask.z ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);

   /********************
    *  Probe pin init  *
    ********************/

	GPIOPinTypeGPIOInput(PROBE_PORT, PROBE_PIN);
    GPIOPadConfigSet(PROBE_PORT, PROBE_PIN, GPIO_STRENGTH_2MA, hal.driver_cap.probe_pull_up ? GPIO_PIN_TYPE_STD_WPU : GPIO_PIN_TYPE_STD_WPD);

   /***********************
    *  Coolant pins init  *
    ***********************/

    GPIOPinTypeGPIOOutput(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
    GPIOPinTypeGPIOOutput(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
    GPIOPadConfigSet(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(COOLANT_MIST_PORT, COOLANT_MIST_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    if(hal.driver_cap.amass_level == 0)
        hal.stepper_cycles_per_tick = &stepperCyclesPerTickPrescaled;

   /******************
    *  Spindle init  *
    ******************/

	GPIOPinTypeGPIOOutput(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
	GPIOPinTypeGPIOOutput(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);
    GPIOPadConfigSet(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

	if((hal.driver_cap.variable_spindle)) {
	    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
        SysCtlDelay(26); // wait a bit for peripherals to wake up
	    GPIOPinConfigure(SPINDLEPWM_MAP);
        GPIOPinTypePWM(SPINDLEPPORT, SPINDLEPPIN);
        GPIOPadConfigSet(SPINDLEPPORT, SPINDLEPPIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
        PWMGenConfigure(SPINDLEPWM, SPINDLEPWM_GEN, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_GEN_SYNC_LOCAL);
        PWMGenPeriodSet(SPINDLEPWM, SPINDLEPWM_GEN, spindle_pwm.period);
        PWMOutputState(SPINDLEPWM, SPINDLEPWM_OUT_BIT, true);
	} else
	    hal.spindle_set_status = &spindleSetState;

#ifdef HAS_KEYPAD

   /*********************
    *  I2C KeyPad init  *
	*********************/

	keypad_setup();

#endif

  // Set defaults

    setSerialReceiveCallback(hal.protocol_process_realtime);
    spindleSetState((spindle_state_t){0}, spindle_pwm.off_value, DEFAULT_SPINDLE_SPEED_OVERRIDE);
    coolantSetState((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    IOInitDone = settings->version == 12;

    return IOInitDone;

}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{

	// Set up systick time with a 1ms period
	SysTickPeriodSet((SysCtlClockGet() / 1000) - 1);
	SysTickIntRegister(systick_isr);
	SysTickIntEnable();
	SysTickEnable();

	// Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
	EEPROMInit();

	serialInit();

	hal.driver_setup = &driver_setup;
	hal.f_step_timer = 20000000;
	hal.rx_buffer_size = RX_BUFFER_SIZE;
	hal.delay_milliseconds = &driver_delay_ms;
    hal.settings_changed = &settings_changed;

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

	hal.spindle_set_status = &spindleSetStateVariable;
	hal.spindle_get_state = &spindleGetState;
	hal.spindle_set_speed = &spindleSetSpeed;
	hal.spindle_compute_pwm_value = &spindleComputePWMValue;

	hal.system_control_get_state = &systemGetState;

    hal.serial_read = &serialGetC;
    hal.serial_write = (void (*)(uint8_t))&serialPutC;
    hal.serial_write_string = &serialWriteS;
    hal.serial_get_rx_buffer_available = &serialRxFree;
    hal.serial_reset_read_buffer = &serialRxFlush;
    hal.serial_cancel_read_buffer = &serialRxCancel;

    hal.eeprom.type = EEPROM_Physical;
	hal.eeprom.get_byte = &eepromGetByte;
	hal.eeprom.put_byte = &eepromPutByte;
	hal.eeprom.memcpy_to_with_checksum = &eepromWriteBlockWithChecksum;
	hal.eeprom.memcpy_from_with_checksum = &eepromReadBlockWithChecksum;

	hal.set_bits_atomic = &bitsSetAtomic;
	hal.clear_bits_atomic = &bitsClearAtomic;
	hal.set_value_atomic = &valueSetAtomic;

    hal.userdefined_mcode_check = &userMCodeCheck;
    hal.userdefined_mcode_validate = &userMCodeValidate;
    hal.userdefined_mcode_execute = &userMCodeExecute;

#ifdef HAS_KEYPAD
    hal.execute_realtime = &process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.variable_spindle = on;
    hal.driver_cap.mist_control = on;
    hal.driver_cap.software_debounce = on;
    hal.driver_cap.step_pulse_delay = on;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = on;
    hal.driver_cap.limits_pull_up = on;
    hal.driver_cap.probe_pull_up = on;

    // no need to move version check before init - compiler will fail any mismatch for existing entries
	return hal.version == 3;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void)
{
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
static void stepper_pulse_isr (void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
	stepperSetStepOutputs(step_port_invert_mask);
}

static void stepper_pulse_isr_delayed (void)
{
    uint32_t iflags = TimerIntStatus(TIMER2_BASE, true);
    TimerIntClear(TIMER2_BASE, iflags); // clear interrupt flags
    stepperSetStepOutputs(iflags & TIMER_TIMA_MATCH ? next_step_outbits : step_port_invert_mask);
}

static void software_debounce_isr (void)
{
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

	axes_signals_t state = limitsGetState();

	if(state.value) //TODO: add check for limit swicthes having same state as when limit_isr were invoked?
		hal.limit_interrupt_callback(state);
}

static void limit_isr (void)
{
	uint32_t iflags = GPIOIntStatus(LIMIT_PORT, true) & HWLIMIT_MASK;

	if(iflags) {
		GPIOIntClear(LIMIT_PORT, iflags);
		hal.limit_interrupt_callback(limitsGetState());
	}
}

static void limit_isr_debounced (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT, true) & HWLIMIT_MASK;

    if(iflags) {
        GPIOIntClear(LIMIT_PORT, iflags);
        // TODO: disable interrups here and reenable in software_debounce_isr?
        TimerLoadSet(TIMER3_BASE, TIMER_A, 32000);  // 32ms
        TimerEnable(TIMER3_BASE, TIMER_A);
    }
}

static void control_isr (void)
{
// No debounce??
	uint8_t iflags = GPIOIntStatus(CONTROL_PORT, true) & HWCONTROL_MASK;

	if(iflags) {
		GPIOIntClear(CONTROL_PORT, iflags);
		hal.control_interrupt_callback(systemGetState());
	}
}

// Interrupt handler for 1 ms interval timer
static void systick_isr (void)
{
	if(!(--ms_count)) {
		SysTickDisable();
		if(delayCallback) {
			delayCallback();
			delayCallback = 0;
		}
	}
}

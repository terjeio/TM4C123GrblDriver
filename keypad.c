/*
  keypad.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of Grbl

  Copyright (c) 2017 Terje Io

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

#include "tiva.h"

#include "keypad.h"
#include "grbl.h"

#define KEYBUF_SIZE 16
#define i2cIsBusy (i2cBusy || I2CMasterBusy(I2C1_BASE))

static bool jogging = false, keyreleased = true;
static volatile bool i2cBusy = false;
static char keybuf_buf[16];
static jogmode_t jogMode = JogMode_Fast;
static volatile uint32_t keybuf_head = 0, keybuf_tail = 0;

static void I2C_interrupt_handler (void);
static void keyclick_int_handler (void);

void keypad_setup (void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

	I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
	I2CIntRegister(I2C1_BASE, I2C_interrupt_handler);

	GPIOPinTypeGPIOInput(KEYINTR_PORT, KEYINTR_PIN);
	GPIOPadConfigSet(KEYINTR_PORT, KEYINTR_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // -> WPU

	GPIOIntRegister(KEYINTR_PORT, keyclick_int_handler);
	GPIOIntTypeSet(KEYINTR_PORT, KEYINTR_PIN, GPIO_BOTH_EDGES);
	GPIOIntEnable(KEYINTR_PORT, KEYINTR_PIN);
}

// get single byte - via interrupt
static void I2C_GetKeycode (void)
{
   if(!i2cIsBusy) { // ignore if busy
   	i2cBusy = true;
       IntPendClear(INT_I2C1);
       I2CMasterIntClear(I2C1_BASE);
       I2CMasterSlaveAddrSet(I2C1_BASE, KEYPAD_I2CADDR, true);
       I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
       I2CMasterIntEnable(I2C1_BASE);
   }
}

static void enqueue_keycode (char cmd) {

    uint32_t bptr = (keybuf_head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf_tail) {                       // If not buffer full
        keybuf_buf[keybuf_head] = cmd;              // add data to buffer
        keybuf_head = bptr;                         // and update pointer
    }
}

// Returns 0 if no keycode enqueued
static char keypad_get_keycode (void) {

    uint32_t data = 0, bptr = keybuf_tail;

    if(bptr != keybuf_head) {
        data = keybuf_buf[bptr++];               // Get next character, increment tmp pointer
        keybuf_tail = bptr & (KEYBUF_SIZE - 1);  // and update pointer
    }

    return data;
}


// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3) {

	char tmp[30];
	char *s = strrchr(str, c);

	while(s) {
		strcpy(tmp, str3);
		strcat(tmp, s + 1);
		strcpy(s, tmp);
		s = strrchr(str, c);
	}

	return str;
}

void process_keypress (uint8_t state) {

	bool addedGcode, jogCommand = false;
	char command[30] = "", keycode = keypad_get_keycode();

	if(keycode)
	  switch(keycode) {

		case 'A':									// Mist override
			enqueue_accessory_ovr(CMD_COOLANT_MIST_OVR_TOGGLE);
			break;

		case 'E':									// Coolant override
			enqueue_accessory_ovr(CMD_COOLANT_FLOOD_OVR_TOGGLE);
			break;

		case 'h':									// "toggle" jog mode
			jogMode	= jogMode == JogMode_Step ? JogMode_Fast : (jogMode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
			break;

		case 'H':									// Home axes
			strcpy(command, "$H");
			break;

		case 'R':									// Jog X-axis right
			strcpy(command, "$J=G91X?F");
			break;

		case 'L':									// Jog X-axis left
			strcpy(command, "$J=G91X-?F");
			break;

		case 'F':									// Jog Y-axis forward
			strcpy(command, "$J=G91Y?F");
			break;

		case 'B':									// Jog Y-axis back
			strcpy(command, "$J=G91Y-?F");
			break;

		case 'q':									// Jog XY-axes SE
			strcpy(command, "$J=G91X?Y-?F");
			break;

		case 'r':									// Jog XY-axes NE
			strcpy(command, "$J=G91X?Y?F");
			break;

		case 's':									// Jog XY-axes NW
			strcpy(command, "$J=G91X-?Y?F");
			break;

		case 't':									// Jog XY-axes SW
			strcpy(command, "$J=G91X-?Y-?F");
			break;

		case 'U':									// Jog Z-axis up
			strcpy(command, "$J=G91Z?F");
			break;

		case 'D':									// Jog Z-axis down
			strcpy(command, "$J=G91Z-?F");
			break;

	}

	if(command[0] != '\0') {

		// add distance and speed to jog commands
		if((jogCommand = (command[0] == '$' && command[1] == 'J')))
			switch(jogMode) {

			case JogMode_Slow:
				strrepl(command, '?', "500");
				strcat(command, "100");
				break;

			case JogMode_Step:
				strrepl(command, '?', "0.25");
				strcat(command, "600");
				break;

			default:
				strrepl(command, '?', "500");
				strcat(command, "3000");
				break;

		}

		if(!(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!
			addedGcode = hal.protocol_enqueue_gcode(command);
			jogging = jogging || (jogCommand && addedGcode);
		}
	}

}

static void driver_keyclick_handler (bool keydown) {

	keyreleased = !keydown;

	if(keydown)
		I2C_GetKeycode();

	else if(jogging) {
		jogging = false;
		hal.protocol_process_realtime(CMD_JOG_CANCEL);
		keybuf_tail = keybuf_head = 0; // flush keycode buffer
	}
}

static void I2C_interrupt_handler (void)
{
	char keycode = I2CMasterIntStatusEx(I2C1_BASE, false);

	I2CMasterIntClear(I2C1_BASE);

	keycode = (uint8_t)I2CMasterDataGet(I2C1_BASE);

	I2CMasterIntDisable(I2C1_BASE);

	i2cBusy = false;

	if(GPIOIntStatus(KEYINTR_PORT, KEYINTR_PIN) == 0) // only add keycode when key is still pressed
		enqueue_keycode(keycode);
}

static void keyclick_int_handler (void) {

	uint32_t iflags = GPIOIntStatus(KEYINTR_PORT, KEYINTR_PIN);

    GPIOIntClear(KEYINTR_PORT, iflags);

	if(iflags & KEYINTR_PIN)
		driver_keyclick_handler(GPIOPinRead(KEYINTR_PORT, KEYINTR_PIN) != 0);
}

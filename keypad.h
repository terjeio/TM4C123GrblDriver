/*
  keypad.h - An embedded CNC Controller with rs274/ngc (g-code) support

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

#define KEYBUF_SIZE 16
#define KEYPAD_I2CADDR 0x49

#define KEYINTR_PIN   GPIO_PIN_4
#define KEYINTR_PORT  GPIO_PORTB_BASE

typedef enum {
	JogMode_Fast = 0,
	JogMode_Slow,
	JogMode_Step
} jogmode_t;

void keypad_setup (void);
void process_keypress (uint8_t state);

/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

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

// User defined mcode handling

#include "grbl.h"

#include "tiva.h"

//
// userMcodeCheck - return mcode if implemented, 0 otherwise
//

uint8_t userMCodeCheck (uint8_t mcode) {

    return mcode == 100 || mcode == 101 ? mcode : 0;

}

//
// userMCodeValidate - validate parameters, return STATUS_OK when ok
//

status_code_t userMCodeValidate (parser_block_t *gc_block, uint16_t *value_words) {

    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_defined_mcode) {

        case 100:
            if(bit_istrue(*value_words, bit(Word_P))) {
                state = Status_OK;
                bit_false(*value_words, bit(Word_P));
            }
            break;

        case 101:
            if(bit_istrue(*value_words, bit(Word_P))) {
                state = Status_OK;
                gc_block->user_defined_mcode_sync = true;
                bit_false(*value_words, bit(Word_P));
            }
            break;
    }

    return state;
}

//
// userMcodeExecute - execute user defined M-code
//

void userMCodeExecute (uint8_t state, parser_block_t *gc_block) {

    switch(gc_block->user_defined_mcode) {

        case 100:
            // do something
            break;

        case 101:
            // do something
            break;
    }
}

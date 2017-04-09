# TM4C123GrblDriver
A Grbl driver for Texas Instruments Tiva C Launchpad, for my HALified library fork of Grbl

This driver needs to be complemented with an UART or USB driver for host communication.

Features:

* All hardware dependent code, except host communication, in a single file.
* Entry point for executing G-code, bypasess serial input buffer - can be used for jogging etc.
* User defined M-code handler. _**WARNING:** Not tested._
* Improved jogging mode cancel, cancel will flush input buffer and stop any ongoing jog. Seems to work ok with 3-key rollover cursor key jogging implented in my G-code sender. Not tested in a real machine but I have so far been unable to fail jog cancel.
* Command \(CTRL-C\) for exiting Grbl.

**NOTE:** currently only tested on a launchpad where the motor step outputs are connected to the RGB led.

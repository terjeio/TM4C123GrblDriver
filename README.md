# TM4C123GrblDriver
A Grbl driver for Texas Instruments Tiva C Launchpad, for my HALified library fork of Grbl

This driver needs to be complemented with an UART or USB driver for host communication.

Optional support for I2C keypad added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode.

Features:

* All hardware dependent code, except host communication, in a separate project \(needs access to grbl library includes\).

**NOTE:** currently only tested on a launchpad where the motor step outputs are connected to the RGB led.

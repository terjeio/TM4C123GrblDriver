# TM4C123GrblDriver

---

#### This version is no longer maintained, newer releases can be found [here](https://github.com/terjeio/grblHAL).

---

A Grbl driver for Texas Instruments Tiva C Launchpad, for my [HALified fork of Grbl](https://github.com/terjeio/grbl).

This driver needs to be complemented with an UART or USB driver for host communication.

Optional support for I2C keypad added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode.

Features:

* All hardware dependent code, except host communication, in a separate project \(needs access to grbl library includes\).

**NOTE:** currently only tested on a launchpad where the motor step outputs are connected to the RGB led.

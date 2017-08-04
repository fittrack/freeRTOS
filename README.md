# FreeRTOS

This is Fittrack's fork of freeRTOS.

It includes some bugfixes:
* portable/GCC/ARM_CM0/port.c: Handle case when an interrupt is received at the wrong time and ulCompletedSysTickDecrements would become negative

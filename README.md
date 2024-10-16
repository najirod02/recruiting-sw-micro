# Recruiting-sw-micro

Repository for the MCU recruiting task of the Eagle team.

## Board pinout

For the board pinout of the __NUCLEO F411RE__, check this [link](https://os.mbed.com/platforms/ST-Nucleo-F411RE/).

For the project, the sensor uses the following pins:
- PA0 for analog read
- PC2 for digital read

## Plotting with python
Just run the script __plot.py__ to plot the data and read the stream.<br/>

You must install:
- matplotlib

Note that if the plot is closed, the entire script will end.

## Timer configuration
In order to setup the two timers used for warning and error state, the following formula has been applied:

$T_{OUT} = \dfrac{(ARR + 1)(PSC + 1)}{F_{CLK}} $

For more info, click the [link](https://deepbluembedded.com/stm32-timer-interrupt-hal-example-timer-mode-lab/).

## Linker file - syntax error

Using the STM32CubeMX software it is possible to setup the linker file.<br/>
With the latest version, a [bug](https://community.st.com/t5/stm32cubemx-mcus/flash-ld-syntax-error-when-upgrading-to-cubemx-v6-12-1/td-p/722343), generates an .ld file that contains some syntax error (lacks 'RAM' symbols).

There are 2 possible solutions:

- Instead of using Makefile as __toolchain__, use __STM32CubeIDE__ in order
to obtain the correct .ld file.

- Use __Makefile__ as __toolchain__ and then __manually add 'RAM'__ where needed.

### Notes for myself
The user button may trigger more than one interrupt with a single interaction.<br/>
The debouncing has been implemented but still, sometimes, the interrupt is fired more than one time.
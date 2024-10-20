# Recruiting-sw-micro

Repository for the MCU recruiting task of the Eagle team.

## Board pinout

For the board pinout of the __NUCLEO F411RE__, check this [link](https://os.mbed.com/platforms/ST-Nucleo-F411RE/).

For the project, the sensor uses the following pins:
- PA0 for analog read
- PC2 for digital read

## Plotting with python
First of all, you must install:

```bash
pip install matplotlib
pip install musicalbeeps
```

After that, connect your board to your device and then run
the Python script to plot the data, read the stream and send commands.<br/>

```bash
python3 plot.py
```

Note that if the plot is closed, the entire script will end.

More info about [musicalbeeps](https://pypi.org/project/musicalbeeps/).


## Timer configuration
To setup the two timers used for warning and error states, the following formula has been applied:

$T_{OUT} = \dfrac{(ARR + 1)(PSC + 1)}{F_{CLK}} $

For more info, click the [link](https://deepbluembedded.com/stm32-timer-interrupt-hal-example-timer-mode-lab/).

## Linker file - syntax error

Using the STM32CubeMX software it is possible to setup the linker file.<br/>
With the latest version, a [bug](https://community.st.com/t5/stm32cubemx-mcus/flash-ld-syntax-error-when-upgrading-to-cubemx-v6-12-1/td-p/722343), generates an .ld file that contains some syntax error (lacking 'RAM' symbols).

There are 2 possible solutions:

- Instead of using Makefile as __toolchain__, use __STM32CubeIDE__
to obtain the correct .ld file.

- Use __Makefile__ as __toolchain__ and then __manually add 'RAM'__ where needed.

## Possible issues
Here a list of issues that I have faced during the developing of this project:

- The user button may trigger more than one interrupt with a single interaction.<br/>
Although debouncing has been implemented, sometimes the interrupt is still fired more than once. The problem could be simply the button itself.

- The sound is played only for a short time and not continuosly. Increasing the duration generates delays in the reading of the data from the serial port and also on in updating the the plot.<br/>
Using a C library might improve the efficiency.
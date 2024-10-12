# Recruiting-sw-micro

Repository for the microcontroller recruiting task of the Eagle team.

## Board pinout

For the board pinout, check this [link](https://os.mbed.com/platforms/ST-Nucleo-F411RE/)

## Linker file - syntax error

Using the STM32CubeMX software it is possible to setup the linker file.<br/>
With the latest version, a [bug](https://community.st.com/t5/stm32cubemx-mcus/flash-ld-syntax-error-when-upgrading-to-cubemx-v6-12-1/td-p/722343), generates an .ld file that contains some syntax error (lacks some 'RAM' symbols).

There are 2 possible solutions:

- Instead of using Makefile as toolchain, use STM32CubeIDE in order
to obtain the correct .ld file.

- Use Makefile as toolchain and then manually add 'RAM' where needed.

## Notes for myself

At the moment, the DMA will store just one value before triggering the interrupt.<br/>
The last 150 elements will be saved in a local buffer and used whenever the user want to print the moving average.
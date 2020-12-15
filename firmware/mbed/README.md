## Building
To compile `nuice-example` run the following:
```bash
mbed compile -m NU32 -t GCC_ARM --source nuice-example --source mbed-os --build BUILD/nuice-example
```

When done compiling the bin file is located at `BUILD\nuice-example\nuice-example.bin`

## Upload
[STM32Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) is a useful tool, but `df-util` provides a simple CLI implementation aswell as DFU drivers.

```bash
dfu-util -a 0 -s 0x08000000:leave -D BUILD/nuice-example/nuice-example.bin 
```
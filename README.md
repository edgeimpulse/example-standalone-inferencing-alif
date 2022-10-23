# Edge Impulse Example: stand-alone inferencing (Alif)

This builds and runs an exported impulse locally on your machine. 

## Prerequisites
1. Create an edge impulse account at [edgeimpulse.com](https://www.edgeimpulse.com/)
2. Install and follow the [Quick Start Guide](https://alifsemi.com/download/AQSG0002) for the [Alif Security Toolkit - Version 0.54.0](https://alifsemi.com/kits/) to set up and enable programming via the SEUART interface.
3. Install either arm gcc or arm clang:

### gcc
Tested with:
```
gcc version 10.2.1 20201103 (release) (GNU Arm Embedded Toolchain 10-2020-q4-major)
```

### clang
[ARM clang compiler](https://developer.arm.com/tools-and-software/embedded/arm-compiler/downloads/version-6) and ensure it is added to your path:
```
which armclang
```

## Downloading your Edge Impulse model
Using an Edge Impulse project with the Alif beta enabled, navigate to the **Deployment** tab. Here you should see an `Ethos-U library` deployment option. Select this and click **build** to download a zip file of the contents. If your project does not have the ethos deployment option, contact [david@edgeimpulse.com](david@edgeimpulse.com)

Additionally, see the [image-example](https://github.com/edgeimpulse/example-standalone-inferencing-alif/tree/image-example) branch for an example of an already downloaded project.

## Add static features
Edge Impulse static examples show the minimal code to run inference, with no peripherals connected to provide data. Instead, we provide a static buffer of data obtained from a studio sample. Follow the steps described [here](https://docs.edgeimpulse.com/docs/running-your-impulse-locally-zephyr#running-the-impulse) to extract sample features from studio, and use these to populate the features array in `source/main.cpp`

For pre-configured example code with features and edge-impulse libraries already populated. See the [image-example](https://github.com/edgeimpulse/example-standalone-inferencing-alif/tree/image-example) branch of this repository.

## Build the firmware
1. Extract the zip file downloaded from edge impulse into the `source` directory of this repository
2. Choose one of the following:
    1. run cmake .. -DTARGET_SUBSYSTEM=HP to target Cortex M55 core 0 (high performance) OR
    2. run cmake .. -DTARGET_SUBSYSTEM=HE to target Cortex M55 core 1 (high efficiency)
3. If you wish to use gcc, add the cmake flag: -DCMAKE_TOOLCHAIN_FILE=../scripts/cmake/toolchains/bare-metal-gcc.cmake
4. armclang is the default toolchain file (-DCMAKE_TOOLCHAIN_FILE=../scripts/cmake/toolchains/bare-metal-armclang.cmake)
5. run make to build to app.axf
6. Example:
```
mkdir build
cd build
cmake .. -DTARGET_SUBSYSTEM=HP -DCMAKE_TOOLCHAIN_FILE=../scripts/cmake/toolchains/bare-metal-gcc.cmake
make -j8
```

## Flash your device

1. If using `armclang`, generate a .bin file from the compiled .axf via:

```
fromelf -v --bin --output bin/app.bin bin/app.axf
```

If using `gcc`, generate a .bin file from the compiled .axf via `objcopy`:
```
arm-none-eabi-objcopy -O binary bin/app.axf bin/app.bin
```

2. Navigate to your installed copy of the Alif Security Toolkit. 

3. Copy the `ei-app-cfg.json` from this repository to the `./build/config` directory of the toolkitcopy

4. Copy the generated `build/bin/app.bin` from this repository into the `./build/images` directory of the toolkit.

5. Run the commands to generate and flash the Alif device via SEUART:

```
app-gen-toc -f build/config/ei-app-cfg.json
app-write-mram
```

## Serial out is on UART2, which is connected to pins P3_16,P3_17 (on connector J413)

- Baud 115200
- "8,N,1" (the typical settings for everything else, 8 bit, no parity, 1 stop bit)
- 1.8 Vcc

# Other details

## Timing

Timing calculations are performed in [ei_classifier_porting.cpp](source/ei_classifier_porting.cpp) and make use of an interrupt attached to SysTick.
- An RTOS may take over this interrupt handler, in which case you should reimplement ei_read_timer_us and _ms.
- The default calculation is based on the default clock rates of the Alif dev kit (400 MHz for HP core, 160 MHz for HE core).  If you change this, redefine EI_CORE_CLOCK_HZ.

## Memory placement

Alif M55 processors have a private fast DTCM, and also access to a larger, but slower, chip global SRAM.
- For armclang the linker file attempts to place as much as possible in DTCM, and overflows into SRAM if needed.
- For gcc, the linker is unable to auto place based on size.  If you get an error during link, see [The linker file for GCC, arena placement](ensemble.ld#L116) and uncomment the line that places the arena in SRAM (line 116) (instead of DTCM).  This will only slow down DSP, as the U55 has to use the SRAM bus to access the model regardless of placement.

When your entire program can't fit into DTCM, sometimes customizing placement of objects can improve performance.
See [ensemble.sct](ensemble.sct) for example placement commands (currently commented out)

## Known issues

With debugger attached, my device boots up directly into Bus_Fault (or possibly another fault).  This can especially happen when you entered Hard Fault before your last reset.

- Power cycle your board and reload your program

Device hangs when using LOG_LEVEL_* (anything not ERROR)
- The ethos IRQ handler from ARM has logging statements.  The default logging calls into the UART driver, which cannot be called from an interrupt. (I suspect there is a deadlock issue). Either comment out these logging statements, set LOG_LEVEL_ERROR (the default), or redirect the logging statements to something interrupt safe.

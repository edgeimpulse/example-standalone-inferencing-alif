# Edge Impulse Example: stand-alone inferencing (Alif)

This builds and runs an exported impulse locally on your machine. 

## Prerequisites
1. Create an edge impulse account at [edgeimpulse.com](https://www.edgeimpulse.com/)
2. Install the [ARM clang compiler](https://developer.arm.com/tools-and-software/embedded/arm-compiler/downloads/version-6) and ensure it is added to your path:
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
    1. run cmake .. -DCORE=HP to target Cortex M55 core 0 (high performance) OR
    2. run cmake .. -DCORE=HE to target Cortex M55 core 1 (high efficiency) 
3. run make to build to app.axf
```
mkdir build
cd build
cmake .. -DCORE=HP
make -j8
```

## Flash your device
If you are using the Alif Dev kit, see Alif `AN0002` for instructions on flashing your Alif development kit with `build/bin/app.axf`

Once programmed, the firmware will run inference and print the results over the serial port.

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
- For gcc, the linker is unable to auto place based on size.  If you get an error during link, see [ensemble.ld](ensemble.ld) and uncomment the line that places the model in SRAM (instead of DTCM).  This will only slow down DSP, as the U55 has to use the SRAM bus to access the model regardless of placement.

When your entire program can't fit into DTCM, sometimes customizing placement of objects can improve performance.
See [ensemble.sct](ensemble.sct) for example placement commands (currently commented out)

## Known issues

With debugger attached, my device boots up directly into Bus_Fault (or possibly another fault).  This can especially happen when you entered Hard Fault before your last reset.

- Power cycle your board and reload your program


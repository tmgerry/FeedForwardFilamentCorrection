# Feed Forward Filament Diameter Correction

Colorado State University - Prawel Lab

## Software Installation Instructions - 

These instructions assume you have used [Klipper Installation And Update Helper](https://github.com/dw-0/kiauh) to install klipper.

1. Clone the repository -
```
git clone https://github.com/tmgerry/FeedForwardFilamentCorrection.git
```

1. Run the install tool and restart klipper - 
```
cd && cd CSU-in-situ/ && python3 klippy/installer.py && cd && sudo service klipper restart
```

1. Configure secondary MCU and add to config based on the example configuration file.

1. Put that file in the same folder as your `printer.cfg` file.

1. Include a line like the following at the top of your existing `printer.cfg` file to activate the diameter sensor. This requires hardware and config file to be setup correctly or klipper will not start.
```
[include diameter_sensor.cfg] # Comment out if no diameter sensor is connected to disable.
```

## Microcontroller Firmware Installation

This klipper add on requires a secondary microcontroller to communicate with the magnetic sensors over I2C. This microcontroller requires klipper firmware to be installed. Other microcontrollers, (especially Sparkfun QWIIC or Adafruit Stemma QT boards, which allow for solder free wiring) are most likely compatible assuming there is a klipper firmware build compatible with the specific chip they use. The Sparkfun RP2040 Pro Micro has been extensively tested and validated to be compatible.

### Adafruit and Sparkfun RP2040 Dev Boards

![Sparkfun RP2040 Pro Micro](https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/8/18288-SparkFun_Pro_Micro_-_RP2040-01.jpg)

Adafruit and Sparkfun use a different flash chip on their boards so the procedure will be the same as standard Raspberry Pi Picos except for minor setting changes in the config, and the fact that most of their boards have individual reset and boot-sel buttons.

1. Open terminal on raspberry pi with klipper installed
1. run in shell `cd klipper`
1. run in shell `make clean`
1. run in shell `make menuconfig` and the config menu will open, set the following options
   1. microcontroller architecture `Raspberry Pi RP2040`
   1. No bootloader offset
   1. Flash Chip - `GENERIC_03H with CLKDIV 4`
   1. Communication Interface - `USBSERIAL`
1. Press `Q` to save and exit
1. run in shell `make`, a bunch of lines saying "Compiling out/src......" will appear.
1. At the end you should see a line saying `Creating uf2 file out/klipper.uf2` This is the firmware file we want to upload to the pi.
1. Copy the file from above, to your computer. SFTP is convenient and easy for this.
1. Plug in the microcontroller to your computer while holding down the reset button, a removable usb-drive should appear in your file browser.
1. Drag the `klipper.uf2` file onto the picos removable drive, it will quickly disappear and restart.
1. Firmware is now flashed and the pico is ready to be used in klipper.


# Hardware

The sensor is based on the [InFiDEL Filament Diameter Sensor](https://www.printables.com/model/57154-infidel-inline-filament-diameter-estimator-lowcost), with modifications to improve accuracy and precision, as well as compatibility with TMAG5273 Hall Effect Sensor.

## Printed Parts

There are two printed parts in this sensor, they can be printed on a standard FDM Printer, but a resin printer yields higher precision parts. We print our sensors with Formlabs Tough 1500, but stiffer materials will likely yield even better results.

The two included .STL files in the CAD folder are the ones that need to be printed.

## General BOM

* 1x Sparkfun Pro Micro RP2040 ([Sparkfun](https://www.sparkfun.com/sparkfun-pro-micro-rp2040.html))
* 1x Sparkfun QWIIC Cable Set ([Sparkfun](https://www.sparkfun.com/sparkfun-qwiic-cable-kit.html))
* Superglue for attaching the TMAG sensor.


## Sensor BOM

* (1x) Sparkfun Mini Linear 3D Hall-Effect Sensor TMAG5273 ([Sparkfun](https://www.sparkfun.com/sparkfun-mini-linear-3d-hall-effect-sensor-tmag5273-qwiic.html))
* (1x) Spring ([McMASTER-CARR](https://www.mcmaster.com/9657K629/)) *similar springs can be sourced from other vendors but we have found them to be of unreliable stiffness*
* (1x) Magnet ([McMASTER-CARR](https://www.mcmaster.com/5862K138/)) *sensor geometry is finely tuned to magnet strength, ordering this specific part is highly advised.*
* (1x) Undersized 3mm Dowel Pin 16mm Long ([McMASTER-CARR](https://www.mcmaster.com/97049A329/)) *recommend McMASTER but can be sourced other places, ensure undersized to allow for easy assembly*
* (2x) Undersized 3mm Dowel Pin 25mm Long ([Amazon](https://www.amazon.com/uxcell-Stainless-Support-Fasten-Elements/dp/B07M63LMCM?s=industrial)) *Pins in this size were harder to source, Pins from Amazon worked fine but some were not within tolerance and could not be used. They can be ordered from misumi but costs significantly more.*
* (4x) 3mmx10mmx4mm Bearings ([Amazon](https://www.amazon.com/dp/B07FW389P1?_encoding=UTF8&ref_=cm_sw_r_ud_dp_GC7XPF5W3NQKGKW1RP3B&th=1))
* (1x) M6 x 1mm Thread, 8mm Long Set Screw ([McMASTER-CARR](https://www.mcmaster.com/92605A127/))
* (4x) M3 Screw to Expand Inserts. ([McMASTER-CARR](https://www.mcmaster.com/94510A030/)) *for rear sensor mounting holes, only necesarry if there is frequent assembly/dissassembly*



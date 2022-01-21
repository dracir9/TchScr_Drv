# Touch Screen Driver
Interface layer and I2C driver for custom resistive touch controller based on a EFM8BB microcontroller.

This ESP-IDF component implements all the functions to communicate with the EFM8BB microcontroller through I2C.
It allows sending calibration and configuration parameters as well as receiving touch events.

## Features
- Directly map touch screen coordinates to TFT coordinates
- Minimum and maximum touch pressure thresholds for reduced noise and false positives
- Up to 20 fully configurable buttons
  + Detect events inside a per-button configurable area
  + Press, release, and hold events per button
  + Per-button configurable hold timeout between 0 and 6.2 s (0.2 s resolution)
- Report touch (X, Y) coordinates

# 3.5" E-Paper New Board Porting Notes

If your display does not turn on after moving to a new board revision, start with the pin map and SPI wiring.

## 1) Update display pin mapping

The display control/SPI pins are now centralized in `EPD.h`:

- `EPD_PIN_CS`
- `EPD_PIN_DC`
- `EPD_PIN_RST`
- `EPD_PIN_BUSY`
- `EPD_PIN_SCK`
- `EPD_PIN_MOSI`

Change these macros to match your board's new wiring.

## 2) Verify GPIO init and SPI bus setup

`EPD_GPIOInit()` in `EPD.cpp` configures the control pins and calls:

```cpp
SPI.begin(EPD_PIN_SCK, -1, EPD_PIN_MOSI, EPD_PIN_CS);
```

If your board uses a different SPI bus/peripheral or needs an explicit `SPIClass`, this is the place to change it.

## 3) Verify low-level driver control signals

`Display_EPD_W21_spi.h` uses the pin macros above for:

- BUSY reads (`isEPD_W21_BUSY`)
- Reset toggles (`EPD_W21_RST_0` / `EPD_W21_RST_1`)
- Data/command mode (`EPD_W21_DC_0` / `EPD_W21_DC_1`)
- Chip select (`EPD_W21_CS_0` / `EPD_W21_CS_1`)

If your hardware is level-shifted/inverted, adjust these macros accordingly.

## 4) Confirm panel controller/resolution compatibility

This firmware expects a **184x384** panel and the command set in `Display_EPD_W21.cpp`.
If your new board uses a different panel/controller, you may need to update:

- `EPD_W` / `EPD_H` in `EPD.h`
- init/update command sequences in `Display_EPD_W21.cpp`

## 5) Quick sanity checks on hardware

- Ensure panel power rail is present/enabled on boot.
- Confirm BUSY pin really changes state (stuck BUSY will hang init loops).
- Confirm SCK/MOSI have activity during `EPD_Init()` calls.

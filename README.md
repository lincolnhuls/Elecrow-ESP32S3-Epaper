## ⚠️ Important Upload Notes

### 📁 Required Files
- **All files in this folder must be included** when uploading.
- This code **only works with the 3.7-inch e-paper display**.

---

### 🔧 Board & Flash Configuration

Use the following **exact** settings in the Arduino IDE or PlatformIO:

- **Board:** ESP32S3 Dev Module  
- **Flash Size:** 8MB (64Mb)  
- **Partition Scheme:** Huge APP (3MB No OTA / 1MB SPIFFS)  
- **PSRAM:** OPI PSRAM  
- **Upload Speed:** 460800  
  > ⚠️ Upload will fail if this is not set correctly

---

### Reseting the Board

To remove the device from its current adoption in the software use the following command
```json
{
  "secret":"yourSecret",
  "cmd":"deregister"
}
```

### Using the LEDs

- **On**
  - Defaults to white staying on until off command is received
```json
{
  "secret": "yourSecret",
  "cmd":"on",
  "red":yourRed,
  "green":yourGreen,
  "blue":yourBlue,
  "duration":yourDurationInSeconds,
  "brightness":yourBrightness,
  "start":firstPixelLedIndex,
  "count":howManyLedsInPixel
}
```

- **Off**
```json
{
  "secret": "yourSecret",
  "cmd":"off",
  "start":firstPixelLedIndex,
  "count":howManyLedsInPixel
}
```

- **Blink**
  - Defaults to white, blinking every 500ms until off command is received
```json
{
  "secret":"yourSecret",
  "cmd":"blink",
  "r":yourRed,
  "g":yourGreen,
  "b":yourBLue,
  "interval":blinkIntervalInMs,
  "times":numberOfBlinks,
  "brightness":yourBrightness,
  "start":firstPixelLedIndex,
  "count":howManyLedsInPixel
}
```

- **Breathe**
  - Defaults to white, breathing a full cycle every second until off command is received
```json
{
  "secret":"yourSecret",
  "cmd":"breathe",
  "r":yourRed,
  "g":yourGreen,
  "b":yourBLue,
  "duration":durationInSecondsFromHighToLow,
  "brightness":yourMaxBrightness,
  "start":firstPixelLedIndex,
  "count":howManyLedsInPixel
}
```


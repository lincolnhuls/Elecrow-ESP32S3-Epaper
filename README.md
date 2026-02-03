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

### Using the LEDs

- **JSON**
```json
{
  "secret": "yourSecret",
  "cmd":"yourLedCommand",
  "red":yourRed,
  "green":yourGreen,
  "blue":yourBlue,
  "duration":yourDurationInSeconds,
  "brightness":yourBrightness,
  "start":firstPixelLedIndex,
  "count":howManyLedsInPixel
}
```
- **Cmd Types** on, off

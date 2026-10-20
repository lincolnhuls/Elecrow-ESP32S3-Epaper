## ⚠️ Important Upload Notes

### 📁 Required Files
- **All files in this folder must be included** when uploading.
- This code is screen size specific, if the code for a different screen size is uploaded, it won't work. **Make sure you upload the right code for your screen size**.

---
### In the PubSubClient library, the PubSubClient.h file has a hardcoded buffer, #define MQTT_MAX_PACKET_SIZE, that needs to be expanded. 5000 should work.
---
### When flashing code you must enter in the network SSID and Password before you upload otherwise connection will fail. There is no external WiFi setup. If no broker host, username, or password is put in the device will try to connect to a broker at the local IP without a username or password.
---
### Device subscribes to warehouse/v1/gateway and its own command topic, light/{deviceUUID}/command. Once it receives the retained gateway announcement it takes the gatewayId and starts publishing heartbeats on 5 minute intervals.  
---

### 🔧 Board & Flash Configuration

Use the following **exact** settings in the Arduino IDE or PlatformIO:

- **Board:** ESP32S3 Dev Module  
- **Flash Size:** 8MB (64Mb)  
- **Partition Scheme:** "8M with spiffs (3MB APP/1.5MB SPIFFS)"  
- **PSRAM:** OPI PSRAM  
- **Upload Speed:** 460800  
  > ⚠️ Upload will fail if this is not set correctly


If using the new 3.5 inch board the upload scheme is different:

- **Board:** ESP32S3 Dev Module  
- **Flash Size:** 16MB (128Mb)  
- **Partition Scheme:** "8M with spiffs (3MB APP/1.5MB SPIFFS)"
- **Upload Speed:** 921600


### Removing the Device

To remove the device from its current adoption in the software use the following command
```json
{ "payload": { "cmd": "remove" } }
```

---

### Updating the Device

To use OTA updates for the device use the following command
```json
{ "payload": { "cmd": "update" } }
```

---

### Using the LEDs

- **LED Power Pin**
  > The LEDs have a power pin that can be used to set the power to the LEDs on or off. Every command that turns the LEDs on first sets the pin HIGH to allow power flow. The pin is toggled LOW to shut off the power only when the off command is sent with the "all" flag set to true

- **On**
  > Defaults to white staying on until off command is received. Duration -1 means on until cmd off or new command, set to an int for an auto shut off after that many seconds
```json
{ "payload": { "cmd": "on", "r": 255, "g": 255, "b": 255, "duration": -1, "brightness": 255, "ledList": [[0, 10]] } }
```

- **Breathe**
  > Defaults to white, breathing a full cycle every second until off command is received. Duration refers to the length in seconds from high to low.
```json
{ "payload": { "cmd": "breathe", "r": 255, "g": 255, "b": 255, "duration": 1, "brightness": 255, "ledList": [[10, 5], [30, 10]] } }
```

- **Blink**
  > Defaults to white, blinking every 500ms until off command is receive. If times is set to 0, it will blink until cmd off or new command. Set it to an int for an auto shut off after that many blinks.
```json
{ "payload": { "cmd": "blink", "r": 255, "g": 255, "b": 255, "interval": 500, "times": 0, "brightness": 255, "ledList": [[1, 1], [5, 3]] } }
```

- **Off**
```json
{ "payload": { "cmd": "off", "ledList": [[0, 10]] } }
```
  > Off also has an optional argument, "all", which when sent with the flag true acts as a global off, turning all leds off, no longer requiring a range.

  > For the LEDs power off pin, to toggle the pin to low and shut off power to the LEDs, you must use the all flag set to true. If the all flag is not used the pin will not be set to low  
```json
{ "payload": { "cmd": "off", "all": true } }
```




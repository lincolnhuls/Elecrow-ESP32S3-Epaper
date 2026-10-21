# ⚠️ Important Upload Notes

### 📁 Required Files

* Include **all files in the firmware folder** when uploading.
* The firmware is screen-size specific. Make sure you upload the correct version for your display.

### 📦 MQTT Packet Size

In `PubSubClient.h`, increase the hardcoded MQTT packet buffer size to `5000`:

```cpp
#define MQTT_MAX_PACKET_SIZE 5000
```

---

# 🔧 Board & Flash Configuration

Use the following settings in Arduino IDE or PlatformIO.

**Standard Board**

* **Board:** ESP32S3 Dev Module
* **Flash Size:** 8MB (64Mb)
* **Partition Scheme:** 8M with spiffs (3MB APP/1.5MB SPIFFS)
* **PSRAM:** OPI PSRAM
* **Upload Speed:** 460800

**New 3.5-Inch Board**

* **Board:** ESP32S3 Dev Module
* **Flash Size:** 16MB (128Mb)
* **Partition Scheme:** 8M with spiffs (3MB APP/1.5MB SPIFFS)
* **Upload Speed:** 921600

> ⚠️ **Important:** The upload speed must be configured correctly or uploading may fail.

---

# 🌐 Network & MQTT Configuration

### Wi-Fi

Enter your network SSID and password in the firmware **before uploading**. There is no external Wi-Fi setup.

### MQTT Broker

Configure the broker host, username, and password as needed. If these are not configured, the device attempts to connect to a broker at the local IP without authentication.

### Device Communication

The device subscribes to:

* `warehouse/v1/gateway` — Receives the retained gateway announcement.
* `light/{deviceUUID}/command` — Receives commands for the device.

After receiving the gateway announcement, the device extracts the `gatewayId` and begins publishing heartbeats every **5 minutes**.

---

# 💡 Using the LEDs

### On

Turns on the specified LEDs. Defaults to white and stays on until an `off` command or a new command is received. Set `duration` to `-1` for indefinite operation or an integer for automatic shutoff in seconds.

```json
{ "payload": { "cmd": "on", "r": 255, "g": 255, "b": 255, "duration": -1, "brightness": 255, "ledList": [[0, 10]] } }
```

### Breathe

Defaults to white and continuously breathes, completing a cycle every second. `duration` specifies the time in seconds from high to low brightness.

```json
{ "payload": { "cmd": "breathe", "r": 255, "g": 255, "b": 255, "duration": 1, "brightness": 255, "ledList": [[10, 5], [30, 10]] } }
```

### Blink

Defaults to white and blinks every 500 ms. Set `times` to `0` to blink indefinitely, or specify a number of blinks before automatically stopping.

```json
{ "payload": { "cmd": "blink", "r": 255, "g": 255, "b": 255, "interval": 500, "times": 0, "brightness": 255, "ledList": [[1, 1], [5, 3]] } }
```

### Off

Turns off the specified LED ranges.

```json
{ "payload": { "cmd": "off", "ledList": [[0, 10]] } }
```

**Global Off**

Set `"all": true` to turn off all LEDs and shut off LED power.

```json
{ "payload": { "cmd": "off", "all": true } }
```

> **LED Power:** Commands that turn on LEDs set the power pin `HIGH`. The pin is set `LOW` only when an `off` command includes `"all": true`. Turning off individual LED ranges does not cut LED power.

---

# 🖥️ Display Commands

### Set Status

Displays a custom status message, replacing any existing message. The text must contain at least one printable ASCII character (`0x20`–`0x7E`), with no newlines. The MQTT body is limited to 1024 bytes. Text wraps to fit the screen, and overflow is truncated with `...`. The status remains until cleared, the device is removed, or it reboots.

```json
{ "payload": { "cmd": "set_status", "text": "Wave 12 - 4 picks left" } }
```

### Clear Status

Removes the status message and restores the idle screen. Does nothing if no status is displayed.

```json
{ "payload": { "cmd": "clear_status" } }
```

---

# 🔄 Device Management

### Remove Device

Removes the device from its current adoption in the software.

```json
{ "payload": { "cmd": "remove" } }
```

### Update Firmware

Requests an OTA firmware update.

```json
{ "payload": { "cmd": "update" } }
```

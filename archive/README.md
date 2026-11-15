# Earlier versions

Kept for reference. Use `firmware/MMCv12ESP32` unless you have a reason not to.

| Sketch | Notes |
|---|---|
| `MMCv9proto`, `MMCv9M5stamp` | Original 2023 version for the prototype board and the M5Stamp. Fully blocking. |
| `MMCv9ESP32`, `MMCv91ESP32` | Same logic with active-low buttons and new pin assignments. |
| `MMCv10ESP32` | First attempt at non-blocking delays. The motor moves straight after the shutter fires; don't use. |
| `MMCv11ESP32` | State-machine version that v12 was rewritten from. The last frame of each stack gets no settle delay. |
| `MMCv11ESP32BLE` | Unfinished Bluetooth LE port. Does not compile. |

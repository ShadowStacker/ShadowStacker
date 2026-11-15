# ShadowStacker

Firmware for a motorised macro focus-stacking rig: an ESP32 turns the knob of a
focusing rail with a stepper motor and fires the camera between moves. You get a
series of frames, each focused a little deeper into the subject, ready to merge
in stacking software.

It also works as a plain intervalometer.

> **Status:** `firmware/MMCv12ESP32` is a rewrite of v11. It compiles cleanly
> and its logic has been checked in simulation, but at the time of writing it
> has not yet been run on the real hardware. The earlier sketches are kept in
> [`archive/`](archive/) if you need to fall back.

## What it does

- Shoot → wait → move → settle → shoot, for as many frames as you ask for
- Control from a USB serial terminal or a Bluetooth serial app on your phone
- Mark the near and far focus points and let it work out the number of frames
- Three buttons on the box: jog forward, jog back, fire the shutter
- Settings are remembered across power cycles
- Firmware updates over WiFi from a browser

## Hardware

- **ESP32 dev board** with classic Bluetooth (an original ESP32, not an S3 or C3)
- **Stepper motor and step/direction driver.** The firmware assumes 2038 steps
  per revolution; change `stepsPerRevolution` at the top of the sketch if yours
  differs.
- **Focusing rail** with the motor coupled to its knob. The firmware assumes one
  turn of the knob moves the carriage 4 mm (about 1.96 µm per step); change
  `mmPerStep` if yours differs.
- **Camera shutter release** triggered from a GPIO pin
- **Three pushbuttons** wired between the pin and ground (internal pull-ups are used)

### Pins

| Function | GPIO | Notes |
|---|---|---|
| Motor direction | 27 | LOW = clockwise |
| Motor step | 26 | |
| Motor enable | 25 | LOW = enabled |
| Shutter output | 21 | HIGH for 100 ms per shot |
| LED | 2 | On during a shot and in WiFi upload mode |
| Jog clockwise button | 19 | Press = connect to ground |
| Jog counterclockwise button | 18 | Press = connect to ground |
| Shutter button | 16 | Press = connect to ground |

All of these are constants at the top of the sketch.

## Installing

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and add the
   **esp32** board package (Boards Manager → search "esp32").
2. Open `firmware/MMCv12ESP32/MMCv12ESP32.ino`.
3. Select your ESP32 board and port, then upload. No extra libraries are needed.

The sketch is about 1.7 MB, which is too big for the default partition scheme.
Under **Tools → Partition Scheme** choose **Minimal SPIFFS (1.9MB APP with OTA)**.

### Ready-made binaries

If you'd rather not build it, [`firmware/bin/`](firmware/bin/) has v12 compiled
for a generic **ESP32 Dev Module** with 4 MB flash and the Minimal SPIFFS
partition scheme (arduino-esp32 core 3.3.12):

| File | Use |
|---|---|
| `ShadowStacker-v12-update.bin` | Upload through the WiFi update page (see below). Only works if the board was last flashed with a partition scheme whose app area is at least 1.7 MB. |
| `ShadowStacker-v12-full-flash.bin` | Complete image (bootloader, partition table and firmware) for flashing over USB at address `0x0`. Use this on a fresh board or if the WiFi update is refused. |

To flash the full image over USB with [esptool](https://github.com/espressif/esptool):

```bash
esptool.py --chip esp32 --baud 460800 write_flash 0x0 ShadowStacker-v12-full-flash.bin
```

Flashing the full image overwrites the whole flash, so saved settings go back
to their defaults.

## Connecting

- **USB:** open the serial monitor at **115200 baud**.
- **Bluetooth:** pair with **Macro ESP32**, then connect with any Bluetooth
  serial terminal app. The menu is printed when you connect.

Send **one command per line**: a letter, followed by a number where one is
needed, for example `m50`. Everything the controller says is sent to both USB
and Bluetooth.

## Quick start: your first stack

1. Set the camera to manual focus and manual exposure.
2. Set the step size, for example 50 microns: `e50`
3. Use the jog buttons (or `j200` / `j-200`) to bring the **nearest** part of
   the subject into focus. Send `a` to mark the start.
4. Jog until the **farthest** part is in focus. Send `b` to mark the end. The
   controller sets the direction and number of frames to cover that distance.
5. Send `g` to go back to the start.
6. Send `s` to start. Send `p` at any time to stop.

When it finishes, `g` takes you back to the start for another pass.

You can skip the marks and set things by hand instead: choose a direction with
`d`, the number of moves with `m`, then `s`.

## Commands

### Running

| Command | What it does |
|---|---|
| `s` | Start the stack |
| `p` | Stop immediately. Also leaves WiFi upload mode. |
| `i` | Take one shot now |
| `w` | Show position, marks and current state |
| `h` | Show the menu with all current settings |
| `x` | Restart the controller |

### Settings

| Command | Setting | Range | Default |
|---|---|---|---|
| `m50` | Number of moves. Frames taken = moves + 1. `m0` runs until you stop it. | 0 – 100000 | 1 |
| `q25` | Motor steps per move. `q0` = no movement (intervalometer). | 0 – 1000000 | 2038 |
| `e50` | Step size in microns, rounded to the nearest whole step | above 0 | – |
| `y900` | Wait after each shot, in ms. Make it longer than your exposure. | 0 – 3600000 | 900 |
| `t1000` | Wait after each move, in ms, so vibration dies down | 0 – 3600000 | 1000 |
| `d1` | Direction: `1` = clockwise, `0` = counterclockwise | 0 or 1 | 0 |
| `r8` | Motor speed in RPM | 0.1 – 15 | 8 |
| `k40` | Backlash take-up in steps (see below). `k0` = off. | 0 – 5000 | 0 |

Settings are saved as soon as you change them. They can't be changed while a
stack is running; send `p` first.

### Positioning

| Command | What it does |
|---|---|
| `j200` / `j-200` | Move a number of steps. Positive = clockwise. |
| `a` | Mark the start of the stack at the current position |
| `b` | Mark the end. Sets direction and number of moves from the distance and step size. |
| `g` | Go to the start mark |

Position is counted from wherever the rail was at power-up, and the marks are
forgotten when the controller restarts.

## Tips

**Choosing a step size.** Each step should be smaller than your depth of field
so neighbouring frames overlap. As a rough guide, at 1:1 magnification and f/8
the depth of field is around 0.6 mm; it shrinks quickly at higher magnification
and wider apertures. If the stacked result shows soft bands, use a smaller step.

**Delays.** `y` has to cover the whole exposure plus the time the camera needs
to write the file. With flash, also allow for recycle time. Raise `t` if frames
look shaken.

**Backlash.** Geared motors have slack that shows up when the direction
reverses: the motor turns a little before the carriage follows. To measure it,
jog one way, then count how many steps it takes going the other way before the
image starts to shift, and set `k` to a bit more than that. With `k` set, `g`
always arrives at the start travelling in the stack direction, and a stack that
starts after a reversal takes up the slack before its first frame.

**Travel limits.** There are no end stops. A long jog or stack can run the
carriage into the end of the rail, so check you have room before starting.

**Intervalometer.** `q0` turns off movement. The time between frames is then
`y` + `t` plus the 100 ms shutter pulse. Use `m0` to run until stopped.

**Microstepping drivers.** The motor is unpowered between moves to keep it cool
and quiet. If your driver microsteps, set `HOLD_TORQUE` to `true` in the sketch
so the rotor can't relax between frames.

## Updating firmware over WiFi

1. Get a `.bin` file: either `ShadowStacker-v12-update.bin` from `firmware/bin/`, or
   build your own with **Sketch → Export Compiled Binary** in the Arduino IDE.
2. Send `u`, then enter your WiFi name and password when asked (send `-` as the
   password for an open network).
3. The controller prints an address, a user name and a 6-digit password. Open
   the address in a browser on the same network and log in.
4. Choose the `.bin` file and press **Upload**. The controller restarts when it
   is done.

If it can't connect within 20 seconds it gives up and returns to normal. Send
`p` to leave upload mode without updating. The WiFi password is not stored.

## Troubleshooting

| Problem | Things to check |
|---|---|
| "Unknown command" | Send one command per line, with nothing after letter-only commands like `s` |
| "Stack is running" | Settings are locked during a run; send `p` first |
| Motor hums but doesn't turn | Lower the speed with `r`, and check the driver's current setting |
| Motor turns the wrong way | Use the other `d` value, or swap one motor coil pair |
| Camera doesn't fire | Try `i`. If the camera has gone to sleep, turn off its auto power-off. |
| First frames of a stack look identical | Backlash: set `k` |
| Can't find it over Bluetooth | The board must be an ESP32 with classic Bluetooth; re-pair with "Macro ESP32" |

## Repository layout

```
firmware/MMCv12ESP32/   current firmware
firmware/bin/           compiled binaries of the current firmware
archive/                earlier versions, kept for reference
```

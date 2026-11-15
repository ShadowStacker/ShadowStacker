# ShadowStacker

A motorised macro focus-stacking rig: an ESP32 turns the knob of a focusing
rail with a stepper motor and fires the camera between moves. You get a series
of frames, each focused a little deeper into the subject, ready to merge in
stacking software.

It is controlled from a phone app over Bluetooth LE, or from any serial
terminal. It also works as a plain intervalometer.

> **Status:** firmware v14 compiles cleanly and its logic, including the
> Bluetooth commands and the firmware-update protocol, has been checked in
> simulation. The app passes its analyzer and unit tests. Neither has yet been
> run on real hardware or a real phone at the time of writing, so expect to
> find things. The earlier sketches are in [`archive/`](archive/).

## What's here

| Folder | Contents |
|---|---|
| [`firmware/MMCv14ESP32/`](firmware/MMCv14ESP32/) | Current firmware (Arduino sketch) |
| [`firmware/bin/`](firmware/bin/) | The same firmware, compiled |
| [`app/`](app/) | Phone app for Android and iPhone (Flutter) |
| [`docs/BLE.md`](docs/BLE.md) | The Bluetooth protocol, for writing your own app |
| [`archive/`](archive/) | Earlier firmware versions |

## What it does

- Shoot → wait → move → settle → shoot, for as many frames as you ask for
- Mark the near and far focus points and let it work out the number of frames
- Hold-to-jog and live position in the app; three buttons on the box as well
- Settings are remembered across power cycles
- Firmware updates from the app over Bluetooth

## Hardware

- **ESP32 dev board.** Any ESP32 with Bluetooth LE.
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
| LED | 2 | On during a shot and during a firmware update |
| Jog clockwise button | 19 | Press = connect to ground |
| Jog counterclockwise button | 18 | Press = connect to ground |
| Shutter button | 16 | Press = connect to ground |

All of these are constants at the top of the sketch.

## Installing the firmware

### Ready-made binaries

[`firmware/bin/`](firmware/bin/) has v14 compiled for a generic **ESP32 Dev
Module** with 4 MB flash and the default partition scheme (arduino-esp32 core
3.3.12, NimBLE-Arduino 2.5.2).

| File | Use |
|---|---|
| `ShadowStacker-v14-update.bin` | For updating a controller that already runs ShadowStacker firmware: through the app (v14 and later), or through the WiFi update page of v11–v13. |
| `ShadowStacker-v14-full-flash.bin` | Complete image (bootloader, partition table and firmware) for flashing over USB at address `0x0`. Use this on a fresh board. Saved settings go back to their defaults. |

To flash the full image over USB with [esptool](https://github.com/espressif/esptool):

```bash
esptool.py --chip esp32 --baud 460800 write_flash 0x0 ShadowStacker-v14-full-flash.bin
```

### Building it yourself

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and add the
   **esp32** board package (Boards Manager → search "esp32").
2. Install the **NimBLE-Arduino** library (Library Manager → search "NimBLE-Arduino").
3. Open `firmware/MMCv14ESP32/MMCv14ESP32.ino`, select your board and port, and
   upload. The default partition scheme is fine; the firmware uses under half of it.

## The app

The app is in [`app/`](app/). It finds the controller, shows live position and
progress, and has four tabs:

- **Control** — start and stop, hold-to-jog, fine jog by steps, mark start and
  end, go to start, single shot
- **Settings** — step size, moves, delays, direction, speed, backlash, and a
  helper that suggests a step size from your sensor, magnification and aperture
- **Console** — everything the controller says, and a box to type raw commands
- **Update** — send new firmware

**Android:** every push that touches the app is built by GitHub Actions. Open
the repository's **Actions** tab, pick the latest **App** run, and download the
`ShadowStacker-android` artifact; it contains `app-release.apk`. Copy it to the
phone and open it to install (you will be asked to allow installing from that
source). The APK is signed with a debug key, which is fine for your own phone
but not for the Play Store.

**iPhone:** the project builds for iOS, but Apple requires a Mac with Xcode to
do it. Open `app/ios/Runner.xcworkspace`, set your signing team, and run it on
your phone. With a free Apple ID the install expires after 7 days.

To build either yourself, install [Flutter](https://docs.flutter.dev/get-started/install)
and run:

```bash
cd app
```

```bash
flutter build apk --release
```

Until the app is on your phone, any BLE terminal app that supports the Nordic
UART Service can send the text commands below.

## Quick start: your first stack

1. Set the camera to manual focus and manual exposure.
2. Connect with the app (or a terminal).
3. In **Settings**, set the step size, or use the helper at the bottom.
4. In **Control**, jog until the **nearest** part of the subject is in focus
   and press **Mark start**.
5. Jog until the **farthest** part is in focus and press **Mark end**. The
   controller sets the direction and number of frames to cover that distance.
6. Press **Go to start**, then **Start stack**. **Stop** works at any time.

When it finishes, **Go to start** takes you back for another pass.

The same thing from a terminal: `e50`, jog, `a`, jog, `b`, `g`, `s`.

## Commands

Send **one command per line** over USB serial (115200 baud) or the Bluetooth
UART service: a letter, followed by a number where one is needed, for example
`m50`. Replies go to both USB and Bluetooth.

### Running

| Command | What it does |
|---|---|
| `s` | Start the stack |
| `p` | Stop immediately. Also cancels a firmware update. |
| `i` | Take one shot now |
| `w` | Show position, marks and current state |
| `h` | Show the menu with all current settings |
| `x` | Restart the controller |
| `u` | Allow one firmware update in the next 60 seconds. USB only. |

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
stack is running; stop it first.

### Positioning

| Command | What it does |
|---|---|
| `j200` / `j-200` | Move a number of steps. Positive = clockwise. |
| `a` | Mark the start of the stack at the current position |
| `b` | Mark the end. Sets direction and number of moves from the distance and step size. |
| `g` | Go to the start mark |
| `f1` / `f-1` / `f0` | Hold-to-jog for apps; see [`docs/BLE.md`](docs/BLE.md) |

Position is counted from wherever the rail was at power-up, and the marks are
forgotten when the controller restarts.

## Updating firmware from the app

1. Get `ShadowStacker-v14-update.bin` (or a newer one) onto your phone.
2. In the app's **Update** tab, choose the file.
3. **Hold down both jog buttons on the controller** and press **Upload**. You
   can let go once it starts.
4. Keep the phone nearby. The controller checks the file and restarts.

The buttons are the lock: without them (or `u` over USB) the controller refuses,
so nobody in radio range can replace the firmware. If the update is interrupted
or the file is damaged, the controller keeps its current firmware.

## Tips

**Choosing a step size.** Each step should be smaller than your depth of field
so neighbouring frames overlap. As a rough guide, at 1:1 magnification and f/8
on APS-C the depth of field is around 0.6 mm; it shrinks quickly at higher
magnification and wider apertures. The app's helper does this sum for you. If
the stacked result shows soft bands, use a smaller step.

**Delays.** The wait after each shot has to cover the whole exposure plus the
time the camera needs to write the file. With flash, also allow for recycle
time. Raise the settle time if frames look shaken.

**Backlash.** Geared motors have slack that shows up when the direction
reverses: the motor turns a little before the carriage follows. To measure it,
jog one way, then count how many steps it takes going the other way before the
image starts to shift, and set the backlash take-up to a bit more than that.
With it set, going to the start always arrives travelling in the stack
direction, and a stack that starts after a reversal takes up the slack before
its first frame.

**Travel limits.** There are no end stops. A long jog or stack can run the
carriage into the end of the rail, so check you have room before starting.

**Intervalometer.** `q0` turns off movement. The time between frames is then
the two delays plus the 100 ms shutter pulse. Use `m0` to run until stopped.

**Microstepping drivers.** The motor is unpowered between moves to keep it cool
and quiet. If your driver microsteps, set `HOLD_TORQUE` to `true` in the sketch
so the rotor can't relax between frames.

**Who can connect.** Bluetooth control is not paired or encrypted: anyone
within range with the app can connect and move the rail. Only one phone can be
connected at a time.

## Troubleshooting

| Problem | Things to check |
|---|---|
| App doesn't find the controller | Bluetooth on, permission granted, controller powered. Another phone or terminal may already be connected. |
| "Unknown command" | Send one command per line, with nothing after letter-only commands like `s` |
| "Stack is running" | Settings are locked during a run; stop it first |
| Update refused | Both jog buttons must be held when you press Upload |
| Motor hums but doesn't turn | Lower the speed, and check the driver's current setting |
| Motor turns the wrong way | Use the other direction, or swap one motor coil pair |
| Camera doesn't fire | Try a single shot. If the camera has gone to sleep, turn off its auto power-off. |
| First frames of a stack look identical | Backlash: set the take-up |

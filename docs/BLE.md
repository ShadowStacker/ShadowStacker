# ShadowStacker Bluetooth LE protocol

For anyone writing their own app or script. Applies to firmware v13 and later;
the firmware update part needs v14.

The controller advertises as **ShadowStacker** and includes the Nordic UART
Service UUID in its advertisement. It accepts one connection at a time. There
is no pairing or encryption: anyone in range can connect and drive the rail.
Firmware updates need physical access (see below).

All multi-byte numbers are little-endian.

## Nordic UART Service: text commands

Service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`

| Characteristic | UUID | Properties | Use |
|---|---|---|---|
| RX | `6E400002-…` | write, write without response | Commands to the controller |
| TX | `6E400003-…` | notify | Replies from the controller |

Commands are the same text lines as over USB serial (see the main README), each
ended with a newline, for example `m50\n`. Replies are human-readable text and
may be split across several notifications; join them and split on newlines.

One command exists mainly for apps:

| Command | Meaning |
|---|---|
| `f1` / `f-1` | Jog clockwise / counterclockwise for as long as this keeps arriving |
| `f0` | Stop jogging |

`f1` and `f-1` must be re-sent at least once a second while a button is held
(the app sends every 300 ms). If they stop arriving, or the connection drops,
the motor stops by itself. There is no reply to `f`.

## ShadowStacker service: live state

Service `5353A000-7C1F-4B0E-9D5A-2F6C8E3B1A70`

### Status — `5353A001-…` (read, notify), 18 bytes

Notified whenever it changes, at most five times a second.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | State (see below) |
| 1 | u8 | Flags: bit 0 start mark set, bit 1 end mark set, bit 2 shutter output on |
| 2 | i32 | Position in steps from the power-up position, clockwise positive |
| 6 | u32 | Frames taken in the current or last stack |
| 10 | u32 | Total frames in the stack; 0 = runs until stopped |
| 14 | u32 | Estimated seconds left; 0 when not running |

States: 0 idle, 1 taking up backlash, 2 shutter firing, 3 waiting after shot,
4 moving to next frame, 5 settling, 6 moving (jog command or go-to-start),
7 jogging (button or hold-to-jog), 8 firmware update in progress. States 1–5
mean a stack is running.

### Settings — `5353A002-…` (read, notify), 48 bytes

Notified when any setting or mark changes. At a small MTU the notification is
truncated; read the characteristic to get all of it.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Record layout version (1) |
| 1 | u8 | Firmware version (13, 14, …) |
| 2 | u8 | Direction: 1 clockwise, 0 counterclockwise |
| 3 | u8 | Flags: bit 0 start mark set, bit 1 end mark set |
| 4 | f32 | Motor speed, RPM |
| 8 | u32 | Moves (frames = moves + 1); 0 = until stopped |
| 12 | u32 | Steps per move |
| 16 | u32 | Wait after shot, ms |
| 20 | u32 | Settle time after move, ms |
| 24 | u32 | Backlash take-up, steps |
| 28 | i32 | Start mark position |
| 32 | i32 | End mark position |
| 36 | u32 | Motor steps per revolution |
| 40 | f32 | Millimetres of rail travel per step |
| 44 | f32 | Maximum RPM |

Settings are changed with the text commands, not by writing this record.

## Firmware update (v14 and later)

| Characteristic | UUID | Properties |
|---|---|---|
| Control | `5353A003-…` | write, notify |
| Data | `5353A004-…` | write without response |

An update is only accepted if **both jog buttons are held down** on the
controller at the moment it begins, or within 60 seconds of sending `u` over
USB serial. Sending `u` over Bluetooth does not unlock it.

Sequence:

1. Subscribe to Control.
2. Write **begin** to Control: `01`, image size (u32), MD5 of the image (16 bytes).
3. Wait for the reply `01 result window(u16)`. Result 0 means go ahead.
4. Write the image to Data in packets of up to MTU − 3 bytes (244 at most).
   After every `window` bytes (4096), stop and wait for an acknowledgement
   `02 received(u32)` with `received` equal to the number of bytes sent so far.
   The last, partial window is acknowledged too.
5. Write **end** (`03`) to Control and wait for `03 result`. Result 0 means the
   image verified against the MD5 and the controller is restarting.

To cancel, write `04` to Control. The controller may notify `05 result` at any
time to say it has abandoned the update.

| Result | Meaning |
|---|---|
| 0 | OK |
| 1 | Not allowed: jog buttons not held |
| 2 | Busy: a stack or move is in progress |
| 3 | Image is empty or too big |
| 4 | Flash write or verification failed |
| 5 | Timed out (no data for 15 seconds) |
| 6 | Cancelled |

A failed or interrupted update leaves the existing firmware untouched.

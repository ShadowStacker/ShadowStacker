# ShadowStacker app

Flutter app for Android and iPhone that controls the ShadowStacker rail over
Bluetooth LE. See the [main README](../README.md) for how to use it.

```bash
flutter pub get
flutter test
flutter build apk --release
```

`lib/protocol.dart` is the Bluetooth protocol (documented in
[`docs/BLE.md`](../docs/BLE.md)), `lib/stacker.dart` is the connection, and
`lib/main.dart` is the screens.

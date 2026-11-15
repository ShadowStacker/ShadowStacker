// The Bluetooth LE protocol spoken by the ShadowStacker firmware (v13 and later).
// Pure Dart with no Flutter or plugin imports, so it can be unit tested.
// The firmware side is documented in docs/BLE.md.

import 'dart:convert';
import 'dart:typed_data';

class Uuids {
  // Nordic UART Service: text commands in, text replies out
  static const nusService = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
  static const nusRx = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
  static const nusTx = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
  // ShadowStacker service: binary status, settings and firmware update
  static const ssService = '5353a000-7c1f-4b0e-9d5a-2f6c8e3b1a70';
  static const ssStatus = '5353a001-7c1f-4b0e-9d5a-2f6c8e3b1a70';
  static const ssConfig = '5353a002-7c1f-4b0e-9d5a-2f6c8e3b1a70';
  static const ssOtaCtrl = '5353a003-7c1f-4b0e-9d5a-2f6c8e3b1a70';
  static const ssOtaData = '5353a004-7c1f-4b0e-9d5a-2f6c8e3b1a70';
}

enum StackerState {
  idle('Idle'),
  prep('Taking up backlash'),
  shooting('Taking a frame'),
  waitAfterShot('Waiting after shot'),
  moving('Moving to next frame'),
  settling('Settling'),
  manualMove('Moving'),
  jogging('Jogging'),
  updating('Updating firmware'),
  unknown('Unknown');

  const StackerState(this.label);
  final String label;
}

/// The 18-byte live status record.
class StackerStatus {
  const StackerStatus({
    required this.state,
    required this.hasStart,
    required this.hasEnd,
    required this.shutterActive,
    required this.position,
    required this.framesTaken,
    required this.framesTotal,
    required this.secondsLeft,
  });

  final StackerState state;
  final bool hasStart;
  final bool hasEnd;
  final bool shutterActive;

  /// Steps from where the rail was at power-up, clockwise positive.
  final int position;
  final int framesTaken;

  /// 0 means the stack runs until stopped.
  final int framesTotal;
  final int secondsLeft;

  bool get stackRunning =>
      state.index >= StackerState.prep.index &&
      state.index <= StackerState.settling.index;

  /// Anything is driving the motor or the controller is otherwise occupied.
  bool get busy => state != StackerState.idle;

  static StackerStatus? decode(List<int> bytes) {
    if (bytes.length < 18) return null;
    final d = ByteData.sublistView(Uint8List.fromList(bytes));
    final s = d.getUint8(0);
    final flags = d.getUint8(1);
    return StackerStatus(
      state: s < StackerState.unknown.index
          ? StackerState.values[s]
          : StackerState.unknown,
      hasStart: flags & 1 != 0,
      hasEnd: flags & 2 != 0,
      shutterActive: flags & 4 != 0,
      position: d.getInt32(2, Endian.little),
      framesTaken: d.getUint32(6, Endian.little),
      framesTotal: d.getUint32(10, Endian.little),
      secondsLeft: d.getUint32(14, Endian.little),
    );
  }
}

/// The 48-byte settings record.
class StackerConfig {
  const StackerConfig({
    required this.layout,
    required this.firmwareVersion,
    required this.clockwise,
    required this.hasStart,
    required this.hasEnd,
    required this.rpm,
    required this.moves,
    required this.stepsPerMove,
    required this.shotDelayMs,
    required this.settleDelayMs,
    required this.backlashSteps,
    required this.startMark,
    required this.endMark,
    required this.stepsPerRev,
    required this.mmPerStep,
    required this.maxRpm,
  });

  final int layout;
  final int firmwareVersion;
  final bool clockwise;
  final bool hasStart;
  final bool hasEnd;
  final double rpm;

  /// Number of moves; 0 means run until stopped.
  final int moves;
  final int stepsPerMove;
  final int shotDelayMs;
  final int settleDelayMs;
  final int backlashSteps;
  final int startMark;
  final int endMark;
  final int stepsPerRev;
  final double mmPerStep;
  final double maxRpm;

  double get stepMicrons => stepsPerMove * mmPerStep * 1000;
  int get frames => moves > 0 ? moves + 1 : 0;
  double get travelMm => moves * stepsPerMove * mmPerStep;

  /// How long a whole stack takes with these settings, mirroring the firmware's
  /// own estimate. Null when the stack runs until stopped.
  int? get estimatedSeconds {
    if (moves <= 0 || rpm <= 0) return null;
    const shutterPulseMs = 100;
    final moveMs = 60000.0 / rpm / stepsPerRev * stepsPerMove;
    final perFrame = moveMs + settleDelayMs + shutterPulseMs + shotDelayMs;
    return ((moves * perFrame + shutterPulseMs + shotDelayMs) / 1000).round();
  }

  static StackerConfig? decode(List<int> bytes) {
    if (bytes.length < 48) return null;
    final d = ByteData.sublistView(Uint8List.fromList(bytes));
    final flags = d.getUint8(3);
    return StackerConfig(
      layout: d.getUint8(0),
      firmwareVersion: d.getUint8(1),
      clockwise: d.getUint8(2) == 1,
      hasStart: flags & 1 != 0,
      hasEnd: flags & 2 != 0,
      rpm: d.getFloat32(4, Endian.little),
      moves: d.getUint32(8, Endian.little),
      stepsPerMove: d.getUint32(12, Endian.little),
      shotDelayMs: d.getUint32(16, Endian.little),
      settleDelayMs: d.getUint32(20, Endian.little),
      backlashSteps: d.getUint32(24, Endian.little),
      startMark: d.getInt32(28, Endian.little),
      endMark: d.getInt32(32, Endian.little),
      stepsPerRev: d.getUint32(36, Endian.little),
      mmPerStep: d.getFloat32(40, Endian.little),
      maxRpm: d.getFloat32(44, Endian.little),
    );
  }
}

/// Total depth of field in millimetres at a given magnification, using the
/// usual close-up approximation 2·N·c·(m+1)/m².
double depthOfFieldMm({
  required double magnification,
  required double aperture,
  required double circleOfConfusionMm,
}) {
  if (magnification <= 0) return 0;
  return 2 *
      aperture *
      circleOfConfusionMm *
      (magnification + 1) /
      (magnification * magnification);
}

/// A step size in microns that leaves the given overlap between neighbouring
/// frames (0.3 = 30% of the depth of field shared).
double suggestedStepMicrons(double depthOfFieldMm, {double overlap = 0.3}) =>
    depthOfFieldMm * 1000 * (1 - overlap);

String formatDuration(int seconds) {
  final h = seconds ~/ 3600;
  final m = (seconds % 3600) ~/ 60;
  final s = seconds % 60;
  final ss = s.toString().padLeft(2, '0');
  if (h > 0) return '$h:${m.toString().padLeft(2, '0')}:$ss';
  return '$m:$ss';
}

/// Turns the chunks that arrive over the UART characteristic back into lines.
class LineAssembler {
  final _pending = <int>[];

  List<String> add(List<int> chunk) {
    final lines = <String>[];
    for (final b in chunk) {
      if (b == 0x0a) {
        final text = utf8.decode(_pending, allowMalformed: true).trimRight();
        if (text.isNotEmpty) lines.add(text);
        _pending.clear();
      } else if (b != 0x0d) {
        _pending.add(b);
      }
    }
    return lines;
  }

  void clear() => _pending.clear();
}

// ---- Firmware update ----

class OtaMessage {
  static const begin = 1;
  static const ack = 2;
  static const end = 3;
  static const cancel = 4;
  static const failed = 5;
}

/// The "begin" control message: 01, image size, MD5 of the image.
Uint8List otaBeginMessage(int size, List<int> md5) {
  assert(md5.length == 16);
  final b = ByteData(21);
  b.setUint8(0, OtaMessage.begin);
  b.setUint32(1, size, Endian.little);
  final out = b.buffer.asUint8List();
  out.setRange(5, 21, md5);
  return out;
}

int readUint32(List<int> b, int offset) =>
    b[offset] | b[offset + 1] << 8 | b[offset + 2] << 16 | b[offset + 3] << 24;

String otaResultText(int code) => switch (code) {
      0 => 'OK',
      1 =>
        'The controller refused the update. Hold both jog buttons on the controller and try again.',
      2 => 'The controller is busy. Stop the stack and try again.',
      3 => 'That file is empty or too big for this controller.',
      4 =>
        'The file failed verification on the controller. Nothing was changed.',
      5 => 'The update timed out. Nothing was changed.',
      6 => 'The update was cancelled.',
      _ => 'The update failed (code $code).',
    };

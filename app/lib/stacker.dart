// Connection to one ShadowStacker controller. Owns the Bluetooth plumbing and
// exposes plain state for the screens to draw.

import 'dart:async';

import 'package:crypto/crypto.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import 'protocol.dart';

class Stacker extends ChangeNotifier {
  BluetoothDevice? device;
  bool connecting = false;
  String? lastError;

  StackerStatus? status;
  StackerConfig? config;

  /// Text the controller has sent, newest last.
  final List<String> log = [];

  /// 0..1 while a firmware update is being sent, otherwise null.
  double? updateProgress;

  BluetoothCharacteristic? _rx, _statusChar, _configChar, _otaCtrl, _otaData;
  final _subs = <StreamSubscription<dynamic>>[];
  final _lines = LineAssembler();
  Timer? _jogTimer;

  // Firmware update bookkeeping, fed by notifications on the control characteristic
  final _otaEvents = StreamController<List<int>>.broadcast();
  int _otaAcked = 0;
  List<int>? _otaReply;
  int? _otaFailure;

  bool get connected => device != null && _rx != null;
  bool get supportsUpdate => _otaCtrl != null && _otaData != null;
  bool get updating => updateProgress != null;
  String get name {
    final n = device?.platformName ?? '';
    return n.isEmpty ? 'ShadowStacker' : n;
  }

  Future<void> connect(BluetoothDevice d) async {
    if (connecting) return;
    connecting = true;
    lastError = null;
    notifyListeners();
    try {
      await d.connect(timeout: const Duration(seconds: 15));
      final services = await d.discoverServices();
      BluetoothCharacteristic? find(String uuid) {
        final guid = Guid(uuid);
        for (final s in services) {
          for (final c in s.characteristics) {
            if (c.uuid == guid) return c;
          }
        }
        return null;
      }

      final rx = find(Uuids.nusRx);
      final tx = find(Uuids.nusTx);
      if (rx == null || tx == null) {
        throw Exception('This device is not a ShadowStacker controller.');
      }
      _statusChar = find(Uuids.ssStatus);
      _configChar = find(Uuids.ssConfig);
      _otaCtrl = find(Uuids.ssOtaCtrl);
      _otaData = find(Uuids.ssOtaData);

      _subs.add(tx.onValueReceived.listen(_onText));
      await tx.setNotifyValue(true);

      final st = _statusChar;
      if (st != null) {
        _subs.add(st.onValueReceived.listen(_onStatus));
        await st.setNotifyValue(true);
        _onStatus(await st.read());
      }
      final cf = _configChar;
      if (cf != null) {
        _subs.add(cf.onValueReceived.listen(_onConfig));
        await cf.setNotifyValue(true);
        _onConfig(await cf.read());
      }
      final oc = _otaCtrl;
      if (oc != null) {
        _subs.add(oc.onValueReceived.listen(_onOtaControl));
        await oc.setNotifyValue(true);
      }

      _subs.add(d.connectionState.listen((s) {
        if (s == BluetoothConnectionState.disconnected) _onDisconnected();
      }));

      _rx = rx;
      device = d;
      log.clear();
    } catch (e) {
      lastError = _describe(e);
      await _teardown();
      try {
        await d.disconnect();
      } catch (_) {}
    } finally {
      connecting = false;
      notifyListeners();
    }
  }

  Future<void> disconnect() async {
    final d = device;
    jogStop();
    await _teardown();
    notifyListeners();
    try {
      await d?.disconnect();
    } catch (_) {}
  }

  void _onDisconnected() {
    if (device == null) return;
    lastError = 'Connection to the controller was lost.';
    _teardown();
    notifyListeners();
  }

  Future<void> _teardown() async {
    _jogTimer?.cancel();
    _jogTimer = null;
    for (final s in _subs) {
      s.cancel();
    }
    _subs.clear();
    _lines.clear();
    _rx = _statusChar = _configChar = _otaCtrl = _otaData = null;
    device = null;
    status = null;
    config = null;
    if (updateProgress != null) {
      updateProgress = null;
      _otaFailure = -1;
      _otaEvents.add(const []);
    }
  }

  void _onText(List<int> bytes) {
    final lines = _lines.add(bytes);
    if (lines.isEmpty) return;
    log.addAll(lines);
    if (log.length > 400) log.removeRange(0, log.length - 400);
    notifyListeners();
  }

  void _onStatus(List<int> bytes) {
    final s = StackerStatus.decode(bytes);
    if (s == null) return;
    status = s;
    notifyListeners();
  }

  Future<void> _onConfig(List<int> bytes) async {
    var data = bytes;
    // A notification can be cut short by a small packet size; a read never is.
    if (data.length < 48) {
      final c = _configChar;
      if (c == null) return;
      try {
        data = await c.read();
      } catch (_) {
        return;
      }
    }
    final cfg = StackerConfig.decode(data);
    if (cfg == null) return;
    config = cfg;
    notifyListeners();
  }

  /// Send one text command, exactly as you would type it in a terminal.
  Future<bool> send(String command, {bool echo = true}) async {
    final rx = _rx;
    if (rx == null) return false;
    if (echo) {
      log.add('> $command');
      notifyListeners();
    }
    try {
      await rx.write('$command\n'.codeUnits);
      return true;
    } catch (e) {
      lastError = _describe(e);
      notifyListeners();
      return false;
    }
  }

  /// Hold-to-jog. The controller stops by itself a second after the last
  /// refresh, so a dropped connection can never leave the motor running.
  void jogStart(int direction) {
    if (!connected || _jogTimer != null) return;
    send('f$direction', echo: false);
    _jogTimer = Timer.periodic(const Duration(milliseconds: 300), (_) {
      send('f$direction', echo: false);
    });
  }

  void jogStop() {
    if (_jogTimer == null) return;
    _jogTimer?.cancel();
    _jogTimer = null;
    send('f0', echo: false);
  }

  // ---- Firmware update ----

  void _onOtaControl(List<int> m) {
    if (m.isEmpty) return;
    switch (m[0]) {
      case OtaMessage.ack:
        if (m.length >= 5) _otaAcked = readUint32(m, 1);
      case OtaMessage.failed:
        _otaFailure = m.length > 1 ? m[1] : 4;
      default:
        _otaReply = m;
    }
    _otaEvents.add(m);
  }

  Future<void> _otaWait(bool Function() done, Duration timeout) async {
    final deadline = DateTime.now().add(timeout);
    while (!done()) {
      final failure = _otaFailure;
      if (failure != null) {
        throw Exception(failure < 0
            ? 'Connection to the controller was lost.'
            : otaResultText(failure));
      }
      final left = deadline.difference(DateTime.now());
      if (left.isNegative) {
        throw Exception('The controller stopped responding.');
      }
      try {
        await _otaEvents.stream.first.timeout(left);
      } on TimeoutException {
        throw Exception('The controller stopped responding.');
      }
    }
  }

  /// Send a firmware image. Throws with a readable message if it fails; the
  /// controller keeps its current firmware unless the whole image verifies.
  Future<void> updateFirmware(Uint8List image) async {
    final ctrl = _otaCtrl;
    final data = _otaData;
    final d = device;
    if (ctrl == null || data == null || d == null) {
      throw Exception(
          'This controller\'s firmware does not support updates over Bluetooth.');
    }
    if (updating) return;
    updateProgress = 0;
    notifyListeners();
    try {
      _otaAcked = 0;
      _otaReply = null;
      _otaFailure = null;
      final digest = md5.convert(image).bytes;

      await ctrl.write(otaBeginMessage(image.length, digest));
      await _otaWait(() => _otaReply != null, const Duration(seconds: 10));
      final begin = _otaReply!;
      if (begin[0] != OtaMessage.begin || begin.length < 4) {
        throw Exception('Unexpected reply from the controller.');
      }
      if (begin[1] != 0) throw Exception(otaResultText(begin[1]));
      final window = begin[2] | begin[3] << 8;

      // Leave room for the 3-byte packet header; 244 is the most the firmware takes.
      var chunk = d.mtuNow - 3;
      if (chunk > 244) chunk = 244;
      if (chunk < 20) chunk = 20;

      var sent = 0;
      while (sent < image.length) {
        final windowEnd =
            sent + window > image.length ? image.length : sent + window;
        while (sent < windowEnd) {
          final end = sent + chunk > windowEnd ? windowEnd : sent + chunk;
          await data.write(Uint8List.sublistView(image, sent, end),
              withoutResponse: true);
          sent = end;
        }
        final target = sent;
        await _otaWait(() => _otaAcked >= target, const Duration(seconds: 15));
        updateProgress = sent / image.length;
        notifyListeners();
      }

      _otaReply = null;
      await ctrl.write([OtaMessage.end]);
      await _otaWait(() => _otaReply != null, const Duration(seconds: 20));
      final end = _otaReply!;
      if (end[0] != OtaMessage.end || end.length < 2 || end[1] != 0) {
        throw Exception(otaResultText(end.length > 1 ? end[1] : 4));
      }
    } catch (e) {
      // Tell the controller to drop whatever it has, if it is still listening
      try {
        if (_otaCtrl != null && _otaFailure == null) {
          await _otaCtrl!.write([OtaMessage.cancel]);
        }
      } catch (_) {}
      throw Exception(_describe(e));
    } finally {
      updateProgress = null;
      notifyListeners();
    }
  }

  String _describe(Object e) {
    if (e is FlutterBluePlusException) {
      return e.description ?? 'Bluetooth error';
    }
    final text = e.toString();
    return text.startsWith('Exception: ') ? text.substring(11) : text;
  }
}

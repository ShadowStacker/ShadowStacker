import 'dart:async';
import 'dart:typed_data';

import 'package:file_picker/file_picker.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:material_ui/material_ui.dart';

import 'protocol.dart';
import 'stacker.dart';

void main() {
  runApp(ShadowStackerApp(stacker: Stacker()));
}

class ShadowStackerApp extends StatelessWidget {
  const ShadowStackerApp({super.key, required this.stacker});

  final Stacker stacker;

  @override
  Widget build(BuildContext context) {
    ThemeData theme(Brightness b) => ThemeData(
          colorScheme: ColorScheme.fromSeed(
              seedColor: const Color(0xFF3F6E8C), brightness: b),
          useMaterial3: true,
        );
    return MaterialApp(
      title: 'ShadowStacker',
      theme: theme(Brightness.light),
      darkTheme: theme(Brightness.dark),
      home: ListenableBuilder(
        listenable: stacker,
        builder: (context, _) => stacker.connected
            ? HomePage(stacker: stacker)
            : ScanPage(stacker: stacker),
      ),
    );
  }
}

// ---------------------------------------------------------------- Scan

class ScanPage extends StatefulWidget {
  const ScanPage({super.key, required this.stacker});

  final Stacker stacker;

  @override
  State<ScanPage> createState() => _ScanPageState();
}

class _ScanPageState extends State<ScanPage> {
  List<ScanResult> _results = [];
  bool _scanning = false;
  String? _error;
  BluetoothAdapterState _adapter = BluetoothAdapterState.unknown;
  final _subs = <StreamSubscription<dynamic>>[];

  @override
  void initState() {
    super.initState();
    _subs.add(FlutterBluePlus.adapterState.listen((s) {
      if (!mounted) return;
      setState(() => _adapter = s);
      if (s == BluetoothAdapterState.on && _results.isEmpty) _scan();
    }));
    _subs.add(FlutterBluePlus.scanResults.listen((r) {
      if (mounted) setState(() => _results = r);
    }));
    _subs.add(FlutterBluePlus.isScanning.listen((s) {
      if (mounted) setState(() => _scanning = s);
    }));
  }

  @override
  void dispose() {
    for (final s in _subs) {
      s.cancel();
    }
    FlutterBluePlus.stopScan();
    super.dispose();
  }

  Future<void> _scan() async {
    setState(() => _error = null);
    try {
      await FlutterBluePlus.startScan(
        withServices: [Guid(Uuids.nusService)],
        timeout: const Duration(seconds: 10),
      );
    } catch (e) {
      if (mounted) {
        setState(() => _error =
            'Could not scan. Check that Bluetooth is on and the app has permission to use it.');
      }
    }
  }

  Future<void> _connect(BluetoothDevice d) async {
    await FlutterBluePlus.stopScan();
    await widget.stacker.connect(d);
  }

  @override
  Widget build(BuildContext context) {
    final stacker = widget.stacker;
    final message = _error ?? stacker.lastError;
    Widget body;
    if (stacker.connecting) {
      body = const Center(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          CircularProgressIndicator(),
          SizedBox(height: 16),
          Text('Connecting…'),
        ]),
      );
    } else if (_adapter != BluetoothAdapterState.on &&
        _adapter != BluetoothAdapterState.unknown) {
      body = const _Hint(
        icon: Icons.bluetooth_disabled,
        text: 'Bluetooth is off. Turn it on to find your controller.',
      );
    } else if (_results.isEmpty) {
      body = _Hint(
        icon: Icons.bluetooth_searching,
        text: _scanning
            ? 'Looking for a ShadowStacker controller…'
            : 'No controller found. Make sure it is powered on and nearby, then scan again.',
      );
    } else {
      body = ListView(
        children: [
          for (final r in _results)
            ListTile(
              leading: const Icon(Icons.linear_scale),
              title: Text(r.device.platformName.isEmpty
                  ? 'ShadowStacker'
                  : r.device.platformName),
              subtitle: Text('Signal ${r.rssi} dBm'),
              trailing: const Icon(Icons.chevron_right),
              onTap: () => _connect(r.device),
            ),
        ],
      );
    }
    return Scaffold(
      appBar: AppBar(title: const Text('ShadowStacker')),
      body: Column(
        children: [
          if (_scanning) const LinearProgressIndicator(),
          if (message != null)
            Padding(
              padding: const EdgeInsets.all(16),
              child: Text(message,
                  style:
                      TextStyle(color: Theme.of(context).colorScheme.error)),
            ),
          Expanded(child: body),
        ],
      ),
      floatingActionButton: stacker.connecting || _scanning
          ? null
          : FloatingActionButton.extended(
              onPressed: _scan,
              icon: const Icon(Icons.refresh),
              label: const Text('Scan'),
            ),
    );
  }
}

class _Hint extends StatelessWidget {
  const _Hint({required this.icon, required this.text});

  final IconData icon;
  final String text;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(32),
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          Icon(icon, size: 56, color: Theme.of(context).colorScheme.outline),
          const SizedBox(height: 16),
          Text(text, textAlign: TextAlign.center),
        ]),
      ),
    );
  }
}

// ---------------------------------------------------------------- Home

class HomePage extends StatefulWidget {
  const HomePage({super.key, required this.stacker});

  final Stacker stacker;

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  int _tab = 0;

  @override
  Widget build(BuildContext context) {
    final stacker = widget.stacker;
    final pages = [
      ControlPage(stacker: stacker),
      SettingsPage(stacker: stacker),
      ConsolePage(stacker: stacker),
      UpdatePage(stacker: stacker),
    ];
    return Scaffold(
      appBar: AppBar(
        title: Text(stacker.name),
        actions: [
          IconButton(
            tooltip: 'Disconnect',
            icon: const Icon(Icons.link_off),
            onPressed: stacker.updating ? null : stacker.disconnect,
          ),
        ],
      ),
      body: SafeArea(child: pages[_tab]),
      bottomNavigationBar: NavigationBar(
        selectedIndex: _tab,
        onDestinationSelected: (i) {
          stacker.jogStop();
          setState(() => _tab = i);
        },
        destinations: const [
          NavigationDestination(icon: Icon(Icons.tune), label: 'Control'),
          NavigationDestination(icon: Icon(Icons.settings), label: 'Settings'),
          NavigationDestination(icon: Icon(Icons.terminal), label: 'Console'),
          NavigationDestination(
              icon: Icon(Icons.system_update_alt), label: 'Update'),
        ],
      ),
    );
  }
}

String _mm(num steps, StackerConfig? cfg) {
  if (cfg == null) return '$steps steps';
  final mm = steps * cfg.mmPerStep;
  return '${mm >= 0 ? '+' : ''}${mm.toStringAsFixed(3)} mm';
}

// ---------------------------------------------------------------- Control

class ControlPage extends StatelessWidget {
  const ControlPage({super.key, required this.stacker});

  final Stacker stacker;

  @override
  Widget build(BuildContext context) {
    final st = stacker.status;
    final cfg = stacker.config;
    final scheme = Theme.of(context).colorScheme;
    final text = Theme.of(context).textTheme;
    final running = st?.stackRunning ?? false;
    final busy = st?.busy ?? false;
    final hasStart = cfg?.hasStart ?? false;

    // Position is shown relative to the start mark when there is one
    String position = '—';
    if (st != null) {
      final rel = hasStart ? st.position - cfg!.startMark : st.position;
      position = '${_mm(rel, cfg)}  ·  $rel steps';
    }

    String plan = '';
    if (cfg != null) {
      if (cfg.stepsPerMove == 0) {
        plan = cfg.moves > 0
            ? '${cfg.frames} frames, no movement (intervalometer)'
            : 'Intervalometer, runs until stopped';
      } else if (cfg.moves > 0) {
        final secs = cfg.estimatedSeconds;
        plan = '${cfg.frames} frames  ·  ${cfg.stepMicrons.toStringAsFixed(0)} µm steps  ·  '
            '${cfg.travelMm.toStringAsFixed(2)} mm'
            '${secs != null ? '  ·  about ${formatDuration(secs)}' : ''}';
      } else {
        plan =
            '${cfg.stepMicrons.toStringAsFixed(0)} µm steps, runs until stopped';
      }
    }

    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(st?.state.label ?? 'Waiting for controller…',
                    style: text.titleLarge),
                const SizedBox(height: 4),
                Text(
                    '${hasStart ? 'From start mark' : 'Position'}: $position',
                    style: text.bodyMedium),
                if (running && st != null) ...[
                  const SizedBox(height: 12),
                  LinearProgressIndicator(
                    value: st.framesTotal > 0
                        ? st.framesTaken / st.framesTotal
                        : null,
                  ),
                  const SizedBox(height: 8),
                  Text(st.framesTotal > 0
                      ? 'Frame ${st.framesTaken} of ${st.framesTotal}  ·  ${formatDuration(st.secondsLeft)} left'
                      : 'Frame ${st.framesTaken}'),
                ] else if (plan.isNotEmpty) ...[
                  const SizedBox(height: 8),
                  Text(plan, style: text.bodySmall),
                ],
              ],
            ),
          ),
        ),
        const SizedBox(height: 12),
        Row(children: [
          Expanded(
            child: FilledButton.icon(
              onPressed: busy ? null : () => stacker.send('s'),
              icon: const Icon(Icons.play_arrow),
              label: const Text('Start stack'),
              style: FilledButton.styleFrom(
                  minimumSize: const Size.fromHeight(56)),
            ),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: FilledButton.icon(
              onPressed: () => stacker.send('p'),
              icon: const Icon(Icons.stop),
              label: const Text('Stop'),
              style: FilledButton.styleFrom(
                minimumSize: const Size.fromHeight(56),
                backgroundColor: scheme.error,
                foregroundColor: scheme.onError,
              ),
            ),
          ),
        ]),
        const SizedBox(height: 20),
        Text('Jog', style: text.titleMedium),
        const SizedBox(height: 8),
        Row(children: [
          Expanded(
            child: _HoldButton(
              icon: Icons.rotate_left,
              label: 'Hold  −',
              enabled: !busy || st?.state == StackerState.jogging,
              onDown: () => stacker.jogStart(-1),
              onUp: stacker.jogStop,
            ),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: _HoldButton(
              icon: Icons.rotate_right,
              label: 'Hold  +',
              enabled: !busy || st?.state == StackerState.jogging,
              onDown: () => stacker.jogStart(1),
              onUp: stacker.jogStop,
            ),
          ),
        ]),
        const SizedBox(height: 8),
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            for (final n in const [-100, -10, -1, 1, 10, 100])
              OutlinedButton(
                onPressed: busy ? null : () => stacker.send('j$n'),
                style: OutlinedButton.styleFrom(
                  padding: const EdgeInsets.symmetric(horizontal: 4),
                  minimumSize: const Size(48, 40),
                ),
                child: Text(n > 0 ? '+$n' : '$n'),
              ),
          ],
        ),
        const SizedBox(height: 4),
        Text('+ is clockwise, − is counterclockwise. Numbers are motor steps.',
            style: text.bodySmall),
        const SizedBox(height: 20),
        Text('Stack range', style: text.titleMedium),
        const SizedBox(height: 4),
        Text(
          !hasStart
              ? 'Focus on the nearest part of the subject and mark the start.'
              : !(cfg?.hasEnd ?? false)
                  ? 'Start marked. Now focus on the farthest part and mark the end.'
                  : 'Range set: ${_mm(cfg!.endMark - cfg.startMark, cfg)} from start to end.',
          style: text.bodySmall,
        ),
        const SizedBox(height: 8),
        Wrap(spacing: 8, runSpacing: 8, children: [
          FilledButton.tonal(
            onPressed: busy ? null : () => stacker.send('a'),
            child: const Text('Mark start'),
          ),
          FilledButton.tonal(
            onPressed: busy || !hasStart ? null : () => stacker.send('b'),
            child: const Text('Mark end'),
          ),
          FilledButton.tonal(
            onPressed: busy || !hasStart ? null : () => stacker.send('g'),
            child: const Text('Go to start'),
          ),
        ]),
        const SizedBox(height: 20),
        OutlinedButton.icon(
          onPressed: running ? null : () => stacker.send('i'),
          icon: const Icon(Icons.camera_alt_outlined),
          label: const Text('Take one shot'),
        ),
      ],
    );
  }
}

/// A button that acts for as long as it is held down.
class _HoldButton extends StatefulWidget {
  const _HoldButton({
    required this.icon,
    required this.label,
    required this.enabled,
    required this.onDown,
    required this.onUp,
  });

  final IconData icon;
  final String label;
  final bool enabled;
  final VoidCallback onDown;
  final VoidCallback onUp;

  @override
  State<_HoldButton> createState() => _HoldButtonState();
}

class _HoldButtonState extends State<_HoldButton> {
  bool _down = false;

  void _set(bool down) {
    if (down == _down) return;
    if (down && !widget.enabled) return;
    setState(() => _down = down);
    down ? widget.onDown() : widget.onUp();
  }

  @override
  void dispose() {
    if (_down) widget.onUp();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final fg = widget.enabled ? scheme.onSecondaryContainer : scheme.outline;
    return Listener(
      onPointerDown: (_) => _set(true),
      onPointerUp: (_) => _set(false),
      onPointerCancel: (_) => _set(false),
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 80),
        height: 72,
        decoration: BoxDecoration(
          color: _down
              ? scheme.primary.withValues(alpha: 0.35)
              : widget.enabled
                  ? scheme.secondaryContainer
                  : scheme.surfaceContainerHighest,
          borderRadius: BorderRadius.circular(16),
        ),
        child: Row(mainAxisAlignment: MainAxisAlignment.center, children: [
          Icon(widget.icon, color: fg),
          const SizedBox(width: 8),
          Text(widget.label,
              style: Theme.of(context)
                  .textTheme
                  .titleMedium
                  ?.copyWith(color: fg)),
        ]),
      ),
    );
  }
}

// ---------------------------------------------------------------- Settings

Future<String?> _askNumber(
  BuildContext context, {
  required String title,
  required String initial,
  String? suffix,
  String? help,
  bool decimal = false,
}) {
  final controller = TextEditingController(text: initial);
  controller.selection =
      TextSelection(baseOffset: 0, extentOffset: initial.length);
  return showDialog<String>(
    context: context,
    builder: (context) => AlertDialog(
      title: Text(title),
      content: Column(mainAxisSize: MainAxisSize.min, children: [
        if (help != null)
          Padding(
            padding: const EdgeInsets.only(bottom: 12),
            child: Text(help, style: Theme.of(context).textTheme.bodySmall),
          ),
        TextField(
          controller: controller,
          autofocus: true,
          keyboardType: TextInputType.numberWithOptions(decimal: decimal),
          decoration: InputDecoration(suffixText: suffix),
          onSubmitted: (v) => Navigator.pop(context, v.trim()),
        ),
      ]),
      actions: [
        TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel')),
        FilledButton(
            onPressed: () => Navigator.pop(context, controller.text.trim()),
            child: const Text('Set')),
      ],
    ),
  );
}

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key, required this.stacker});

  final Stacker stacker;

  @override
  State<SettingsPage> createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  // Depth-of-field helper inputs
  double _magnification = 1.0;
  double _aperture = 8.0;
  double _coc = 0.020;

  Future<void> _edit(String command, String title, String initial,
      {String? suffix, String? help, bool decimal = false}) async {
    final v = await _askNumber(context,
        title: title,
        initial: initial,
        suffix: suffix,
        help: help,
        decimal: decimal);
    if (v == null || v.isEmpty || num.tryParse(v) == null) return;
    await widget.stacker.send('$command$v');
  }

  @override
  Widget build(BuildContext context) {
    final stacker = widget.stacker;
    final cfg = stacker.config;
    final locked = stacker.status?.stackRunning ?? false;
    if (cfg == null) {
      return const _Hint(
          icon: Icons.settings, text: 'Waiting for the controller…');
    }
    final text = Theme.of(context).textTheme;
    final dof = depthOfFieldMm(
        magnification: _magnification,
        aperture: _aperture,
        circleOfConfusionMm: _coc);
    final suggested = suggestedStepMicrons(dof);

    Widget tile(String title, String value, VoidCallback onTap) => ListTile(
          title: Text(title),
          trailing: Text(value, style: text.bodyLarge),
          enabled: !locked,
          onTap: onTap,
        );

    return ListView(
      children: [
        if (locked)
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 16, 16, 0),
            child: Text('Settings are locked while a stack is running.',
                style: TextStyle(color: Theme.of(context).colorScheme.error)),
          ),
        tile(
          'Step size',
          '${cfg.stepMicrons.toStringAsFixed(1)} µm  (${cfg.stepsPerMove} steps)',
          () => _edit('e', 'Step size', cfg.stepMicrons.toStringAsFixed(0),
              suffix: 'µm',
              decimal: true,
              help:
                  'Distance the rail moves between frames. It is rounded to whole motor steps of ${(cfg.mmPerStep * 1000).toStringAsFixed(2)} µm.'),
        ),
        tile(
          'Moves',
          cfg.moves > 0
              ? '${cfg.moves}  (${cfg.frames} frames)'
              : 'Until stopped',
          () => _edit('m', 'Number of moves', '${cfg.moves}',
              help:
                  'Frames taken = moves + 1. Use 0 to run until you press Stop. Marking a start and end sets this for you.'),
        ),
        tile(
          'Wait after shot',
          '${cfg.shotDelayMs} ms',
          () => _edit('y', 'Wait after shot', '${cfg.shotDelayMs}',
              suffix: 'ms',
              help:
                  'Must cover the whole exposure plus the time the camera needs to save the frame.'),
        ),
        tile(
          'Settle time after move',
          '${cfg.settleDelayMs} ms',
          () => _edit('t', 'Settle time after move', '${cfg.settleDelayMs}',
              suffix: 'ms',
              help: 'Lets vibration die down before the next frame.'),
        ),
        ListTile(
          title: const Text('Direction'),
          enabled: !locked,
          trailing: SegmentedButton<bool>(
            segments: const [
              ButtonSegment(value: false, label: Text('CCW')),
              ButtonSegment(value: true, label: Text('CW')),
            ],
            selected: {cfg.clockwise},
            onSelectionChanged: locked
                ? null
                : (s) => stacker.send('d${s.first ? 1 : 0}'),
          ),
        ),
        tile(
          'Motor speed',
          '${cfg.rpm.toStringAsFixed(1)} RPM',
          () => _edit('r', 'Motor speed', cfg.rpm.toStringAsFixed(1),
              suffix: 'RPM',
              decimal: true,
              help: '0.1 to ${cfg.maxRpm.toStringAsFixed(0)} RPM.'),
        ),
        tile(
          'Backlash take-up',
          cfg.backlashSteps > 0 ? '${cfg.backlashSteps} steps' : 'Off',
          () => _edit('k', 'Backlash take-up', '${cfg.backlashSteps}',
              suffix: 'steps',
              help:
                  'Steps of gear slack to take up when the motor reverses. 0 turns it off.'),
        ),
        const Divider(),
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 8, 16, 0),
          child: Text('Step size helper', style: text.titleMedium),
        ),
        ListTile(
          title: const Text('Sensor'),
          trailing: DropdownButton<double>(
            value: _coc,
            onChanged: (v) => setState(() => _coc = v ?? _coc),
            items: const [
              DropdownMenuItem(value: 0.030, child: Text('Full frame')),
              DropdownMenuItem(value: 0.020, child: Text('APS-C')),
              DropdownMenuItem(value: 0.015, child: Text('Micro 4/3')),
            ],
          ),
        ),
        ListTile(
          title: const Text('Magnification'),
          trailing:
              Text('${_magnification.toStringAsFixed(2)}×', style: text.bodyLarge),
          onTap: () async {
            final v = await _askNumber(context,
                title: 'Magnification',
                initial: _magnification.toStringAsFixed(2),
                suffix: '×',
                decimal: true,
                help: '1.0 means life size (1:1).');
            final n = double.tryParse(v ?? '');
            if (n != null && n > 0) setState(() => _magnification = n);
          },
        ),
        ListTile(
          title: const Text('Aperture'),
          trailing:
              Text('f/${_aperture.toStringAsFixed(1)}', style: text.bodyLarge),
          onTap: () async {
            final v = await _askNumber(context,
                title: 'Aperture',
                initial: _aperture.toStringAsFixed(1),
                suffix: 'f-number',
                decimal: true);
            final n = double.tryParse(v ?? '');
            if (n != null && n > 0) setState(() => _aperture = n);
          },
        ),
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 4, 16, 24),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                  'Depth of field is about ${(dof * 1000).toStringAsFixed(0)} µm. '
                  'A step of ${suggested.toStringAsFixed(0)} µm overlaps neighbouring frames by 30%.'),
              const SizedBox(height: 8),
              FilledButton.tonal(
                onPressed: locked || suggested <= 0
                    ? null
                    : () => stacker.send('e${suggested.toStringAsFixed(1)}'),
                child: Text('Use ${suggested.toStringAsFixed(0)} µm'),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

// ---------------------------------------------------------------- Console

class ConsolePage extends StatefulWidget {
  const ConsolePage({super.key, required this.stacker});

  final Stacker stacker;

  @override
  State<ConsolePage> createState() => _ConsolePageState();
}

class _ConsolePageState extends State<ConsolePage> {
  final _input = TextEditingController();
  final _scroll = ScrollController();

  void _send() {
    final t = _input.text.trim();
    if (t.isEmpty) return;
    widget.stacker.send(t);
    _input.clear();
  }

  @override
  void dispose() {
    _input.dispose();
    _scroll.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final log = widget.stacker.log;
    return Column(
      children: [
        Expanded(
          child: log.isEmpty
              ? const _Hint(
                  icon: Icons.terminal,
                  text:
                      'Messages from the controller appear here. Send h for the list of commands.')
              : ListView.builder(
                  controller: _scroll,
                  reverse: true,
                  padding: const EdgeInsets.all(12),
                  itemCount: log.length,
                  itemBuilder: (context, i) {
                    final line = log[log.length - 1 - i];
                    return Text(
                      line,
                      style: TextStyle(
                        fontFamily: 'monospace',
                        fontSize: 13,
                        color: line.startsWith('> ')
                            ? Theme.of(context).colorScheme.primary
                            : null,
                      ),
                    );
                  },
                ),
        ),
        Padding(
          padding: const EdgeInsets.fromLTRB(12, 0, 12, 12),
          child: Row(children: [
            Expanded(
              child: TextField(
                controller: _input,
                autocorrect: false,
                decoration: const InputDecoration(
                    hintText: 'Command, e.g. m50', isDense: true),
                onSubmitted: (_) => _send(),
              ),
            ),
            IconButton(onPressed: _send, icon: const Icon(Icons.send)),
          ]),
        ),
      ],
    );
  }
}

// ---------------------------------------------------------------- Update

class UpdatePage extends StatefulWidget {
  const UpdatePage({super.key, required this.stacker});

  final Stacker stacker;

  @override
  State<UpdatePage> createState() => _UpdatePageState();
}

class _UpdatePageState extends State<UpdatePage> {
  String? _fileName;
  Uint8List? _bytes;
  String? _message;
  bool _failed = false;

  Future<void> _pick() async {
    try {
      final files = await FilePicker.pickFiles(type: FileType.any);
      if (files.isEmpty) return;
      final bytes = await files.first.readAsBytes();
      if (!mounted) return;
      setState(() {
        _fileName = files.first.name;
        _bytes = bytes;
        _message = null;
      });
    } catch (e) {
      if (mounted) {
        setState(() {
          _failed = true;
          _message = 'Could not read that file.';
        });
      }
    }
  }

  Future<void> _upload() async {
    final bytes = _bytes;
    if (bytes == null) return;
    setState(() => _message = null);
    try {
      // Every ESP32 firmware image starts with this byte
      if (bytes.isEmpty || bytes[0] != 0xE9) {
        throw Exception(
            'That does not look like an ESP32 firmware file. Choose the “update” .bin, not the “full-flash” one.');
      }
      await widget.stacker.updateFirmware(bytes);
      if (!mounted) return;
      setState(() {
        _failed = false;
        _message =
            'Update sent and verified. The controller is restarting; reconnect in a few seconds.';
        _fileName = null;
        _bytes = null;
      });
    } catch (e) {
      if (!mounted) return;
      final t = e.toString();
      setState(() {
        _failed = true;
        _message = t.startsWith('Exception: ') ? t.substring(11) : t;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    final stacker = widget.stacker;
    final cfg = stacker.config;
    final text = Theme.of(context).textTheme;
    final progress = stacker.updateProgress;
    final busy = stacker.status?.busy ?? false;
    final bytes = _bytes;
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        Text('Firmware', style: text.titleMedium),
        const SizedBox(height: 4),
        Text(cfg == null
            ? 'Version unknown'
            : 'This controller is running version ${cfg.firmwareVersion}.'),
        const SizedBox(height: 16),
        if (!stacker.supportsUpdate)
          const Text(
              'This firmware version cannot be updated over Bluetooth. Install version 14 or later once over USB (or through the WiFi update page on version 13).')
        else ...[
          const Text('1.  Choose the firmware “update” .bin file.'),
          const SizedBox(height: 8),
          OutlinedButton.icon(
            onPressed: progress != null ? null : _pick,
            icon: const Icon(Icons.folder_open),
            label: Text(bytes == null
                ? 'Choose file'
                : '$_fileName  (${(bytes.length / 1024).toStringAsFixed(0)} KB)'),
          ),
          const SizedBox(height: 16),
          const Text(
              '2.  Hold down both jog buttons on the controller, then press Upload. You can let go once it starts.'),
          const SizedBox(height: 8),
          FilledButton.icon(
            onPressed:
                bytes == null || progress != null || busy ? null : _upload,
            icon: const Icon(Icons.upload),
            label: const Text('Upload'),
          ),
          if (progress != null) ...[
            const SizedBox(height: 16),
            LinearProgressIndicator(value: progress),
            const SizedBox(height: 8),
            Text(
                'Sending… ${(progress * 100).toStringAsFixed(0)}%. Keep the phone near the controller.'),
          ],
        ],
        if (_message != null) ...[
          const SizedBox(height: 16),
          Text(_message!,
              style: TextStyle(
                  color: _failed
                      ? Theme.of(context).colorScheme.error
                      : Theme.of(context).colorScheme.primary)),
        ],
      ],
    );
  }
}

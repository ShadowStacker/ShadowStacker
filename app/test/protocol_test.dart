import 'package:flutter_test/flutter_test.dart';
import 'package:shadowstacker/protocol.dart';

List<int> hex(String s) => [
      for (var i = 0; i < s.length; i += 2)
        int.parse(s.substring(i, i + 2), radix: 16)
    ];

void main() {
  // These byte strings were captured from the v14 firmware running in simulation,
  // so the two ends of the protocol are checked against each other.
  test('decodes a settings record produced by the firmware', () {
    final c = StackerConfig.decode(hex(
        '010e010300000041030000006400000084030000e8030000000000005effffff58000000f6070000c9a0003b00007041'))!;
    expect(c.layout, 1);
    expect(c.firmwareVersion, 14);
    expect(c.clockwise, true);
    expect(c.hasStart, true);
    expect(c.hasEnd, true);
    expect(c.rpm, 8.0);
    expect(c.moves, 3);
    expect(c.frames, 4);
    expect(c.stepsPerMove, 100);
    expect(c.shotDelayMs, 900);
    expect(c.settleDelayMs, 1000);
    expect(c.backlashSteps, 0);
    expect(c.startMark, -162);
    expect(c.endMark, 88);
    expect(c.stepsPerRev, 2038);
    expect(c.mmPerStep, closeTo(4 / 2038, 1e-9));
    expect(c.maxRpm, 15.0);
    expect(c.stepMicrons, closeTo(196.27, 0.01));
    expect(c.travelMm, closeTo(0.5888, 0.0001));
    // 3 moves of 0.368 s + (1000 + 100 + 900) ms each, plus the last frame
    expect(c.estimatedSeconds, 8);
  });

  test('decodes a status record produced by the firmware', () {
    final s = StackerStatus.decode(hex('04036cffffff010000000400000008000000'))!;
    expect(s.state, StackerState.moving);
    expect(s.stackRunning, true);
    expect(s.hasStart, true);
    expect(s.hasEnd, true);
    expect(s.position, -148);
    expect(s.framesTaken, 1);
    expect(s.framesTotal, 4);
    expect(s.secondsLeft, 8);

    final idle = StackerStatus.decode(hex('00035effffff040000000400000000000000'))!;
    expect(idle.state, StackerState.idle);
    expect(idle.busy, false);
    expect(idle.position, -162);
  });

  test('rejects short or unknown records', () {
    expect(StackerStatus.decode([0, 1, 2]), isNull);
    expect(StackerConfig.decode(List.filled(47, 0)), isNull);
    final odd = StackerStatus.decode([99, ...List.filled(17, 0)])!;
    expect(odd.state, StackerState.unknown);
  });

  test('reassembles lines split across packets', () {
    final a = LineAssembler();
    expect(a.add('Frame 1 of'.codeUnits), isEmpty);
    expect(a.add(' 4 taken\r\nEstim'.codeUnits), ['Frame 1 of 4 taken']);
    expect(a.add('ated time left: 7 (S)\n\n'.codeUnits),
        ['Estimated time left: 7 (S)']);
  });

  test('builds the firmware update begin message', () {
    final m = otaBeginMessage(10000, List.generate(16, (i) => 0xA0 + i));
    expect(m.length, 21);
    expect(m.sublist(0, 5), [1, 0x10, 0x27, 0, 0]);
    expect(m[5], 0xA0);
    expect(m[20], 0xAF);
    expect(readUint32([2, 0x00, 0x10, 0, 0], 1), 4096);
  });

  test('depth of field and suggested step', () {
    final dof = depthOfFieldMm(
        magnification: 1, aperture: 8, circleOfConfusionMm: 0.020);
    expect(dof, closeTo(0.64, 1e-9));
    expect(suggestedStepMicrons(dof), closeTo(448, 1e-6));
    expect(
        depthOfFieldMm(
            magnification: 0, aperture: 8, circleOfConfusionMm: 0.02),
        0);
  });

  test('formats durations', () {
    expect(formatDuration(7), '0:07');
    expect(formatDuration(200), '3:20');
    expect(formatDuration(3725), '1:02:05');
  });
}

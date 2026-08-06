# Motion slot HARD veto investigation (2026-08-06)

## Symptom

`phorce play 1` is rejected even though motion slot 1 exists, the robot reports
`physical_idle: true`, and `queue_count: 0`.

```text
REJECT_QUEUE_FULL
[PCM: rejected_at_enqueue / veto_active]
```

The request is not queued and is not sent to the PCM.

## Environment

- ROS 2 Humble
- `ROS_DOMAIN_ID=21`
- One `/phorce_monitor` and one `/motion_action_server`
- All eight `phorce-*` packages: `0.1.0+git20260806`
- Monitor parameters:

```bash
ros2 run agx_phorce_bridge phorce_monitor \
  --ros-args -p nic:=eno1 -p mode:=command -p axes:=2 -p mbx_enabled:=true
```

## Reproduction state

- Button 1 changes `physical_idle` from `false` to `true`.
- `active: false`
- `recovery_required: false`
- `queue_count: 0`
- `last_err: 0`
- Action server count remains 1.
- A complete monitor/action-server restart does not clear the fault.

Representative `/phorce/status` values after restart:

```yaml
master_state: 3
feedback_rate_hz: 999.17
wkc_err: 0
fresh_losses: 0
verdict_pass: 0
verdict_soft: 0
verdict_hard: 108612
ecat_state: 8
axis_oper_mask: 256       # 0x0100
axis_stale_mask: 0
axis_fault_mask: 0
status_flags: 72          # 0x0048
estop_active: false
ethercat_operational: true
mbx_veto_active: true
mbx_verdict_source: 1
mbx_lane_up: true
mbx_lane_bind_result: 0
mbx_rejected_presend: 0
mbx_emergencies: 0
```

## Findings

### The rejection is before EtherCAT transmission

The mailbox enqueue gate rejects the write because the latest supervisor verdict
is HARD. This is not a PCM SDO abort and not an actual full motion queue.

### `status_flags=0x48` is not itself a danger-bit veto

The installed PDO contract defines:

- `0x08`: `AM_CMD_FRESH` (healthy-high)
- `0x40`: `ETHERCAT_OPERATIONAL` (healthy-high)
- HARD raw danger mask: `0x33`

Therefore `0x48 & 0x33 == 0`; E-Stop, Rx contract fault, and LAN9252 I/O fault
are not asserted in this sample.

### Expected and observed axis masks do not match

The installed runtime treats the `axes` parameter as an explicit bitmask. Its
diagnostic strings document `axes:=2` as `0x0002`, not as a count of two axes.
The PDO contract also fixes the current powered PCM profile to `0x0002`.

Observed:

```text
expected_axis_mask = 0x0002
axis_oper_mask     = 0x0100
```

`FeedbackMonitor` requires `axis_oper_mask` to contain every bit in
`expected_axis_mask`. The observed `0x0100` does not contain `0x0002`, so CM
feedback is folded to untrusted/not-fresh. The safety supervisor maps untrusted
CM feedback to `CM_FEEDBACK_STALE`, which is a HARD rejection. This explains
`verdict_hard == cycles` and `verdict_pass == 0` from the first cycle after every
restart.

## Likely root cause

The PCM/PhACT topology reports the operational actuator at axis bit 8 while the
hackathon powered profile requires axis bit 1. Likely causes, in descending
order:

1. PhACT DIP/node-ID configuration does not match the powered profile.
2. PCM firmware axis-to-bit mapping or configured expected actuator differs from
   the Jetson runtime contract.
3. PCM firmware and the `20260806` Jetson runtime use incompatible PDO axis-mask
   conventions.

This is not safely fixed by changing `axes:=256`: the installed contract states
that the powered PCM profile expects exactly `0x0002`, and PCM relay eligibility
also requires exact agreement with that value.

## Required operator checks

1. Read the actual PhACT DIP/node ID and compare it with the profile expecting
   axis bit `0x0002`.
2. Confirm PCM firmware build/revision and its `CM_ECAT_EXPECTED_AXIS_MASK`.
3. Confirm the PCM reports `axis_oper_mask=0x0002` after correcting topology.
4. Only retry motion after `/phorce/status` shows:

```text
axis_oper_mask: 2
mbx_veto_active: false
verdict_pass: increasing
```

Do not bypass the HARD veto, disable the mailbox gate, or substitute
`axes:=256` without an approved profile/firmware change.

## Earlier issue resolved separately

ROS domain 0 initially exposed two action servers and two monitors. Moving this
robot stack and its clients to `ROS_DOMAIN_ID=21` reduced both to one. That
discovery collision is no longer part of the current HARD-veto failure.

## Resolution and generalized participant configuration

Replacing the connected actuator with node `0x03` changed
`axis_oper_mask=0x0100` to `0x0002`. After that change the monitor reported
`mbx_veto_active=false`, `verdict_pass` increased continuously, and motion slot
playback succeeded. This confirms the expected/observed axis-mask mismatch as
the cause.

For participant motion-slot playback, the generalized startup configuration is:

```bash
ros2 run agx_phorce_bridge phorce_monitor --ros-args \
  -p nic:=eno1 -p mode:=op_idle -p axes:=auto -p mbx_enabled:=true
```

`axes:=auto` confirms one stable nonzero observed mask for 250 consecutive valid
frames before promoting it to the explicit expected profile. `mode:=op_idle`
keeps RT joint streaming off while retaining feedback supervision and the
mailbox safety gate used by P-Vector motion-slot playback. This avoids the
command-mode powered-profile restriction without bypassing a safety verdict.

### Hardware validation

The generalized configuration was validated on the node `0x03` actuator:

```yaml
mode: op_idle
master_state: 3
axis_oper_mask: 2
axis_stale_mask: 0
axis_fault_mask: 0
verdict_pass: 14594
verdict_hard: 1
mbx_veto_active: false
mbx_rejected_enqueue: 0
mbx_rejected_presend: 0
```

Motion slot 1 then completed successfully:

```text
SUCCEEDED — completed 1/1 — token=(1358950815,2)
PCM explicitly confirmed aggregate motion-slot completion.
```

The single startup HARD verdict did not increase while PASS verdicts increased
continuously, so it is consistent with initialization rather than an active
safety fault.

## Six-axis follow-up validation

After expanding the setup to the six-axis robot, the runtime reported:

```yaml
mode: op_idle
axis_oper_mask: 455       # 0x01C7: axes 0, 1, 2, 6, 7, and 8
axis_stale_mask: 0
axis_fault_mask: 0
ethercat_operational: true
mbx_veto_active: false
```

The six set bits in `axis_oper_mask` match the six axes used by this robot.
There was no evidence of an offline, stale, or faulted actuator.

The first playback test mistakenly invoked motion slot 1:

```bash
export ROS_DOMAIN_ID=21
phorce play 1 --target robot
```

Slot 1 returned `SUCCEEDED`, but not every motor moved. This was not a
communication or completion-contract failure: `SUCCEEDED` confirms PCM
aggregate slot completion, not that every operational axis had a nonzero
position change. The intended test asset was slot 4, so the observed partial
movement came from selecting the wrong motion slot.

The runtime was kept in the participant motion-slot configuration:

```bash
# Terminal 1: EtherCAT feedback and mailbox safety gate
export ROS_DOMAIN_ID=21
ros2 run agx_phorce_bridge phorce_monitor --ros-args \
  -p nic:=eno1 -p mode:=op_idle -p axes:=auto -p mbx_enabled:=true

# Terminal 2: motion-slot action server
export ROS_DOMAIN_ID=21
ros2 run agx_motion_slot motion_action_server --ros-args -p backend:=ecat
```

The corrected playback command was:

```bash
export ROS_DOMAIN_ID=21
phorce doctor
phorce list
phorce play 4 --target robot
```

### Result

Motion slot 4 executed successfully and all six axes moved correctly. This
validates the following together on the tested hardware:

- ROS isolation on domain 21
- automatic six-axis mask acquisition with `axes:=auto`
- `op_idle` motion-slot operation without RT joint streaming
- EtherCAT feedback and mailbox safety gating
- the PCM-loaded motion asset in slot 4

Final cause: operator-side motion ID selection (`1` instead of `4`). No runtime,
firmware, axis mapping, or safety-gate modification was required for this
follow-up issue. The only correction was to execute motion slot 4.

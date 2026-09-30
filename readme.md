# Stepsew

ESP32 firmware for a pedal-controlled sewing-machine stepper motor. Releasing
the pedal after sewing stops the needle at bottom dead center (BDC). Short
pedal taps at rest toggle the needle between BDC and top dead center (TDC).

## Build

```sh
pio run -e esp32dev
```

The project pins Espressif32 7.1.3 and FastAccelStepper 1.3.0. Position capture
uses the classic ESP32's IDF 4 pulse-counter hardware; changing to another ESP32
variant or an IDF 5 platform requires adapting that code. Compiler warnings in
project sources are treated as errors.

## Wiring and configuration

Settings live in [`include/config.h`](include/config.h).

| Signal | GPIO | Configuration |
| --- | --- | --- |
| Motor STEP | 14 | Rising-edge step pulses |
| Motor DIR | 27 | Fixed LOW; forward-only operation |
| Motor ENABLE | 26 | Active LOW |
| Pedal | 2 | 12-bit analog input |
| BDC sensor | 33 | Pull-up input; falling edge marks BDC |

The defaults retain 2,000 pulses per motor revolution, 60–3,000 motor RPM, and
400,000 steps/s² acceleration. Pedal thresholds remain 2,000 for starting and
2,250 for variable speed, with 20 counts of hysteresis. Readings are sampled
every millisecond and averaged over 20 samples. The speed curve uses exponent
1.2 and is clamped to the configured speed limits.

`kStepsPerNeedleCycle` is independent of `kMotorStepsPerRevolution`. The current
value of 4,667 accounts for approximately 2.33 motor revolutions per needle
cycle with 2,000 pulses per motor revolution. Confirm the exact ratio from
pulley tooth counts or the measured BDC crossing interval. This value includes
both the pulley ratio and motor-driver microstep setting.

Align the sensor's falling edge with physical BDC. `kBdcOffsetSteps` can add a
forward offset when the sensor is mounted before BDC. The final position must
remain within the sensor's active LOW region. Homing accepts one captured
sensor pulse to plan the approach; it does not require two crossings with
matching spacing. A fresh edge is then required during the final moving
search. Both homing and the return tap use that same search.

`kBdcToTdcSteps` sets the forward distance from BDC to TDC. Its default is half
a needle cycle (2,333 steps). The return move watches for the next BDC edge
throughout its forward travel from TDC. Both moves rotate
forward. Calibrate this distance if the machine's linkage
does not put TDC exactly half a cycle after BDC. There is no TDC sensor: its
position is inferred from the established BDC reference and counted STEP pulses.

## Short pedal taps

After a successful BDC stop:

- Press and release the pedal within 20–250 ms to raise the needle to TDC.
- Once the needle has stopped at TDC, another short tap lowers it to BDC.
- Hold the pedal for 250 ms to start normal sewing. Releasing it after sewing
  homes to BDC as usual.

The motor stays still while distinguishing a tap from a held press. Tap motion
starts on pedal release. The upward move runs at 60 RPM and takes about
1.2 seconds with the default settings. The downward move scans continuously
at 60 RPM from TDC and requests normal deceleration when a falling edge is
captured. It clears pending capture and the preceding debounce lockout before
starting, and uses speed-dependent debounce throughout the return. Capture is
not restricted to an estimated half-cycle endpoint or a narrow window around it.
The return has a finite travel limit of one nominal needle cycle (4,667 steps),
so a missing signal cannot cause unlimited rotation. It normally stops at the
first usable BDC crossing, about half a cycle from TDC. Homing also allows
a full-cycle search after its approach, so a noisy planning reference cannot
restrict final capture to a narrow predicted window.

Queued pulses and braking cause some travel beyond the captured edge. The
maximum accepted travel beyond the configured BDC offset is calculated from
speed, queue planning, latency allowance, and acceleration
(`kBdcCaptureToleranceSteps`: 44 pulses, about 3.4° of the needle cycle at
60 RPM). A direct scan entered from up to 120 RPM budgets up to 97 pulses
for the initial queued speed and deceleration. This is a limit, not a commanded offset or a guarantee of exact
physical BDC. After motion ends, HIGH samples reset LOW qualification but
allow up to 50 ms (`kBdcStopQualificationMs`) for the mechanism/input to settle
before rejecting the candidate. The sensor must still stay LOW for 10 ms at
rest, followed by final verification. If the sensor's active region is too narrow for braking,
the search continues within its remaining travel budget and otherwise faults.
Align the magnet/sensor and calibrate `kBdcOffsetSteps` for the physical stop.
`BDC stop: edge_to_stop=... offset=...` reports the measured pulse travel.
The next upward move subtracts the measured overrun from its step count to
retain the configured TDC target relative to the captured BDC reference.
Tap duration is measured using the filtered pedal reading
and the existing start/release hysteresis; pulses shorter than 20 ms are ignored.
Adjust `kPedalTapMinMs` and `kPedalTapMaxMs` to change these durations.

Wait for each toggle to finish before tapping again. Short presses during a
toggle are ignored rather than queued. A held press during a toggle resumes
sewing after the same 250 ms threshold, including when the toggle finishes
during that press. A continuous hold cannot trigger repeated toggles.

The controller verifies the final pulse counts and requires a stable LOW BDC
sensor at BDC, or a stable HIGH sensor at TDC, before releasing the motor.
The upward toggle deadline is about 3.3 seconds with the current configuration
and scales with the configured needle-cycle length. Returning to BDC allows
three additional seconds for the search (`kBdcSearchTimeoutMs`). Timeout
messages distinguish motion, sensor search, and final sensor verification.
Failures latch the same fault as
homing; they do not switch the remembered needle position.

Taps are enabled only after a verified BDC stop. At power-up, a pedal press
starts sewing immediately as before; sew and release once to establish the
reference. A detected sensor crossing or changed level while parked invalidates
the reference. After manually moving the handwheel, sew and release again
before using taps: the single BDC sensor cannot detect every manual movement,
particularly near TDC.

## Stop behavior

1. Pedal release starts homing once, from either sewing-speed mode.
2. At or below `kPositionFeedbackMaxRpm` (120 motor RPM by default), release
   converts the existing motion directly into a finite scan at 60 RPM, watching
   for the next Hall edge immediately. Actual, queued, and requested speed
   must all be within the threshold. It logs `Homing: direct low-speed BDC scan`.
   This path has no predicted approach point and requires no earlier reference
   or qualifying stitch. A recent edge still under the sensor can request an
   immediate stop. Travel remains limited to one nominal cycle, and normal
   braking and final LOW verification still apply. An edge already passed
   outside the usable stop region cannot be recovered by reversing.
3. Above the low-speed threshold, the motor changes to 120 RPM
   (4,000 STEP pulses/s). The controller waits
   until current and queued speeds are low enough for 50 ms before planning
   the stop. `kHomingRpm` controls this speed independently of sewing and taps.
4. BDC pulses are tracked during sewing and deceleration for diagnostics.
   Once speed has settled, pending capture and the previous reference are
   discarded. One fresh falling edge at homing speed establishes the reference
   for the approach; no reference from high-speed motion is reused.
5. The motor moves forward to a reachable approach position 93 steps before
   the estimated BDC edge, adding complete needle cycles when required by
   queued pulses and braking distance (`kBdcApproachLeadSteps`). The estimate is
   only an approach target; the final scan allows a full nominal cycle.
   At the current ratio, one needle cycle at homing speed takes about 1.17 s.
6. `Homing: locating the BDC sensor edge` starts the same bounded, continuous
   60 RPM search used by the return tap. A fresh edge with a stable LOW at rest is
   required. An earlier sewing pulse alone cannot establish the final stop.
7. After applying any BDC offset, the controller verifies both the library's
   position and the independent hardware pulse count and requires the sensor
   to remain LOW for 10 ms before releasing the motor outputs.

A pedal press lasting at least 50 ms (`kHomingResumePressMs`) cancels homing
and resumes sewing. Release hysteresis resets that timer; a brief pedal rebound
cannot cancel the stop. The next release either starts a direct low-speed scan
or reacquires a reference at settled speed. A new sewing
session from rest starts fresh reference acquisition, so references from before
a handwheel adjustment are not reused. The controller does not automatically
move or assume it is homed at power-up.

The sensor interrupt applies a speed-dependent falling-edge lockout:

`lockout_us = 1,000,000 × steps_per_needle_cycle / pulses_per_second × 50%`

`kSensorDebouncePercent` controls that fraction. At steady speed the defaults
are about 1,167 ms at 60 motor RPM, 583 ms at 120 RPM, and 23 ms at 3,000 RPM.
The rate uses the highest actual, queued, or requested pulse frequency, so
acceleration shortens the lockout promptly. An active lockout uses the smaller
of its original duration and the current duration: deceleration cannot extend
it and hide the next crossing. Rejected edges do not restart the timer.
Calculations run in the control loop; the ISR uses the published duration and
retains the first edge's timestamp and pulse count.

The moving lockout is bounded by `kSensorDebounceMinUs` (1 ms) and
`kSensorDebounceMaxUs` (2 s). At rest the setting is
`kSensorStoppedDebounceUs` (500 ms). The final BDC search uses the same
speed-dependent debounce as sewing: there is no search-specific 1 ms override.
Candidate rejection does not reset debounce, so repeated edges from the same
burst cannot repeatedly restart braking. A false first edge can still mask a
nearby genuine edge during its lockout; the finite travel and time limits
remain in effect.

This suppresses repeated edges but cannot distinguish an isolated noise spike
from a genuine Hall crossing. No minimum pulse width is required. The handler
does not reread the input level to decide whether an edge counts:
a pulse may already have ended when the interrupt or control loop runs. The
first pending edge's timestamp and hardware pulse count are retained until the
loop consumes them, so bounce cannot replace that reference. Subsequent edges
within three quarters of a needle cycle are ignored based on motor travel.
No exact two-pulse spacing check is used. The final scan searches up to one
nominal cycle from the approach and faults if no usable edge is found. Large raw interrupt counts can indicate
chatter or interference; software cannot establish physical BDC from a signal
that also stays active at unrelated positions.

PCNT unit 7 independently counts STEP pulses, including while the application
is busy. The sensor interrupt captures that count without resetting the
library's position. Unsigned position differences handle counter wraparound.
Units used internally by FastAccelStepper are left available to the library.

`kForwardPlanningMs` is 8 ms, configured only during motor initialization.
FastAccelStepper's task refills roughly every 4 ms; this retains two refill
cycles while reducing queued travel before a normal stop can take effect.
At 60 RPM, 8 ms corresponds to 16 pulses, versus 40 pulses with the former
20 ms setting. Task timing and deceleration add further travel, so this is
neither an exact stop distance nor a guarantee that the magnet remains in the
sensor's active region. Both sewing and positioning use this queue setting;
check smooth high-speed operation as well as the final stop after changing it.

The homing deadline covers two needle cycles at homing speed, maximum
deceleration, settling, a one-second margin, and three seconds for the final
edge search (about 6.6 seconds total with the current settings). Missing or unusable sensor
signals, failed motor commands, and position mismatches latch a fault. The
motor decelerates and its outputs are released once it stops. If a fault stop
has not completed within one second, pending pulses are aborted. Correct the
cause and reset the controller to resume; pressing the pedal cannot clear a
fault. Serial output at 115200 baud reports state transitions and fault causes.

Homing timeout messages identify the stage that failed. They also report the
sensor level, raw falling-edge interrupts, `debounced` edges rejected by the
time lockout, captured/stale/ignored edges,
accepted references, last observed interval, last accepted interval, and
hardware pulse count. The last observed interval may describe rejected bounce;
the accepted interval shows spacing between references used for planning. Counts
cover the current sewing session, including deceleration and reference
acquisition. `raw=0` means no falling-edge interrupts reached the handler;
`raw` can exceed `captured` due to debounce and when multiple admitted edges
arrive before the loop consumes the first one. The interval is diagnostic only, not an exact-spacing
requirement. The 16-bit hardware pulse count wraps every 65,536 steps, while
the library position continues counting; only their low 16 bits must match.
Diagnostics are printed after requesting a stop, so they do not delay capture.

A live `BDC pin=33 HIGH raw=... db=... read=... merged=... us=...` status line
is attempted once per second, including while stopped or faulted. It skips
printing if the serial transmit buffer lacks room. These counters start at boot:

- `raw`: all falling-edge interrupts, before debounce.
- `db`: interrupts rejected by the time lockout.
- `read`: captured events consumed by the control loop, before position filtering.
- `merged`: admitted interrupts coalesced while an earlier event was pending.
- `us`: currently configured lockout in microseconds; an existing lockout may
  be shorter because it began at a faster speed.

The HIGH/LOW value is an instantaneous reading and may miss a short pulse;
the raw counter retains evidence of its interrupt. Moving the magnet past the
sensor by hand while stopped can therefore check capture independently of motor
operation. Search faults also print `observed`, `stale`, `outside`, and
`returned_HIGH` counts. The latter identifies captured candidates rejected
because LOW did not persist at rest. `BDC rejected edges` additionally reports
minimum and maximum pulse travel from those edges to standstill, to help
distinguish signal glitches from braking past the active region. These counts
alone do not prove which cause applies. `outside` includes approach edges before
the final search window, which are normally ignored.

## Hardware verification

A successful build cannot establish mechanical positioning accuracy. Before
using the positioning feature:

- Confirm the driver microstep setting, pulses per needle cycle, rotation
  direction, and the sensor's falling-edge alignment with needle-down BDC.
- Release the pedal from minimum and higher speeds at several points in the
  needle cycle. Confirm the needle consistently finishes at BDC, motion has
  ended before ENABLE is released, and the serial output reaches `Stopped`.
- Press the pedal again while slowing, seeking BDC, and approaching the target.
  Confirm sewing resumes without reversing or briefly disabling the driver.
- After a verified BDC stop, tap once and check physical TDC, then tap again
  and check BDC. The return should report `Tap: locating the BDC sensor edge`
  before `Stopped: needle at BDC`. Confirm each tap produces just one toggle,
  all motion is forward, and the needle stays at its final position after the
  driver is released. Repeat the sequence to check that BDC does not drift.
- Hold the pedal at BDC and TDC. Confirm normal sewing starts after 250 ms and
  releasing the pedal returns to BDC. Check a held press during a toggle too.
- With the mechanism unloaded, check a disconnected sensor and a sensor held
  LOW. Homing must time out, stop the motor, and reject pedal input until reset.
- Check repeated stops and restarting after manually turning the handwheel.
  Release at different points during minimum-speed sewing and check for the
  direct-scan message, without `Homing: approaching BDC`. Release after fast sewing and check for fresh reference
  acquisition after speed settling. If BDC verification fails, check
  the sensor's active region and calibration rather than increasing the timeout.

No software tests are included.

## Source layout

- `main.cpp`: Arduino setup and control-loop scheduling.
- `machine.cpp`: sewing, tap gestures, BDC/TDC positioning, and fault states.
- `stepper.cpp`: FastAccelStepper commands and pulse-based stop planning.
- `bdc_sensor.cpp`: interrupt capture and atomic event transfer.
- `pedal.cpp`: fixed-size averaging and bounded speed mapping.

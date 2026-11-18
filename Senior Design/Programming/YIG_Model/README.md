# YIG Tuning Model (MATLAB / Simulink)

A closed-loop model of an AlNiCo-latched YIG filter with a passive bias
magnet, driven from a vendor calibration LUT, with a swappable controller
block.

## Quick start

```matlab
cd <path to this folder>
yig_setup                          % paths + device LUT + magnet fit + builds the .slx the first time
out = sim('yig_tuning_model');     % run the Simulink model
s = yig_plot_results(out);
yig_spec_report(s, YIG, ref_profile);   % check the project specs
```

To run a different device, set `yig_device` before `yig_setup`:

```matlab
yig_device = 'pcb2_yig6_20260223';  yig_setup
```

Without Simulink, or to debug with breakpoints:

```matlab
out = yig_sim_script();  s = yig_plot_results(out);  yig_spec_report(s, YIG, ref_profile);
```

Both run the same plant and controller code.

## Device LUTs

Each board's vendor calibration lives in its own function in `devices/`:

| File | Board | Reset | Write points (V → GHz) |
|---|---|---|---|
| `pcb1_yig12_20260223.m` | PCB1, YIG12, new passive (default) | 50 V, POL 7B | 20→4.5, 25→7.0, 30→10.0, 35→12.8 (POL 7A) |
| `pcb2_yig6_20260223.m` | PCB2, YIG6, new passive | 50 V, POL 7A | 20→4.6, 25→7.2, 30→10.0, 35→12.8 (POL 7B) |
| `pcb2_old_20250611.m` | PCB2, old passive | 75 V, POL 7A | 20→4.6, 27→7.0, 36→10.0, 50→13.0 (POL 7B) |

To add a board, copy one of these and fill in the fields:

```matlab
dev.name      = 'PCBx-YIGy-date';   % free text
dev.sparam    = 'S12';              % which S-parameter the LUT was measured on
dev.pol_reset = '7B';               % H-bridge code that drives the field DOWN
dev.pol_write = '7A';               % H-bridge code that drives the field UP
dev.V_reset   = 50;                 % reset pulse voltage [V]
dev.f         = [4.5 7 10 12.8]*1e9;% LUT centre frequencies [Hz], increasing
dev.V         = [20 25 30 35];      % write voltage for each frequency [V]
```

The vendor procedure is: reset with `V_reset` on the reset polarity, then
fire one write pulse of `V` volts on the write polarity. The model uses
polarity −1 for reset (field down) and +1 for write (field up); the POL
codes are only labels, since they swap between boards.

`yig_setup` passes the LUT to `params/yig_params.m`, then
`sim/yig_fit_device.m` replays the vendor procedure through the plant code
and fits the magnet (`B_pass`, `Bmax`, `hc_mean`, `hc_sd`, `hu_sd`, `D`) so
the model reproduces the LUT. It also forces the reset pulse to fully
saturate the magnet, which is what makes a LUT valid. The fit takes about
20 s and prints a table of LUT vs model frequency. Typical results: under
3 MHz RMS for the new-passive boards, about 34 MHz RMS for the old passive.

The centre frequency follows the vendor formula fc [MHz] = 2.8·B [G] + 2217.
The filter passband shape is only a rough stand-in. The vendor notes that a
real YIG response needs FEM (e.g. HFSS). Only the detector reading `y(4)`
uses it; the controller tunes on the centre frequency.

## Where to put your controller

Edit **`control/yig_controller.m`**. The yellow *YIG Controller* block in
the model is a one-line wrapper that calls it every `Tc` (default 20 µs, i.e. 50 kHz):

```matlab
u = yig_controller(f_target, y, YIG);
```

To try several controllers side by side, copy the file under a new name,
e.g. `control/my_pid.m`, with the same signature. Then double-click the
block and change that line to `u = my_pid(f_target, y, YIG);`.

Controller code runs in a MATLAB Function block, which means it is compiled
with code generation. That brings a few rules:

- Keep state in `persistent` variables and initialise them inside `if isempty(...)`.
- Use fixed-size arrays only. No strings, cells, `disp`, or plotting.
- Keep persistents numeric `double`. Assigning a `logical` to a double persistent is a compile error.
- Anything tunable goes in `params/yig_params.m` under `P.ctrl.*`.

### Controller I/O

| Input `y(k)` | Meaning | Unit |
|---|---|---|
| 1 | main-coil current (shunt) | A |
| 2 | storage-cap voltage | V |
| 3 | Hall-sensor reading (convert with the bench calibration, see `hall_to_f` in the controller) | T |
| 4 | detector power at `f_probe` | dBm |
| 5 | magnet temperature | °C |
| 6 | `busy`: a pulse is in progress | 0/1 |
| 7 | `ready`: idle and cap at `V0_set` | 0/1 |
| 8 | trim-coil current | A |

| Output `u(k)` | Meaning | Unit |
|---|---|---|
| 1 | `V0_set`: charger target | V |
| 2 | `fire`: a 0→1 edge fires one pulse | – |
| 3 | `polarity`: +1 / −1 | – |
| 4 | `I_lim`: peak-current cutoff (0 means natural peak) | A |
| 5 | `I_trim`: trim-coil current command | A |
| 6 | `f_probe`: probe-tone frequency | Hz |
| 7 | `hall_on`: Hall sensor powered (used for the power estimate) | 0/1 |
| 8 | controller mode id (logging only) | – |

`y` arrives one plant sample late. This models ADC latency and breaks the
algebraic loop.

## Model structure

```
Target Frequency ─► YIG Controller ─► RT ─► YIG Plant ─┬─► log_truth / Scope
                         ▲                     ▲        │
                         └── RT ◄─ ADC latency ◄────────┘ y
                                Environment (T, stray B) ┘
```

The **YIG Plant** subsystem runs every `Ts` (1 µs):

| Block | File | What it models |
|---|---|---|
| Pulse Driver | `plant/yig_driver_step.m` | Constant-current charger (with a charge-voltage error at firing), storage cap, and H-bridge into the main coil. The bridge turns off at the natural current peak or at `I_lim`, then the body diodes recover energy back into the cap. Also models the trim-coil current loop. |
| AlNiCo Magnet | `plant/yig_magnet_step.m` | Preisach hysteresis (Everett table plus wipe-out memory) with the gap's self-demagnetising field, solved implicitly each step |
| Gap Field and YIG Filter | `plant/yig_resonator_step.m` | B_gap = passive bias + AlNiCo (with tempco), main coil, trim coil, and stray field. Includes eddy-current lag. Gives f0 = 2.8 MHz/G·\|B\| + 2217 MHz and a rough n-stage filter response at the probe tone. |
| Sensors | `plant/yig_sensors_step.m` | Hall sensor (`plant/yig_hall_ideal.m`: coupling, gain/offset error, temperature drift; plus noise, bandwidth and 16-bit ADC), other sensors' noise and quantisation |

`truth` columns (in `yig_plot_results`):
1 I_coil, 2 Vc, 3 busy, 4 ready, 5 I_trim, 6 m, 7 H_eff, 8 H_coil, 9 f0, 10 B_gap, 11 P_out_dBm.

## Baseline controller (what ships in `yig_controller.m`)

This is the vendor procedure, closed around a **Hall sensor** that measures
the field near the YIG sphere. `P.ctrl.strategy` picks how the magnet is
fine-tuned after the first write:

| | Strategy 1: approach from below | Strategy 2: overshoot, correct down (default) |
|---|---|---|
| First write aims | `write_bias` (20 MHz) **low** | `write_bias` **high** |
| Fine correction | small **+** pulses (~600 MHz/V) | small **reverse** pulses (~100–170 MHz/V, gentler) |
| Magnet ended up on the wrong side | reset + rewrite (a GHz-sized outage) | re-fire the same write (restores the overshoot), correct down again |
| Needs | the vendor LUT | the vendor LUT **plus** a reverse-pulse calibration |

Steps:

1. **Reset.** A natural −pulse at `V_reset` puts the magnet at its lowest field (about 2.4–3.3 GHz depending on the board).
2. **Write.** A natural +pulse at `V_k` from the device LUT, aimed low or high (above).
3. **Measure.** After the eddy currents settle (`t_settle` = 7.5·`tau_eddy`), estimate the magnet-only frequency. Inside the accept window → feed the remaining error straight into the trim coil and go to TRACK. Otherwise: too low → bigger write (no reset; Preisach wipe-out); too high → reset and write smaller.
4. **Track.** A PI loop on the trim coil (about ±56 MHz) holds the target.
5. **Standby.** After `t_standby` (5 ms), refine pulses (per strategy) move the magnet until the trim supplies less than `standby_enter_tol` (6 MHz). Then the trim and Hall sensor switch off and the magnet holds with zero power. Every `t_wake` (20 ms) the Hall is powered briefly; if the error exceeds `standby_wake_tol` (8 MHz) the controller goes back to TRACK.

The voltage table (`P.ctrl.cal_f`, `P.ctrl.cal_V`) starts as the device LUT
and is refined after every write. Writes never exceed `V_reset`, so a reset
can always erase them.

### Reverse pulses

In the hysteresis model a **smaller** pulse in the same direction as the last
one does nothing, but a **reverse** pulse lowers the field once it is past a
"dead zone". Example, PCB1 after a +30 V write (about 10.00 GHz): −15 V
lowers it 2 MHz, −18 V 37 MHz, −20 V 160 MHz, −24 V 1.1 GHz, −40 V back to
the reset level. The dead zone differs a lot per board (corrections start at
about 0.5× the write voltage on PCB1, 0.83× on PCB2-YIG6), so strategy 2
uses a **reverse calibration table** (`P.ctrl.rev_V`): for every 1 V of write
voltage, the reverse voltage that lowers the frequency by 5/10/20/40/80 MHz.
`yig_fit_device` builds it from the fitted magnet; on the bench it would be
measured the same way as the vendor LUT. If a correction misses, the
controller brackets the right voltage between "too little" and "too much",
re-firing the write to undo an overshoot.

### Strategy comparison

3 boards × 3 noise seeds, default test profile (script sim):

| Board | Strategy | Pulses per run (reset / + / reverse) | Runs with a > 0.5 GHz excursion | Worst error otherwise | Worst response |
|---|---|---|---|---|---|
| PCB1 | 1 | 5.7 / 18.7 / 0 = 24.3 | 1 of 3 | 87 MHz | 2.32 ms |
| PCB1 | 2 | 5.7 / 8.3 / 7.3 = 21.3 | 1 of 3 | 57 MHz | 2.35 ms |
| PCB2-YIG6 | 1 | 5.0 / 20.0 / 0 = 25.0 | 3 of 3 | – | 2.21 ms |
| PCB2-YIG6 | 2 | 4.0 / 7.0 / 9.7 = 20.7 | 0 of 3 | 99 MHz | 2.18 ms |
| Old passive | 1 | 7.3 / 22.7 / 0 = 30.0 | 3 of 3 | – | 4.71 ms |
| Old passive | 2 | 7.3 / 11.0 / 8.7 = 27.0 | 1 of 3 | 122 MHz | 4.75 ms |

- **Response time is the same.** Both strategies share the reset + first write + trim feedforward, which is what reaches 10 MHz. Overshooting is not faster to the first in-spec moment.
- **Strategy 2 settles the magnet with fewer pulses** (10–17 % fewer) and **avoids most reset outages** (2 of 9 runs vs 7 of 9), because a magnet that drifts or lands high is corrected down instead of reset.
- With the calibration table, strategy 2 usually needs **one** reverse pulse when the target is near a LUT voltage. Without it (dead zone found by search), it needed up to 20.
- The remaining outages are fallbacks: a disturbance that saturates the trim, or a correction that does not converge (the guard then re-writes from a reset).
- **Known weakness of strategy 2:** "restoring" the overshoot by re-firing the same write voltage does not land exactly back on it. The magnet's self-demagnetising field is weaker after a reverse correction, so the same pulse pushes it higher (about +20 MHz further in one trace). The search can then fail to converge and fall back to a reset. Results also vary with the noise sequence: one Simulink run on PCB1 had three such outages. A smarter correction for "corrected too far" (a small calibrated + pulse instead of re-firing the write) is the next improvement.

### Hall sensor and calibration

`sim/yig_calibrate_hall.m` replays a bench calibration at 25 °C: write the
magnet across the tuning range, record Hall reading vs true frequency, fit
a line. That removes the sensor's coupling factor, gain error and offset.
The firmware also divides out the datasheet's typical gain tempco. What
remains is the difference between the true and assumed tempco, plus offset
drift (placeholders: 50 ppm/°C and 0.05 G/°C, about 0.7 MHz/°C at 13 GHz).

## Spec check

`sim/yig_spec_report.m` checks a run against the project specs using the
**true** centre frequency, so sensor errors count:

| Spec | How it is measured |
|---|---|
| Program with 10 MHz accuracy | worst error after the response point, plus total time out of spec |
| < 1 ms response | command → true frequency within 10 MHz and staying there for 1 ms |
| Standby power < 1 mW | average of `P_sleep` + Hall power while on + trim I²R over standby/wake time |

Results with the current placeholders (script sim, default profile):

| Spec | Result | What decides it |
|---|---|---|
| Standby < 1 mW | **Pass**, about 0.3 mW on every board | `P_sleep`, Hall power × wake duty, trim off |
| < 1 ms response | **Fail**, about 1.2 ms when the LUT's first guess is good, 2–5 ms when it isn't | the settle time after a write (7.5·`tau_eddy`), and the 4-point LUT: straight-line interpolation can be ~250 MHz off at 5 GHz, which costs a second write the first time a target is visited |
| 10 MHz accuracy | **Fail** in short bursts: about 50–120 MHz for about 0.5 ms after every pulse; occasionally a GHz-sized re-tune outage (more often with strategy 1) | the main coil's own field during a pulse (`k_main`) leaking through the eddy lag; with strategy 1, a magnet that ends up too high needs a reset |

Levers this points to (all placeholders today, so measure them first):
- **`tau_eddy`.** At 50 µs instead of 100 µs, responses drop to about 0.8 ms, but each pulse's disturbance roughly doubles (less smoothing).
- **A denser LUT** (more calibration points) removes most second writes.
- **`k_main`** (how much of the main coil's field reaches the sphere directly) sets the size of the per-pulse disturbance.
- **Charger accuracy** (`Vset_err_sd`, 5 mV ≈ 3 MHz on a + write, ≈ 0.5–0.8 MHz on a reverse correction): much worse and the magnet alone can't land within the standby tolerance with strategy 1.
- **The reverse-pulse curve** is a model prediction (the fit only saw + writes). Bench test: write +30 V, then reverse pulses of increasing size, measuring the frequency after each.

## Parameters

All parameters live in `params/yig_params.m`, which builds the struct `YIG`.
Values are marked **(measured)**, **(vendor)**, **(fitted)** by
`yig_fit_device`, or are placeholders. The coil L, R, C and `Vmax` are
measured; the frequency formula is from the vendor. The rest are
placeholders, notably `tau_eddy`, `k_main`, the Hall sensor (`P.hall.*`),
the power figures (`P.pwr.*`, including the trim-coil resistance) and the
charge-voltage error `Vset_err_sd`.
After editing, re-run `yig_setup`. If an array size changes, the blocks
recompile automatically.

The test inputs are `ref_profile` (targets) and `env_profile`
(temperature and stray field). Both are defined in `yig_setup.m`.

## Files

```
yig_setup.m                 run first (set yig_device to pick a board)
devices/*.m                 vendor calibration LUT per board
params/yig_params.m         all parameters
params/yig_build_everett.m  Preisach table from the magnet parameters
sim/yig_fit_device.m        fits the magnet model to a device LUT
sim/yig_calibrate_hall.m    bench calibration of the Hall sensor
sim/yig_spec_report.m       checks a run against the project specs
plant/yig_hall_ideal.m      noise-free Hall reading (shared by sensor + calibration)
control/yig_controller.m    <- your control law
plant/*.m                   plant blocks
sim/build_yig_model.m       (re)builds yig_tuning_model.slx
sim/yig_sim_script.m        Simulink-free simulation
sim/yig_plot_results.m      plots for either run type
```

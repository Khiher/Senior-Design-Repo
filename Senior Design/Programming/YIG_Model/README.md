# YIG Tuning Model (MATLAB / Simulink)

A closed-loop model of an AlNiCo-latched YIG filter with a passive bias
magnet, driven from a vendor calibration LUT, with a swappable controller
block.

## Quick start

```matlab
cd <path to this folder>
yig_setup                          % paths + device LUT + magnet fit + builds the .slx the first time
out = sim('yig_tuning_model');     % run the Simulink model
yig_plot_results(out)
```

To run a different device, set `yig_device` before `yig_setup`:

```matlab
yig_device = 'pcb2_yig6_20260223';  yig_setup
```

Without Simulink, or to debug with breakpoints:

```matlab
out = yig_sim_script();  yig_plot_results(out)
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
the model is a one-line wrapper that calls it every `Tc` (default 100 µs):

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
| 3 | filter centre-frequency estimate | Hz |
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
| Sensors | `plant/yig_sensors_step.m` | Noise and ADC quantisation |

`truth` columns (in `yig_plot_results`):
1 I_coil, 2 Vc, 3 busy, 4 ready, 5 I_trim, 6 m, 7 H_eff, 8 H_coil, 9 f0, 10 B_gap, 11 P_out_dBm.

## Baseline controller (what ships in `yig_controller.m`)

This is the vendor procedure with a closed loop around the measured centre
frequency `y(3)`:

1. **Reset.** A natural −pulse at `V_reset` puts the magnet at its lowest field (about 2.4–3.3 GHz depending on the board).
2. **Write.** A natural +pulse at `V_k`. The first guess comes from the device LUT.
3. **Measure and correct.** If the frequency misses by more than `write_tol` (25 MHz): too low → fire a bigger write with no reset, because Preisach wipe-out erases the smaller one; too high → reset, then write a smaller `V_k`. `V_k` comes from secant steps with a bisection fallback.
4. **Track.** The magnet holds the field with zero power. A PI loop on the trim coil (about ±56 MHz) removes residual error and drift. If the trim saturates, the controller rewrites the magnet.

The voltage table (`P.ctrl.cal_f`, `P.ctrl.cal_V`) starts as the device LUT
and is refined after every write. Writes never exceed `V_reset`, so a reset
can always erase them.

On the default test profile (10 → 12.5 → 5 → 5.02 → 7.5 GHz) all three
devices settle to under 1 MHz in about 5–16 ms per retune, using 1–4 write
pulses. Small moves are usually handled by the trim coil alone.

Closed loop assumes the real system can measure its centre frequency
(for example by sweeping a probe tone across the detector). To see how
close the LUT alone gets, set `P.ctrl.max_iter = 1` (one write per target);
note the trim PI in step 4 still uses `y(3)`.

## Parameters

All parameters live in `params/yig_params.m`, which builds the struct `YIG`.
Values are marked **(measured)**, **(vendor)**, **(fitted)** by
`yig_fit_device`, or are placeholders. The coil L, R, C and `Vmax` are
measured; the frequency formula is from the vendor. The rest, including the
noise figures and the charge-voltage error `Vset_err_sd`, are placeholders.
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
control/yig_controller.m    <- your control law
plant/*.m                   plant blocks
sim/build_yig_model.m       (re)builds yig_tuning_model.slx
sim/yig_sim_script.m        Simulink-free simulation
sim/yig_plot_results.m      plots for either run type
```

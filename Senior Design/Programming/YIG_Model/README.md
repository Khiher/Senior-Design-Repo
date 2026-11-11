# YIG Tuning Model (MATLAB / Simulink)

A closed-loop model of an AlNiCo-latched YIG filter with a swappable
controller block.

## Quick start

```matlab
cd 'C:\Users\Fairret\Desktop\Classwork Dump\ESE 4500\Matlab\YIG_Model'
yig_setup                          % paths + parameters + builds the .slx the first time
out = sim('yig_tuning_model');     % run the Simulink model
yig_plot_results(out)
```

Without Simulink, or to debug with breakpoints:

```matlab
out = yig_sim_script();  yig_plot_results(out)
```

Both run the same plant and controller code.

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
| Pulse Driver | `plant/yig_driver_step.m` | Constant-current charger, storage cap, and H-bridge into the main coil. The bridge turns off at Vc = 0 or at `I_lim`, then the body diodes recover energy back into the cap. Also models the trim-coil current loop. |
| AlNiCo Magnet | `plant/yig_magnet_step.m` | Preisach hysteresis (Everett table plus wipe-out memory) with the gap's self-demagnetising field, solved implicitly each step |
| Gap Field and YIG Filter | `plant/yig_resonator_step.m` | B_gap from the magnet (Br tempco), main coil, trim coil, and stray field. Includes eddy-current lag. Gives f0 = γ·\|B\| and an n-stage filter response at the probe tone. |
| Sensors | `plant/yig_sensors_step.m` | Noise and ADC quantisation |

`truth` columns (in `yig_plot_results`):
1 I_coil, 2 Vc, 3 busy, 4 ready, 5 I_trim, 6 m, 7 H_eff, 8 H_coil, 9 f0, 10 B_gap, 11 P_out_dBm.

## Baseline controller (what ships in `yig_controller.m`)

1. **Saturate.** A 300 V natural pulse of about 19 A puts the magnet at positive remanence (about 18.2 GHz).
2. **Knock down.** A −pulse in peak-current mode at `I_k`. f(I_k) is monotone, roughly 6 GHz/A, so the controller searches `I_k` with a bracketed secant. A deeper knock needs no reset, because Preisach wipe-out erases the previous one. A shallower knock needs a re-saturate first.
3. **Track.** The magnet holds the field with zero power. A PI loop on the trim coil (about ±56 MHz) removes residual error and drift. If the trim saturates, the controller rewrites the magnet.

A cal table (`P.ctrl.cal_f`, `P.ctrl.cal_I`) is refined after every write.

On the default test profile, large retunes settle to under 1 MHz in about
15–30 ms using 4–8 pulses. Small moves are handled by the trim coil alone.

## Parameters

All parameters live in `params/yig_params.m`, which builds the struct `YIG`.
**Every value is a placeholder.** Replace the coil L/R, C, `kH`, the magnet
distribution, `Bmax`, bandwidth, and the noise figures with measured values.
After editing, re-run `yig_setup` (or `YIG = yig_params();`). If an array
size changes, the blocks recompile automatically.

The test inputs are `ref_profile` (targets) and `env_profile`
(temperature and stray field). Both are defined in `yig_setup.m`.

## Files

```
yig_setup.m                 run first
params/yig_params.m         all parameters
control/yig_controller.m    <- your control law
plant/*.m                   plant blocks
sim/build_yig_model.m       (re)builds yig_tuning_model.slx
sim/yig_sim_script.m        Simulink-free simulation
sim/yig_plot_results.m      plots for either run type
```

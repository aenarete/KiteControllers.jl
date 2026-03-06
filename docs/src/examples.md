# Examples

The example scripts can be copied to a local `examples/` folder with:

```julia
using KiteControllers
KiteControllers.copy_examples()
```

Or install everything (examples + required packages) in one step:

```julia
KiteControllers.install_examples()
```

---

## autopilot.jl — Full Autopilot Simulation

The primary example demonstrating the complete control stack.

**Features:**
- Loads a kite model (`KPS4`) and kite control unit (`KCU`)
- Creates all settings (`WCSettings`, `FPCSettings`, `FPPSettings`)
- Builds a `SystemStateControl` and connects it to the kite model
- Runs a real-time simulation loop with a 3-D viewer

**Minimal usage pattern:**

```julia
using KiteControllers, KiteModels, KiteViewers

# ---- Settings ----
set  = load_settings("system")
wcs  = WCSettings(true; dt = 1/set.sample_freq)
fcs  = FPCSettings(true; dt = wcs.dt)
fpps = FPPSettings(true)

# ---- Kite model ----
kcu  = KCU(set)
kps4 = KPS4(kcu)

# ---- Controller ----
ssc = SystemStateControl(
    wcs, fcs, fpps;
    u_d0    = 0.01 * set.depower_offset,
    u_d     = 0.01 * set.depower,
    v_wind  = set.v_wind,
)

# ---- Start power production ----
on_autopilot(ssc)

# ---- Simulation loop ----
for step in 1:set.sim_time * set.sample_freq
    # integrate kite model one time step …
    next_step!(kps4; set_speed=nothing, dt=wcs.dt)

    # build a SysState from the kite model
    sys_state = SysState(kps4)

    # feed the controller
    on_new_systate(ssc, sys_state)

    # get winch command
    v_set = calc_v_set(ssc)
end
```

---

## parking_4p.jl — Parking Controller

Demonstrates how to bring a four-line kite to a stable parking position at
high elevation using `on_parking`.

```julia
using KiteControllers, KiteModels
# create ssc as above, then:
on_parking(ssc, tether_length=200.0)
```

---

## minipilot.jl — Minimal Autopilot

A stripped-down autopilot without a 3-D viewer, suitable for batch simulations.
Exposes the core control loop in ~50 lines.

---

## batch_pilot.jl — Batch Simulation

Runs multiple simulations back-to-back over a range of wind speeds and
records key statistics (mean power, reel-out speed, etc.) to an
[Apache Arrow](https://arrow.apache.org/) file in the `output/` folder.

**Run from the command line:**

```bash
julia --project=examples examples/batch_pilot.jl
```

Statistics are written to `output/batch-<project>_stats.yaml` and the full
time-series log to `output/batch-<project>.arrow`.

---

## tune_4p.jl — Controller Tuning

Interactive script for tuning the flight path controller gains (`p`, `i`, `d`, `gain`)
for a four-line kite.  Changes gain values and immediately re-runs the simulation to
visualise the effect.

---

## joystick.jl — Manual Joystick Control

Allows manual steering via a gamepad or joystick while the winch is controlled
automatically by the `WinchController`.  Useful for testing the winch loop in
isolation.

---

## Learning Resources

| File | Purpose |
|:-----|:--------|
| `autopilot.jl` | Full autopilot — best starting point |
| `minipilot.jl` | Minimal autopilot without viewer |
| `parking_4p.jl` | Parking sequence |
| `batch_pilot.jl` | Batch / sweep simulations |
| `tune_4p.jl` | Gain tuning |
| `joystick.jl` | Manual steering with automated winch |
| `plots.jl` | Post-processing and plotting of Arrow log files |
| `stats.jl` | Statistical analysis of batch results |

---

## Configuration Files

All settings are stored as YAML files in the `data/` folder.

| File | Settings struct |
|:-----|:----------------|
| `settings.yaml` | General kite / simulation settings (`KiteUtils.Settings`) |
| `system.yaml` | System-level parameters |
| `fpc_settings.yaml` | [`FPCSettings`](@ref) — flight path controller |
| `fpp_settings.yaml` | [`FPPSettings`](@ref) — flight path planner |
| `wc_settings.yaml` | `WCSettings` — winch controller |

Copy the default configuration files with:

```julia
KiteControllers.copy_control_settings()
```

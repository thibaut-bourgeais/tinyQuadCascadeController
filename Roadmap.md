# ESP32 Drone Flight Controller — Roadmap

This document outlines the development roadmap for building a **custom cascaded flight controller** on an ESP32 (PlatformIO + Arduino framework).  
The goal is to fly a rectangular quadrotor (130×100 mm motor spacing) with **position → trajectory → tracking** control.

---

# Milestone 0 — Project base & structure

**Goals**
- Clean PlatformIO project, FreeRTOS tasks, logging, centralized constants.

**Deliverables**
- `src/main.cpp` (FreeRTOS tasks or timers), `include/config.hpp`, `lib/` modules.

**Structure**
```

src/main.cpp
include/config.hpp
lib/
drivers/imu.hpp, esc.hpp
math/vec.hpp, mat.hpp, filters.hpp
control/rate.hpp, attitude.hpp, position.hpp, traj.hpp
mix/mixer\_rect.hpp, loopshaper.hpp

````

**Tests**
- Successful build, “rate task” ticks at 500–1000 Hz (LED blink or Serial counter).

---

# Milestone 1 — Drivers & timing

**Goals**
- IMU (I2C/SPI), ESC outputs (PWM/DShot via LEDC), high-rate timers.

**Deliverables**
- `drivers/imu.hpp` (read gyro/accel, calibration offsets),
- `drivers/esc.hpp` (write 4× PWM/DShot, arm/disarm, map 0–1 → µs).

**Snippets**
- ISR/Timer `rate_task()` at 500–1000 Hz,
- `esc.writeAll(u0,u1,u2,u3)`.

**Tests**
- IMU values readable, ESCs arm, motor command variation visible (without props).

---

# Milestone 2 — Attitude estimation

**Goals**
- Lightweight complementary filter / Mahony filter (no heavy quaternions).

**Deliverables**
- `filters.hpp`: 1st-order LPF + complementary filter,
- `state`: rotation matrix `R` (3×3) + angular velocity `omega` (rad/s), optional magnetometer yaw.

**Tests**
- Move frame by hand → stable angles, low latency, no divergence.

---

# Milestone 3 — Mixer + Loop-shaper

**Goals**
- Mixer for rectangular geometry (130×100 mm), desaturation, shaping (EMA + slew).

**Deliverables**
- `mix/mixer_rect.hpp` with precomputed `Binv` + `mixAndDesat(...)`,
- `loopshaper.hpp`: `motorEMAstep()` and `motorSlewStep()`.

**Tests (without props!)**
- Step torque on yaw → relative motor variations consistent (sanity signs),
- Force saturation >1 → check yaw sacrificed last.

---

# Milestone 4 — Rate loop (ω)

**Goals**
- PI (or lightweight PID) stabilizing when holding the frame.

**Deliverables**
- `control/rate.hpp`:
  - `tau = Kp*(w_des - w) + Ki*Iw + w×(Jw)`,
  - Anti-windup + clamp.

**Snippets (in `rate_task`)**
```cpp
Vec3 ew = w_des - w;
Iw += ew * dt; Iw = clamp(Iw, -IwMax, +IwMax);
Vec3 tau = KPw*ew + KIw*Iw + cross(w, J*w);
Vec4 u = mixAndDesat(tau, F_hover, geom); // F_hover = m*g
u = motorEMAstep(ema, u);
u = motorSlewStep(u, u, 0.02f);
esc.writeAll(u);
````

**Tests**

* Without props: PID output bounded (monitor τ).
* With props (tied down): small ω\_des steps (±30–60°/s). Tune `KPw` to near oscillation, back off 20–30%, then add `KIw`.

---

# Milestone 5 — Attitude loop (R/q)

**Goals**

* Geometric controller (SO(3)) → compute ω\_des.

**Deliverables**

* `control/attitude.hpp`:

  * `eR = 0.5*vee(Rd^T R - R^T Rd)`,
  * `w_des = -K_R * eR + [0,0,yawRateRef]`.

**Tests**

* Hold frame, impose Rd (small inclinations), ω follows without overshoot,
* Clamp ω\_des and max tilt angles.

---

# Milestone 6 — Altitude (Z) + collective

**Goals**

* PID on Z (or vertical acceleration) → compute **F\_des**, map via auto `hover_cmd`.

**Deliverables**

* `control/position.hpp` (altitude first),
* `collectiveDelta()` + `adaptHover()`.

**Tests**

* Auto-hover throttle (hands above drone) → drone maintains approximate height (\~fixed). Avoid outdoor wind initially.

---

# Milestone 7 — XY position → thrust vector + Rd

**Goals**

* Position PID (PDI) for XY with optional acceleration feed-forward,
* Convert \$\mathbf{t} = a\_{cmd} + g\hat z\$ → \$F^{des}, R\_{des}\$ (with yaw reference).

**Deliverables**

* `control/position.hpp`: `positionOuterLoop(...)`,
* `control/traj.hpp`: trapezoidal velocity generator per axis → \$(p\_r, v\_r, a\_r)\$.

**Tests**

* Start with low bandwidth XY (2–3 Hz), yaw fixed,
* Waypoint at 1–2 m: drone converges smoothly without oscillation.

---

# Milestone 8 — Trajectories & modes

**Goals**

* Waypoint queue, S-curve generation, loiter, land, arming/disarming, failsafe.

**Deliverables**

* `traj.hpp`: trajectory segments,
* `mode_manager.hpp`: flight modes (ARMED, DISARMED, LOITER, GOTO, LAND),
* `safety.hpp`: kill switch, RC/telemetry timeout, tilt >60°, low battery.

**Tests**

* Execute sequence of waypoints, loiter, return.

---

# Milestone 9 — Tuning & logging

**Goals**

* Tune gains with 50–100 Hz logs (UDP or Serial),
* Log: `t, p, v, R, w, pr, vr, Rd, w_des, F, u[4]`.

**Deliverables**

* `logger.hpp` (CSV/UDP),
* Optional Python plotting script.

**Tuning procedure**

1. **Rate**: increase `KPw` to edge of vibration, back off 20–30%, add `KIw` (5–10%/s).
2. **Attitude**: set `K_R` for \~150–300 °/s at 15–20° error.
3. **Altitude (Z)**: tune `Kp,Kd` for clean climb, small `Ki` if drift.
4. **XY**: start with `Kp=2–3`, `Kd=2–4 s⁻¹`, `Ki=0–0.3 s⁻¹`, then enable `a_r`.

---

## ✅ Safety checklists (go/no-go)

* **Before props**: IMU stable, τ bounded, desaturation OK, arming lock works.
* **First spool-up (tied down)**: rate loop responds, no high-pitched oscillation.
* **First hover**: altitude hold + attitude only, no XY.
* **XY control**: calm air, low bandwidth, tilt limit 25–30°.
* **Failsafe**: test kill switch, comms loss, low battery.

---

## Next concrete actions

1. Generate **empty headers** (signatures only) to progressively implement:

   * `drivers/esc.hpp`, `drivers/imu.hpp`
   * `control/rate.hpp`, `control/attitude.hpp`, `control/position.hpp`
   * `mix/mixer_rect.hpp`, `loopshaper.hpp`

2. Implement **500 Hz timer** in `main.cpp` (rate task), connect **mixer + loopshaper** (fixed command) to validate ESC pipeline.

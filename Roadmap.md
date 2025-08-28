Parfait. Voilà une **roadmap pragmatique** (façon sprints) pour aller jusqu’au vol autonome “position → trajectoire → suivi”, en restant léger pour l’ESP32 + PlatformIO. Chaque étape a : objectifs, livrables, tests, et snippets à poser (sans tout réécrire).

---

# Sprint 0 — Base projet & arborescence

**Objectifs**

* Projet PIO propre, tâches temps réel, logs, et constantes centralisées.

**Livrables**

* `src/main.cpp` (tasks FreeRTOS ou timers), `include/config.hpp`, `lib/` modules.

**Arbo**

```
src/main.cpp
include/config.hpp
lib/
  drivers/imu.hpp, esc.hpp
  math/vec.hpp, mat.hpp, filters.hpp
  control/rate.hpp, attitude.hpp, position.hpp, traj.hpp
  mix/mixer_rect.hpp, loopshaper.hpp
```

**Tests**

* Build OK, tick “rate” à 500–1000 Hz (LED ou counter Serial 1 Hz).

---

# Sprint 1 — Drivers & timing

**Objectifs**

* IMU (I2C/SPI), ESC (PWM/DShot via LEDC), timers high-rate.

**Livrables**

* `drivers/imu.hpp` (read gyro/accel, calib offsets),
* `drivers/esc.hpp` (write 4× PWM/DShot, arm/disarm, map 0–1 → µs).

**Snippets à placer**

* ISR/Timer `rate_task()` pour 500–1000 Hz.
* `esc.writeAll(u0,u1,u2,u3)`.

**Tests**

* Gyro/acc lisibles, ESC armables, variation commande visible sans hélices.

---

# Sprint 2 — Estimation d’attitude (léger)

**Objectifs**

* Filtre complémentaire/MAHONY simple (pas de quaternions lourds si tu veux).

**Livrables**

* `filters.hpp` : LPF 1er ordre, complementary filter.
* `state`: `R` (3×3) + `omega` (rad/s). Option yaw magnétomètre si dispo.

**Tests**

* Bouger la frame à la main → angles stables, latence faible, pas d’explosion.

---

# Sprint 3 — Mixer + Loop-shaper (déjà quasi prêt)

**Objectifs**

* Mixer rectangle 130×100 mm, désaturation, shaping (EMA+slew).

**Livrables**

* `mix/mixer_rect.hpp` avec ton `Binv` + `mixAndDesat(...)`.
* `loopshaper.hpp` : `motorEMAstep()` puis `motorSlewStep()`.

**Tests (sans hélices !)**

* Step de couple sur un axe (ex. yaw) → variation relative des 4 sorties est logique (sanity signs).
* Saturation : forcer >1 → vérif que la désat sacrifie **yaw** en dernier.

---

# Sprint 4 — Boucle **Rate** (ω) (la plus critique)

**Objectifs**

* PI (voire PID léger) stabilisant en tenant les bras.

**Livrables**

* `control/rate.hpp`:

  * `tau = Kp*(w_des-w) + Ki*Iw + w×(Jw)`
  * Anti-windup + clamp.

**Snippets (à mettre dans `rate_task`)**

```cpp
// Read omega (gyro), compute w_des from attitude loop (pour l’instant 0)
Vec3 ew = w_des - w;
Iw += ew * dt; Iw = clamp(Iw, -IwMax, +IwMax);
Vec3 tau = KPw*ew + KIw*Iw + cross(w, J*w);
Vec4 u = mixAndDesat(tau, F_hover, geom); // F_hover=m*g
u = motorEMAstep(ema, u);
u = motorSlewStep(u, u, 0.02f);
esc.writeAll(u);
```

**Tests**

* Sans hélices, PID ne “part” pas (monitor τ).
* Avec hélices **attachées** (enchaîné au sol), fais petits steps ω\_des (±30–60°/s). Ajuste `KPw` jusqu’avant vibration, puis ajoute `KIw`.

---

# Sprint 5 — Boucle **Attitude** (R/q)

**Objectifs**

* Contrôleur géométrique (SO(3)) → $w_{des}$.

**Livrables**

* `control/attitude.hpp`:

  * `eR = 0.5*vee(Rd^T R - R^T Rd)`
  * `w_des = -K_R * eR + [0,0,yawRateRef]`.

**Tests**

* Tenir le drone, imposer Rd (petites inclinaisons), ω suit sans overshoot.
* Limites : clamp ω\_des, clamp angles.

---

# Sprint 6 — Altitude (Z) + collectif

**Objectifs**

* PID en Z (ou en accélération verticale) → calculer **F\_des**, mapping via **hover\_cmd** auto.

**Livrables**

* `control/position.hpp` (Z d’abord),
* `collectiveDelta()` + `adaptHover()` (déjà fourni).

**Tests**

* Throttle “auto-hover” mains au-dessus → le drone tient vers \~fixe (en extérieur éviter vent).

---

# Sprint 7 — XY position → thrust vector + Rd

**Objectifs**

* Contrôleur PID (PDI) positionnel pour XY avec **option a\_r feed-forward**.
* Conversion $\mathbf{t} = a_cmd + g\hat z$ → $F^{des}, R_{des}$ (avec yaw ref).

**Livrables**

* `control/position.hpp`: `positionOuterLoop(...)` (déjà donné).
* `control/traj.hpp`: générateur “trapezoidal velocity” 1D par axe → $(p_r, v_r, a_r)$.

**Tests**

* D’abord XY **faible bande passante** (2–3 Hz), yaw fixe.
* Waypoint à 1–2 m : le drone converge proprement (pas d’oscillation).

---

# Sprint 8 — Trajectoires & gestion des modes

**Objectifs**

* Waypoint queue, S-curve, loiter, land, arm/disarm, failsafe.

**Livrables**

* `traj.hpp` (segments pos/vel/acc),
* `mode_manager.hpp` (ARMED, DISARMED, LOITER, GOTO, LAND),
* `safety.hpp` (kill switch, timeout RC/telemetry, tilt > 60°, batterie basse).

**Tests**

* Enchaîner 2–3 waypoints, pause (loiter), retour.

---

# Sprint 9 — Tuning & logs

**Objectifs**

* Ajuster gains avec logs 50–100 Hz (UDP ou Serial).
* Enregistrer : `t, p, v, R, w, pr, vr, Rd, w_des, F, u[4]`.

**Livrables**

* `logger.hpp` (CSV/UDP), script Python pour plots (optionnel).

**Procédure tuning rapide**

1. **Rate**: monte `KPw` jusqu’au bord de la vibration, recule 20–30 %, ajoute `KIw` (5–10 %/s).
2. **Attitude**: `K_R` pour \~150–300 °/s à 15–20° d’erreur.
3. **Z**: `Kp,Kd` pour montée propre, `Ki` petit si drift.
4. **XY**: commence avec `Kp=2–3`, `Kd=2–4 s⁻¹`, `Ki=0–0.3 s⁻¹`, puis active `a_r`.

---

## Checklists “go/no-go” (sécurité)

* **Avant hélices**: IMU stable, τ borné, désat OK, arming lock.
* **Première mise en gaz** (attaché): réponse rate, pas de sifflement fort.
* **Premier vol**: mode “altitude hold” + attitude, pas de position XY.
* **Position XY**: pas de rafales, BP basse, limites de tilt 25–30°.
* **Failsafe**: test kill switch, perte com, batterie basse.

---

## Prochaines actions (concrètes)

1. Je te génère les **headers vides** suivants (avec seulement les signatures) pour que tu colles tes implémentations au fur et à mesure :

   * `drivers/esc.hpp`, `drivers/imu.hpp`
   * `control/rate.hpp`, `control/attitude.hpp`, `control/position.hpp`
   * `mix/mixer_rect.hpp`, `loopshaper.hpp`
2. On pose le **timer 500 Hz** dans `main.cpp` (rate task) et on y branche **mixer + loopshaper** (en commande fixe) pour valider la chaîne jusqu’aux ESC.

Si tu veux, dis-moi si tu pars en **PWM 1000–2000 µs** ou **DShot600** et quelle IMU tu utilises (MPU/BMI/ICM). Je te fournis les **snippets exacts** pour `esc.writeAll()` et l’init IMU, plus le `platformio.ini` minimal.

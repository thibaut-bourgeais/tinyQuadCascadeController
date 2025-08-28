# ESP32 Drone Flight Controller

## 📌 About
This project is a **custom flight controller** for a quadrotor drone, running on an **ESP32 (Arduino framework, PlatformIO)**.  
Instead of being RC-driven, the controller receives **position targets** and generates trajectories to reach them.  
The architecture is **cascaded**: position → attitude → rate → motors, with a geometric attitude controller and robust motor mixing for a rectangular quad (130×100 mm).

The main goals are:
- Lightweight implementation suitable for ESP32.
- Modular C++ structure (`src/`, `lib/`).
- Real-time scheduling with **FreeRTOS** tasks (rate, attitude, position).
- Custom **mixer** for non-square geometry + desaturation logic.
- Position control with simple trajectory generation (S-curves / trapezoidal velocity).

---

## 🛠 Features (planned)
- IMU driver + complementary filter.
- High-rate **rate loop** (500–1000 Hz).
- **Attitude loop** (200 Hz) with SO(3) geometric control.
- **Position loop** (50 Hz) with optional acceleration feed-forward.
- Mixer + loop-shaper (EMA + slew-rate limiter).
- Altitude hold & hover adaptation.
- Trajectory generator (waypoints, trapezoidal velocity).
- Safety: arming/disarming, failsafe, logging.

---

## 🚀 Roadmap (summary)

- **0st Milsestone** – Project skeleton, PlatformIO, FreeRTOS tasks, config headers.  
- **1st Milsestone** – Drivers (IMU, ESC, timers).  
- **2cd Milsestone** – Attitude estimation (complementary filter).  
- **3rd Milsestone** – Mixer + loop-shaper.  
- **4th Milsestone** – Rate loop (PI + motor mapping).  
- **5th Milsestone** – Attitude loop (geometric).  
- **6th Milsestone** – Altitude control + hover adaptation.  
- **7th Milsestone** – XY position control → thrust vector + desired attitude.  
- **8th Milsestone** – Trajectories + mode management (LOITER, GOTO, LAND).  
- **9th Milsestone** – Logging & tuning.

---

## 📂 Project structure

# ESP32 Drone Flight Controller

## 📌 About
This project is a **custom flight controller** for a quadrotor drone, running on an **ESP32 (Arduino framework, PlatformIO)**.  
Instead of being RC-driven, the controller receives **position targets** and generates trajectories to reach them.  
The architecture is **cascaded**: position → attitude → rate → motors, with a geometric attitude controller and robust motor mixing for a rectangular quad (130×100 mm).

The main goals are:
- Lightweight implementation suitable for ESP32.
- Modular C++ structure (`src/`, `lib/`).
- Real-time scheduling with **FreeRTOS** tasks (rate, attitude, position).
- Custom **mixer** for non-square geometry + desaturation logic.
- Position control with simple trajectory generation (S-curves / trapezoidal velocity).

---

## 🛠 Features (planned)
- IMU driver + complementary filter.
- High-rate **rate loop** (500–1000 Hz).
- **Attitude loop** (200 Hz) with SO(3) geometric control.
- **Position loop** (50 Hz) with optional acceleration feed-forward.
- Mixer + loop-shaper (EMA + slew-rate limiter).
- Altitude hold & hover adaptation.
- Trajectory generator (waypoints, trapezoidal velocity).
- Safety: arming/disarming, failsafe, logging.

---

## 🚀 Roadmap (summary)

- **Sprint 0** – Project skeleton, PlatformIO, FreeRTOS tasks, config headers.  
- **Sprint 1** – Drivers (IMU, ESC, timers).  
- **Sprint 2** – Attitude estimation (complementary filter).  
- **Sprint 3** – Mixer + loop-shaper.  
- **Sprint 4** – Rate loop (PI + motor mapping).  
- **Sprint 5** – Attitude loop (geometric).  
- **Sprint 6** – Altitude control + hover adaptation.  
- **Sprint 7** – XY position control → thrust vector + desired attitude.  
- **Sprint 8** – Trajectories + mode management (LOITER, GOTO, LAND).  
- **Sprint 9** – Logging & tuning.

---

## 📂 Project structure

# Vehicle Physics Controller — Godot 4

A from-scratch arcade vehicle simulation built on `RigidBody3D`, featuring per-wheel suspension, slip-angle-based traction, and controllable drift mechanics.

---

## Demo

> *(Replace with a GIF or short screen recording of the car drifting)*

---

## How it works

The car is made up of four independent `RaycastWheel` nodes. Every physics tick each wheel:

1. **Casts a ray** downward to detect the ground contact point
2. **Applies suspension** via a spring-damper model — restoring force minus velocity-proportional damping, applied at the wheel's world position so weight transfer is physically derived rather than faked
3. **Computes a lateral slip angle** between the wheel's forward axis and its actual velocity vector
4. **Samples a grip curve** at that slip ratio to produce a lateral force magnitude — this is what keeps the car on the road during cornering, or lets it slide when grip is intentionally reduced

Drift is triggered by holding the handbrake, which smoothly lerps `rear_grip_state` down to a configurable fraction of normal grip. The car recovers gradually once released, preventing snap transitions back to full traction.

Counter-steer detection applies a corrective yaw torque when the player steers into a slide, scaled by the current grip value so the assistance weakens naturally as the car straightens up.

---

## Project structure

```
vehicle_controller.gd   — RigidBody3D subclass; main simulation loop
raycast_wheel.gd        — Per-wheel node: suspension geometry, grip curve, motor flag
```

---

## Key parameters

| Export | Default | Effect |
|---|---|---|
| `acceleration` | 12000 | Peak drive force (N) |
| `max_speed` | 150 | Speed at which accel_curve reaches zero |
| `rear_grip_mult` | 0.8 | Rear grip relative to front — lower promotes oversteer |
| `rear_handbrake_reduction` | 0.6 | Grip fraction retained on handbrake |
| `drift_yaw_torque` | 1200 | Counter-steer assist magnitude |
| `tire_max_turn_degrees` | 50 | Maximum front wheel lock angle |

Acceleration and grip are both curve-driven, so torque delivery and cornering force can be shaped precisely without touching code.

---

## What I would improve next

- **Ackermann steering geometry** — inner and outer wheels currently steer to the same angle
- **Pacejka-inspired tyre model** — replace the normalised slip-angle curve with a more physically grounded approach
- **Dynamic weight transfer** — suspension forces currently react to weight shift but drive force doesn't account for it
- **Input abstraction** — decouple input polling from the simulation so the same vehicle can be driven by AI or a replay system
- **Gearbox** — stepped torque ratios with an acceleration curve per gear

---

## Built with

- [Godot 4](https://godotengine.org/)

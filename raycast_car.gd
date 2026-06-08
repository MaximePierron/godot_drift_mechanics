## vehicle_controller.gd
##
## Arcade-style vehicle simulation built on top of [RigidBody3D].
## Each wheel is represented by a [RaycastWheel] node that handles suspension,
## acceleration, and lateral traction independently per tick.
##
## Drift behaviour is driven by reducing rear grip on handbrake input and
## applying a corrective yaw torque when the player counter-steers.
##
## Dependencies: RaycastWheel (custom resource/node), GPUParticles3D (skid marks)

extends RigidBody3D

# ─── Wheel references ────────────────────────────────────────────────────────

@export var wheels: Array[RaycastWheel]

# ─── Acceleration ────────────────────────────────────────────────────────────

@export var acceleration := 12000.0
@export var max_speed    := 150.0
## Maps speed ratio (0–1) to an acceleration multiplier, allowing torque to
## taper off naturally as the car approaches max_speed.
@export var accel_curve: Curve

# ─── Steering ────────────────────────────────────────────────────────────────

## Degrees per second the front wheels rotate toward the target angle.
@export var tire_turn_speed      := 10.0
## Maximum lock angle for the front wheels in either direction.
@export var tire_max_turn_degrees := 50.0

# ─── Grip ────────────────────────────────────────────────────────────────────

## Per-axle grip curves override the wheel's own grip_curve when assigned.
## Lateral force = curve.sample(slip_ratio) × grip_mult × (mass × gravity / 4).
@export var front_grip_curve: Curve
@export var rear_grip_curve:  Curve

@export var front_grip_mult := 1.0
## Rear grip is intentionally lower than front to promote oversteer.
@export var rear_grip_mult  := 0.8

# ─── Drift tuning ────────────────────────────────────────────────────────────

## Fraction of normal rear grip retained while the handbrake is held.
## Lower values make it easier to break traction but harder to recover.
@export var rear_handbrake_reduction := 0.6
## Magnitude of the yaw impulse applied while the player counter-steers during
## a drift. Scales with base_grip so it naturally weakens as grip returns.
@export var drift_yaw_torque := 1200.0

# ─── Visual effects ──────────────────────────────────────────────────────────

## One GPUParticles3D per wheel, indexed to match the [wheels] array.
@export var skid_marks: Array[GPUParticles3D]

# ─── Runtime state ───────────────────────────────────────────────────────────

var motor_input    := 0
var handbrake      := false   # true while the handbrake action is held
var is_slipping    := false   # true when rear lateral slip exceeds the threshold
## Smoothly interpolated multiplier applied on top of rear grip.
## Transitions from 1.0 (full grip) down to rear_handbrake_reduction on handbrake.
var rear_grip_state := 1.0


# ─── Lifecycle ───────────────────────────────────────────────────────────────

func _ready() -> void:
	# Lower the centre of mass to improve stability during hard cornering.
	center_of_mass_mode = CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = Vector3(0, -0.3, -0.1)


# ─── Input ───────────────────────────────────────────────────────────────────

func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("handbrake"):
		handbrake  = true
		is_slipping = true
	if event.is_action_released("handbrake"):
		handbrake = false

	if event.is_action_pressed("accelerate"):
		motor_input = 1
	if event.is_action_pressed("decelerate"):
		motor_input = -1
	if event.is_action_released("accelerate") or event.is_action_released("decelerate"):
		motor_input = 0


# ─── Physics ─────────────────────────────────────────────────────────────────

func _physics_process(delta: float) -> void:
	_basic_steering_rotation(delta)

	var id := 0
	for wheel in wheels:
		wheel.force_raycast_update()
		_do_single_wheel_suspension(wheel)
		_do_single_wheel_acceleration(wheel)
		_do_single_wheel_traction(wheel, id)
		id += 1

# ─── Helpers ─────────────────────────────────────────────────────────────────

## Returns the world-space velocity of an arbitrary point on the rigid body,
## accounting for both linear and angular contributions.
func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)


## Returns the signed lateral slip angle (radians) between the wheel's forward
## axis and its actual velocity direction. Positive = slipping outward.
func _compute_lateral_slip(wheel_ray: RaycastWheel) -> float:
	var wheel_forward := -wheel_ray.global_basis.z.normalized()
	var wheel_vel     := _get_point_velocity(wheel_ray.wheel.global_position)
	var vel           := Vector3(wheel_vel.x, 0.0, wheel_vel.z)

	if vel.length() < 0.01:
		return 0.0

	var cos_ang: float = clamp(wheel_forward.dot(vel.normalized()), -1.0, 1.0)
	var ang            := acos(cos_ang)
	var cross_y        := wheel_forward.cross(vel.normalized()).y
	return ang * sign(cross_y)


# ─── Per-wheel sub-steps ─────────────────────────────────────────────────────

func _basic_steering_rotation(delta: float) -> void:
	var raw_turn   := Input.get_axis("turn_right", "turn_left")
	var turn_input := raw_turn * tire_turn_speed

	# Double steering sensitivity during a drift so counter-steering is effective
	# even when the physics update is fighting the slip angle.
	if is_slipping:
		turn_input *= 2

	if turn_input != 0:
		$wheelFL.rotation.y = clampf(
			$wheelFL.rotation.y + turn_input * delta,
			deg_to_rad(-tire_max_turn_degrees), deg_to_rad(tire_max_turn_degrees))
		$wheelFR.rotation.y = clampf(
			$wheelFR.rotation.y + turn_input * delta,
			deg_to_rad(-tire_max_turn_degrees), deg_to_rad(tire_max_turn_degrees))
	else:
		# Return wheels to centre when no steering input is held.
		$wheelFL.rotation.y = move_toward($wheelFL.rotation.y, 0, deg_to_rad(tire_max_turn_degrees) * delta)
		$wheelFR.rotation.y = move_toward($wheelFR.rotation.y, 0, deg_to_rad(tire_max_turn_degrees) * delta)


func _do_single_wheel_traction(wheel_ray: RaycastWheel, id: int) -> void:
	if not wheel_ray.is_colliding():
		return

	var steer_side_dir := wheel_ray.global_basis.x.normalized()
	var tire_vel       := _get_point_velocity(wheel_ray.wheel.global_position)
	var steering_x_vel := steer_side_dir.dot(tire_vel)

	var speed      : float = max(tire_vel.length(), 0.01)
	var slip_angle         := absf(_compute_lateral_slip(wheel_ray))
	# Normalise slip to a 0–1 curve sample; 30° is treated as full slip.
	var slip_sample: float = clamp(slip_angle / deg_to_rad(30.0), 0.0, 1.0)

	# Resolve which grip curve to use: per-axle export overrides wheel default.
	var sample_curve: Curve = null
	if wheel_ray.is_rear:
		sample_curve = rear_grip_curve if rear_grip_curve else wheel_ray.grip_curve
	else:
		sample_curve = front_grip_curve if front_grip_curve else wheel_ray.grip_curve

	if sample_curve == null:
		push_error("No grip curve found for wheel " + str(id))
		return

	var base_grip := sample_curve.sample_baked(slip_sample)
	base_grip    *= rear_grip_mult if wheel_ray.is_rear else front_grip_mult

	# Gradually reduce rear grip on handbrake; recover more slowly during a slide
	# so the car doesn't snap back to full traction instantly.
	if wheel_ray.is_rear:
		if handbrake:
			rear_grip_state = lerp(rear_grip_state, rear_handbrake_reduction, 6.0 * get_physics_process_delta_time())
		elif is_slipping:
			rear_grip_state = lerp(rear_grip_state, 0.7, 0.3 * get_physics_process_delta_time())
		else:
			rear_grip_state = lerp(rear_grip_state, 1.0, 0.5 * get_physics_process_delta_time())

		base_grip *= rear_grip_state

	# ── Skid marks ──────────────────────────────────────────────────────────
	skid_marks[id].global_position = wheel_ray.get_collision_point() + Vector3.UP * 0.01
	skid_marks[id].look_at(skid_marks[id].global_position + global_basis.z)

	var grip_factor := absf(steering_x_vel) / speed
	skid_marks[id].emitting = handbrake or grip_factor >= 0.2

	# ── Lateral (cornering) force ────────────────────────────────────────────
	# Modelled as a spring opposing sideways sliding, scaled by the grip curve
	# output and a quarter of the car's total weight.
	var gravity : float = ProjectSettings.get_setting("physics/3d/default_gravity")
	var x_force := -steer_side_dir * steering_x_vel * base_grip * (mass * gravity / 4.0)

	# ── Longitudinal drag ────────────────────────────────────────────────────
	# Small forward-direction damping keeps the car from coasting indefinitely.
	var f_vel    := -wheel_ray.global_basis.z.dot(tire_vel)
	var z_force  := global_basis.z * f_vel * 0.06 * (mass * gravity / 4.0)

	var force_pos := wheel_ray.wheel.global_position - global_position
	apply_force(x_force, force_pos)
	apply_force(z_force, force_pos)

	# Rear wheel slip detection — triggers drift assistance logic above.
	if wheel_ray.is_rear and absf(steering_x_vel) / speed > 0.4:
		is_slipping = true

	# ── Counter-steer yaw torque ─────────────────────────────────────────────
	# Adds a corrective rotation impulse while the player steers during a drift,
	# making slides feel controllable rather than chaotic.
	var steer_input := Input.get_axis("turn_right", "turn_left")
	if (handbrake or is_slipping) and abs(steer_input) > 0.01:
		var yaw_dir   : float = sign(steer_input)
		var slip_boost        := 1.5 if is_slipping else 1.0
		# Rear wheels contribute more yaw than fronts to keep pivoting natural.
		var axle_weight       := 1.0 if wheel_ray.is_rear else 0.4
		var torque            := Vector3.UP * yaw_dir * drift_yaw_torque * base_grip * slip_boost * axle_weight
		apply_torque_impulse(torque * get_physics_process_delta_time())

	# Passive lateral damping when fully gripped — suppresses oscillation on
	# straight-line driving without affecting active slip states.
	if not is_slipping:
		var lateral_vel := steer_side_dir * steer_side_dir.dot(tire_vel)
		apply_force(-lateral_vel * 0.1 * mass, force_pos)


func _do_single_wheel_acceleration(wheel_ray: RaycastWheel) -> void:
	# Spin the wheel mesh to match the car's forward speed (visual only).
	var forward_dir := -wheel_ray.global_basis.z
	var vel         := forward_dir.dot(linear_velocity)
	wheel_ray.wheel.rotate_x(-vel * get_process_delta_time() / wheel_ray.wheel_radius)

	if not wheel_ray.is_colliding() or not wheel_ray.is_motor or not motor_input:
		return

	var contact   := wheel_ray.wheel.global_position
	var force_pos := contact - global_position

	# Sample the acceleration curve so peak torque tapers toward max_speed.
	var speed_ratio: float = clamp(abs(vel) / max_speed, 0.0, 1.0)
	var accel              := accel_curve.sample_baked(speed_ratio) if accel_curve else 1.0
	var force_vector       := forward_dir * acceleration * motor_input * accel

	# Apply a progressive torque boost to the rear wheels during a slide so the
	# player can maintain momentum through a drift with throttle input alone.
	if wheel_ray.is_rear and is_slipping:
		var boost: float = lerp(1.0, 1.6, float(abs(motor_input)))
		force_vector    *= boost

	# Project onto the contact surface so bumpy terrain doesn't inject vertical force.
	var projected_force := force_vector - wheel_ray.get_collision_normal() * force_vector.dot(wheel_ray.get_collision_normal())
	apply_force(projected_force, force_pos)


func _do_single_wheel_suspension(wheel_ray: RaycastWheel) -> void:
	if not wheel_ray.is_colliding():
		return

	wheel_ray.target_position.y = -(wheel_ray.rest_dist + wheel_ray.wheel_radius + wheel_ray.over_extend)

	var contact       := wheel_ray.get_collision_point()
	var spring_up_dir := wheel_ray.global_transform.basis.y
	var spring_len    := wheel_ray.global_position.distance_to(contact) - wheel_ray.wheel_radius
	var offset        := wheel_ray.rest_dist - spring_len

	# Move the visual wheel mesh to follow the suspension travel.
	wheel_ray.wheel.position.y = -spring_len

	# Classic spring-damper: restoring force minus velocity-proportional damping.
	var spring_force      := wheel_ray.spring_strength * offset
	var world_vel         := _get_point_velocity(contact)
	var relative_vel      := spring_up_dir.dot(world_vel)
	var spring_damp_force := wheel_ray.spring_damping * relative_vel
	var force_vector      := (spring_force - spring_damp_force) * wheel_ray.get_collision_normal()

	var force_pos_offset := wheel_ray.wheel.global_position - global_position
	apply_force(force_vector, force_pos_offset)


# ─── Roadmap ─────────────────────────────────────────────────────────────────
#
# HANDLING
#   [ ] Reset is_slipping when grip should be back
#   [ ] Ackermann steering geometry — inner/outer wheel angles during tight turns
#   [ ] Pacejka-inspired tyre model to replace the current slip-angle approximation
#   [ ] Dynamic weight transfer under braking, acceleration, and cornering
#   [ ] Passive drift without handbrake — tune curves so oversteer is achievable
#       on throttle alone
#
# DRIVETRAIN
#   [ ] Gearbox with configurable ratios
#   [ ] Improved braking — dedicated deceleration force, brake balance front/rear
#
# ARCHITECTURE
#   [ ] Decouple input from simulation so the car can be driven by AI or replays
#   [ ] Unit tests for suspension spring and traction force calculations
#
# CONTENT
#   [ ] Car selection with per-vehicle stat profiles
#   [ ] Per-car audio (engine, tyre squeal, impact)

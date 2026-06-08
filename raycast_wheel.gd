extends RayCast3D
class_name RaycastWheel

@export var spring_strength := 12000.0
@export var spring_damping := 1500.0
@export var rest_dist := 0.5
@export var over_extend := 0.2
@export var wheel_radius := 0.4
@export var is_motor := false
@export var is_rear := false
@export var grip_curve : Curve

@onready var wheel: Node3D = get_child(0)

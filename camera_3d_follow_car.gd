extends Camera3D

@export var target: NodePath
@export var offset := Vector3(0, 3, 8) # height, distance back
@export var follow_speed := 5.0        # higher = snappier follow

var car: Node3D

func _ready():
	car = get_node(target)

func _process(delta):
	if not car:
		return
	
	# Desired camera position (behind car in its local space)
	var target_pos = car.global_transform.origin
	target_pos += -car.global_transform.basis.z * offset.z  # behind
	target_pos += Vector3.UP * offset.y                    # above
	
	# Smooth follow
	global_transform.origin = global_transform.origin.lerp(target_pos, follow_speed * delta)
	
	# Always look at the car
	look_at(car.global_transform.origin, Vector3.UP)

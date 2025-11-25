package demos

///////////////////////////////// RigidBodyType
// (oimo/dynamics/rigidbody/RigidBodyType.go)
// The list of a rigid body's motion types.

type RigidBodyType int

const (
	_DYNAMIC RigidBodyType = iota
	_STATIC
	_KINEMATIC
)

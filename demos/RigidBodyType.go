package demos

///////////////////////////////// RigidBodyType
// (oimo/dynamics/rigidbody/RigidBodyType.go)
// The list of a rigid body's motion types.

type RigidBodyType int

const (
	RigidBodyType_DYNAMIC RigidBodyType = iota
	RigidBodyType_STATIC
	RigidBodyType_KINEMATIC
)

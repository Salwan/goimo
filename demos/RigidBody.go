package demos

import "github.com/g3n/engine/math32"

// ////////////////////// RigidBody
// (oimo/dynamics/rigidbody/RigidBody.go)
// A rigid body. To add a rigid body to a physics world, create a `RigidBody` instance, create and add shapes via `RigidBody.addShape`, and add the rigid body to the physics world through `World.addRigidBody`. Rigid bodies have three motion types: dynamic, static, and kinematic. See `RigidBodyType` for details of motion types.

type RigidBody struct {
	next *RigidBody
	prev *RigidBody

	shapeList     Shape
	shapeListLast Shape
	numShapes     int

	vel    Vec3
	angVel Vec3

	pseudoVel    Vec3
	angPseudoVel Vec3

	pTransform Transform
	transform  Transform

	_type int

	sleepTime                        float64
	sleeping                         bool
	autoSleep                        bool
	sleepingVelocityThreshold        float64
	sleepingAngularVelocityThreshold float64
	sleepingTimeThreshold            float64

	mass                            float64
	invMass                         float64
	localInertia                    Mat3
	rotFactor                       Vec3
	invLocalInertia                 Mat3
	invLocalInertiaWithoutRotFactor Mat3
	invInertia                      Mat3

	linearDamping  float64
	angularDamping float64

	force  Vec3
	torque Vec3

	linearContactImpulse  Vec3
	angularContactImpulse Vec3

	world *World

	contactLinkList     ContactLink
	contactLinkListLast ContactLink
	numContactLinks     int

	jointLinkList     JointLink
	jointLinkListLast JointLink
	numJointLinks     int

	addedToIsland bool
	gravityScale  float64

	userData any
}

func NewRigidBody(config *RigidBodyConfig) *RigidBody {
	return &RigidBody{}
}

func (rb *RigidBody) AddShape(shape *Shape)                             {}
func (rb *RigidBody) SetAngularVelocity(angularVelocity math32.Vector3) {}

package demos

////////////////////////////////////////// JointConfig
// (oimo/dynamics/constraint/joint/JointConfig.go)
// A joint configuration is used for constructions of various joints. An instance of any kind of the joint configurations can safely be reused.

type JointConfig struct {
	// The first rigid body attached to the joint.
	rigidBody1 *RigidBody

	// The second rigid body attached to the joint.
	rigidBody2 *RigidBody

	// The local position of the first rigid body's anchor point.
	localAnchor1 Vec3

	// The local position of the second rigid body's anchor point.
	localAnchor2 Vec3

	// Whether to allow the connected rigid bodies to collide each other.
	allowCollision bool

	// The type of the constraint solver for the joint.
	// See `ConstraintSolverType` for details.
	solverType ConstraintSolverType

	// The type of the position correction algorithm for the joint.
	// See `PositionCorrectionAlgorithm` for details.
	positionCorrectionAlgorithm PositionCorrectionAlgorithm

	// The joint will be destroyed when magnitude of the constraint force exceeds the value.
	// Set `0` for unbreakable joints.
	breakForce float64

	// The joint will be destroyed when magnitude of the constraint torque exceeds the value.
	// Set `0` for unbreakable joints.
	breakTorque float64
}

func NewJointConfig() *JointConfig {
	return &JointConfig{
		solverType:                  Settings.DefaultJointConstraintSolverType,
		positionCorrectionAlgorithm: Settings.DefaultJointPositionCorrectionAlgorithm,
	}
}

func (j *JointConfig) init(rb1, rb2 *RigidBody, worldAnchor Vec3) {
	j.rigidBody1 = rb1
	j.rigidBody2 = rb2
	j.rigidBody1.GetLocalPointTo(worldAnchor, &j.localAnchor1)
	j.rigidBody2.GetLocalPointTo(worldAnchor, &j.localAnchor2)
}

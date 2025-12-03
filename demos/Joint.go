package demos

// //////////////////////// Joint
// (oimo/dynamics/constraint/joint/Joint.go)
// The base class of joints. Joints are used to connect two rigid bodies in various ways. See `JointType` for all types of joints.

type Joint struct {
	b1 *RigidBody
	b2 *RigidBody

	link1                       *JointLink
	link2                       *JointLink
	positionCorrectionAlgorithm PositionCorrectionAlgorithm

	allowCollision bool

	prev  *Joint
	next  *Joint
	world *World

	localAnchor1    Vec3
	localAnchor2    Vec3
	relativeAnchor1 Vec3
	relativeAnchor2 Vec3
	anchor1         Vec3
	anchor2         Vec3

	localBasisX1 Vec3
	localBasisY1 Vec3
	localBasisZ1 Vec3
	localBasisX2 Vec3
	localBasisY2 Vec3
	localBasisZ2 Vec3

	basisX1 Vec3
	basisY1 Vec3
	basisZ1 Vec3
	basisX2 Vec3
	basisY2 Vec3
	basisZ2 Vec3

	impulses []JointImpulse

	// computed in constraint solver
	appliedForce  Vec3
	appliedTorque Vec3

	breakForce  float64
	breakTorque float64

	_type int

	solver IConstraintSolver

	// Extra field that users can use for their own purposes.
	userData any
}

func NewJoint(config *JointConfig, _type int) *Joint {
	j := &Joint{
		positionCorrectionAlgorithm: Settings.DefaultJointPositionCorrectionAlgorithm,
		_type:                       _type,
		b1:                          config.rigidBody1,
		b2:                          config.rigidBody2,
		allowCollision:              config.allowCollision,
		breakForce:                  config.breakForce,
		breakTorque:                 config.breakTorque,
		localAnchor1:                config.localAnchor1,
		localAnchor2:                config.localAnchor2,
		impulses:                    make([]JointImpulse, Settings.MaxJacobianRows),
	}

	j.link1 = NewJointLink(j)
	j.link2 = NewJointLink(j)

	switch config.solverType {
	case ConstraintSolverType_DIRECT:
		j.solver = NewDirectJointConstraintSolver(j)
	case ConstraintSolverType_ITERATIVE:
		j.solver = NewPgsJointConstraintSolver(j)
	}

	return j
}

// TODO

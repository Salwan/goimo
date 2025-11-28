package demos

// /////////////////////////////////////// ContactConstraint
// (oimo/dynamics/constraint/contact/ContactConstraint.go)
// A contact constraint provides collision information for a contact constraint solver. This holds a contact manifold, which has some contact points, contact normals, and contact impulses. See `Manifold` for more information. (size>64)
type ContactConstraint struct {
	positionCorrectionAlgorithm PositionCorrectionAlgorithm

	manifold *Manifold

	s1          *Shape
	s2          *Shape
	tf1         *Transform
	tf2         *Transform
	invM1       float64
	invM2       float64
	friction    float64
	restitution float64

	invI1 *Mat3
	invI2 *Mat3

	b1 *RigidBody
	b2 *RigidBody

	solver IConstraintSolver
}

func NewContactConstraint(manifold *Manifold) *ContactConstraint {
	cc := &ContactConstraint{
		manifold: manifold,
	}
	cc.solver = NewPgsContactConstraintSolver(cc)
	return cc
}

func (cc *ContactConstraint) attach(s1, s2 *Shape) {
	cc.s1 = s1
	cc.s2 = s2
	cc.b1 = s1.rigidBody
	cc.b2 = s2.rigidBody
	cc.tf1 = &cc.b1.transform
	cc.tf2 = &cc.b2.transform
}

func (cc *ContactConstraint) detach() {
	cc.s1, cc.s2, cc.b1, cc.b2, cc.tf1, cc.tf2 = nil, nil, nil, nil, nil, nil
}

func (cc *ContactConstraint) isTouching() bool {
	for i := range cc.manifold.numPoints {
		if cc.manifold.points[i].depth >= 0 {
			return true
		}
	}
	return false
}

// TODO

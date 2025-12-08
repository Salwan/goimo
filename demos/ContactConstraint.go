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

// --- internal ---

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

func (self *ContactConstraint) getVelocitySolverInfo(timeStep TimeStep, info *ContactSolverInfo) {
	info.b1 = self.b1
	info.b2 = self.b2

	normal := self.manifold.normal
	tangent := self.manifold.tangent
	binormal := self.manifold.binormal

	friction := MathUtil.Sqrt(self.s1.friction * self.s2.friction)
	restitution := MathUtil.Sqrt(self.s1.restitution * self.s2.restitution)

	num := self.manifold.numPoints
	info.numRows = 0

	// unused
	// posDiff := self.tf1.position.Sub(self.tf2.position)

	for i := range num {
		p := self.manifold.points[i]

		if p.depth < 0 {
			p.disabled = true

			// clear accumulated impulses
			p.impulse.clear()

			// skip separated points
			continue
		} else {
			p.disabled = false
		}

		row := info.rows[info.numRows]
		info.numRows++

		row.friction = friction
		row.cfm = 0 // TODO(Oimo): implement APIs for CFM setting?

		// set Jacobian
		j := row.jacobianN
		j.lin1 = normal
		j.lin2 = normal
		j.ang1 = p.relPos1.Cross(normal)
		j.ang2 = p.relPos2.Cross(normal)

		j = row.jacobianT
		j.lin1 = tangent
		j.lin2 = tangent
		j.ang1 = p.relPos1.Cross(tangent)
		j.ang2 = p.relPos2.Cross(tangent)

		j = row.jacobianB
		j.lin1 = binormal
		j.lin2 = binormal
		j.ang1 = p.relPos1.Cross(binormal)
		j.ang2 = p.relPos2.Cross(binormal)

		// compute relative velocity
		j = row.jacobianN
		rvn := (j.lin1.Dot(self.b1.vel) + j.ang1.Dot(self.b1.angVel)) - (j.lin2.Dot(self.b2.vel) + j.ang2.Dot(self.b2.angVel))

		// disable bounce for warm-started contacts
		if rvn < -Settings.ContactEnableBounceThreshold && !p.warmStarted {
			row.rhs = -rvn * restitution
		} else {
			row.rhs = 0
		}

		// set minimum RHS for baumgarte position correction
		if self.positionCorrectionAlgorithm == PositionCorrectionAlgorithm_BAUMGARTE {
			if p.depth > Settings.LinearSlop {
				minRhs := (p.depth - Settings.LinearSlop) * Settings.VelocityBaumgarte * timeStep.InvDt
				if row.rhs < minRhs {
					row.rhs = minRhs
				}
			}
		}

		// reset impulses if warm starting is disabled
		if !p.warmStarted {
			p.impulse.clear()
		}

		row.impulse = &p.impulse
	}
}

func (self *ContactConstraint) getPositionSolverInfo(info *ContactSolverInfo) {
	info.b1 = self.b1
	info.b2 = self.b2

	normal := self.manifold.normal

	num := self.manifold.numPoints
	info.numRows = 0

	for i := range num {
		p := self.manifold.points[i]

		if p.disabled {
			continue // skip disabled points
		}

		row := info.rows[info.numRows]
		info.numRows++

		// set normal jacobian
		j := row.jacobianN
		j.lin1 = normal
		j.lin2 = normal
		j.ang1 = p.relPos1.Cross(normal)
		j.ang2 = p.relPos2.Cross(normal)

		row.rhs = p.depth - Settings.LinearSlop
		if row.rhs < 0 {
			row.rhs = 0
		}

		row.impulse = &p.impulse
	}
}

// !! don't forget to call this from constraint solvers !!
func (self *ContactConstraint) syncManifold() {
	self.manifold.updateDepthsAndPositions(self.tf1, self.tf2)
}

// --- public ---

// Returns the first shape of the contact.
func (self *ContactConstraint) getShape1() *Shape {
	return self.s1
}

// Returns the second shape of the contact.
func (self *ContactConstraint) getShape2() *Shape {
	return self.s2
}

// Returns the contact manifold.
func (self *ContactConstraint) getManifold() *Manifold {
	return self.manifold
}

// Returns whether the two rigid bodies are touching.
func (cc *ContactConstraint) IsTouching() bool {
	for i := range cc.manifold.numPoints {
		if cc.manifold.points[i].depth >= 0 {
			return true
		}
	}
	return false
}

package demos

///////////////////////////////////////// ContactSolverInfoRow
// (oimo/dynamics/constraint/info/contact/ContactSolverInfoRow.go)
// Internal class.

type ContactSolverInfoRow struct {
	// Used for both velocity and position solver.
	jacobianN *JacobianRow

	// Used for velocity solver.
	jacobianT *JacobianRow

	// Used for velocity solver.
	jacobianB *JacobianRow

	// Used for both velocity and position solver.
	rhs float64

	// Used for velocity solver.
	cfm float64

	// Used for velocity solver.
	friction float64

	// Used for both velocity and position solver.
	impulse *ContactImpulse
}

func NewContactSolverInfoRow() *ContactSolverInfoRow {
	return &ContactSolverInfoRow{
		jacobianN: NewJacobianRow(),
		jacobianT: NewJacobianRow(),
		jacobianB: NewJacobianRow(),
	}
}

func (self *ContactSolverInfoRow) Clear() {
	self.jacobianN.Clear()
	self.jacobianT.Clear()
	self.jacobianB.Clear()
	self.rhs, self.cfm, self.friction = 0.0, 0.0, 0.0
	self.impulse = nil
}

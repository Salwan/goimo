package demos

/////////////////////////////////////////////// JointSolverInfoRow
// (oimo/dynamics/constraint/info/joint/JointSolverInfoRow.go)
// Internal class.

type JointSolverInfoRow struct {
	// Used for both velocity and position solver.
	jacobian *JacobianRow

	// Used for both velocity and position solver.
	rhs float64

	// Used for velocity solver.
	cfm float64

	// Used for both velocity and position solver.
	minImpulse float64

	// Used for both velocity and position solver.
	maxImpulse float64

	// Used for velocity solver.
	motorSpeed float64

	// Used for velocity solver.
	motorMaxImpulse float64

	// Used for both velocity and position solver.
	impulse *JointImpulse
}

func NewJointSolverInfoRow() *JointSolverInfoRow {
	return &JointSolverInfoRow{
		jacobian: NewJacobianRow(),
	}
}

func (j *JointSolverInfoRow) clear() {
	j.jacobian.clear()
	j.rhs, j.cfm = 0, 0
	j.minImpulse, j.maxImpulse, j.motorSpeed, j.motorMaxImpulse = 0, 0, 0, 0
	j.impulse = nil
}

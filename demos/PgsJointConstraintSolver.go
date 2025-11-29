package demos

///////////////////////////////////////////////// PgsJointConstraintSolver
// (oimo/dynamics/constraint/solver/pgs/PgsJointConstraintSolver.go)
// A joint constraint solver using projected Gauss-Seidel (sequential impulse).

type PgsJointConstraintSolver struct {
	*ConstraintSolver

	joint    *Joint
	info     *JointSolverInfo
	massData []*JointSolverMassDataRow
}

func NewPgsJointConstraintSolver(joint *Joint) *PgsJointConstraintSolver {
	p := &PgsJointConstraintSolver{
		ConstraintSolver: NewConstraintSolver(),

		joint:    joint,
		info:     NewJointSolverInfo(),
		massData: make([]*JointSolverMassDataRow, Settings.MaxJacobianRows),
	}

	for i := range len(p.massData) {
		p.massData[i] = NewJointSolverMassDataRow()
	}

	return p
}

// TODO

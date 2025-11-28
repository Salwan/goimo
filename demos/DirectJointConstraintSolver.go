package demos

///////////////////////////////////////////////// DirectJointConstraintSolver
// (oimo/dynamics/constraint/solver/direct/DirectJointConstraintSolver.go)
// The direct solver of a mixed linear complementality problem (MLCP) for joint constraints.

type DirectJointConstraintSolver struct {
	*ConstraintSolver

	info *JointSolverInfo
	//massData []*JointSolverMassDataRow

	// TODO
}

func NewDirectJointConstraintSolver(joint *Joint) *DirectJointConstraintSolver {
	return &DirectJointConstraintSolver{}
}

// TODO

package demos

// ///////////////////////////////////////// PgsContactConstraintSolver
// (oimo/dynamics/constraint/solver/pgs/PgsContactConstraintSolver.go)
// A contact constraint solver using projected Gauss-Seidel (sequential impulse).
type PgsContactConstraintSolver struct {
	*ConstraintSolver

	constraint *ContactConstraint

	info *ContactSolverInfo

	massData []*ContactSolverMassDataRow
}

func NewPgsContactConstraintSolver(constraint *ContactConstraint) *PgsContactConstraintSolver {
	pgs := &PgsContactConstraintSolver{
		ConstraintSolver: NewConstraintSolver(),
		constraint:       constraint,
		info:             NewContactSolverInfo(),
		massData:         make([]*ContactSolverMassDataRow, Settings.MaxManifoldPoints),
	}
	for i := range len(pgs.massData) {
		pgs.massData[i] = NewContactSolverMassDataRow()
	}
	return pgs
}

// TODO

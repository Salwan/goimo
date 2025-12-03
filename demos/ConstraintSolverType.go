package demos

//////////////////////// ConstraintSolverType
// (oimo/dynamics/constraint/solver/ConstraintSolverType.go)
// The list of the constraint solvers.

type ConstraintSolverType int

const (
	ConstraintSolverType_ITERATIVE ConstraintSolverType = iota
	ConstraintSolverType_DIRECT
)

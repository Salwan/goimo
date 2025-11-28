package demos

// ///////////////////////// ConstraintSolver
// (oimo/dynamics/constraint/ConstraintSolver.go)
// The base class of all constarint solvers.

type IConstraintSolver interface{}

type ConstraintSolver struct {
	b1            *RigidBody
	b2            *RigidBody
	addedToIsland bool
}

func NewConstraintSolver() *ConstraintSolver {
	return &ConstraintSolver{}
}

// TODO

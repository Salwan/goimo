package demos

// ///////////////////////// ConstraintSolver
// (oimo/dynamics/constraint/ConstraintSolver.go)
// The base class of all constarint solvers.

type ConstraintSolver struct {
	b1            *RigidBody
	b2            *RigidBody
	addedToIsland bool
}

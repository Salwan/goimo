package demos

// ///////////////////////// ConstraintSolver
// (oimo/dynamics/constraint/ConstraintSolver.go)
// The base class of all constarint solvers.

type IConstraintSolver interface {
	// Prepares for velocity iteration. Time step information `timeStep` is given for computing time-depending data.
	PreSolveVelocity(timeStep TimeStep)

	// Applies initial impulses.
	WarmStart(timeStep TimeStep)

	// Performs single velocity iteration.
	SolveVelocity()

	// Performs post-processes of velocity part. Time step information `timeStep` is given for computing time-depending data.
	PostSolveVelocity(timeStep TimeStep)

	// Prepares for position iteration (split impulse or nonlinear Gauss-Seidel). Time step information `timeStep` is given for computing time-depending data.
	// This may not be called depending on position correction algorithm.
	PreSolvePosition(timeStep TimeStep)

	// Performs single position iteration (split impulse)
	SolvePositionSplitImpulse()

	// Performs single position iteration (nonlinear Gauss-Seidel)
	SolvePositionNgs(timeStep TimeStep)

	// Performs post-processes.
	PostSolve()

	GetAddedToIsland() bool
	SetAddedToIsland(b bool)
}

type ConstraintSolver struct {
	b1            *RigidBody
	b2            *RigidBody
	addedToIsland bool
}

func NewConstraintSolver() *ConstraintSolver {
	return &ConstraintSolver{}
}

func (cs *ConstraintSolver) PreSolveVelocity(timeStep TimeStep) {
	panic("abstract call")
}

func (cs *ConstraintSolver) WarmStart(timeStep TimeStep) {
	panic("abstract call")
}

func (cs *ConstraintSolver) SolveVelocity() {
	panic("abstract call")
}

func (cs *ConstraintSolver) PostSolveVelocity(timeStep TimeStep) {
	panic("abstract call")
}

func (cs *ConstraintSolver) PreSolvePosition(timeStep TimeStep) {
	panic("abstract call")
}

func (cs *ConstraintSolver) SolvePositionSplitImpulse() {
	panic("abstract call")
}

func (cs *ConstraintSolver) SolvePositionNgs(timeStep TimeStep) {
	panic("abstract call")
}

func (cs *ConstraintSolver) PostSolve() {
	panic("abstract call")
}

func (cs *ConstraintSolver) GetAddedToIsland() bool {
	return cs.addedToIsland
}

func (cs *ConstraintSolver) SetAddedToIsland(b bool) {
	cs.addedToIsland = b
}

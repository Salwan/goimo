package demos

// ///////////////////////// ConstraintSolver
// (oimo/dynamics/constraint/ConstraintSolver.go)
// The base class of all constarint solvers.

type IConstraintSolver interface {
	// Prepares for velocity iteration. Time step information `timeStep` is given for computing time-depending data.
	preSolveVelocity(timeStep TimeStep)

	// Applies initial impulses.
	warmStart(timeStep TimeStep)

	// Performs single velocity iteration.
	solveVelocity()

	// Performs post-processes of velocity part. Time step information `timeStep` is given for computing time-depending data.
	postSolveVelocity(timeStep TimeStep)

	// Prepares for position iteration (split impulse or nonlinear Gauss-Seidel). Time step information `timeStep` is given for computing time-depending data.
	// This may not be called depending on position correction algorithm.
	preSolvePosition(timeStep TimeStep)

	// Performs single position iteration (split impulse)
	solvePositionSplitImpulse()

	// Performs single position iteration (nonlinear Gauss-Seidel)
	solvePositionNgs(timeStep TimeStep)

	// Performs post-processes.
	postSolve()
}

type ConstraintSolver struct {
	b1            *RigidBody
	b2            *RigidBody
	addedToIsland bool
}

func NewConstraintSolver() *ConstraintSolver {
	return &ConstraintSolver{}
}

func (cs *ConstraintSolver) preSolveVelocity(timeStep TimeStep) {}

func (cs *ConstraintSolver) warmStart(timeStep TimeStep) {}

func (cs *ConstraintSolver) solveVelocity() {}

func (cs *ConstraintSolver) postSolveVelocity(timeStep TimeStep) {}

func (cs *ConstraintSolver) preSolvePosition(timeStep TimeStep) {}

func (cs *ConstraintSolver) solvePositionSplitImpulse() {}

func (cs *ConstraintSolver) solvePositionNgs(timeStep TimeStep) {}

func (cs *ConstraintSolver) postSolve() {}

package demos

// ///////////////////////// Island
// (oimo/dynamics/Island.go)
// Simulation island.

type Island struct {
	gravity Vec3

	numRigidBodies int
	rigidBodies    []*RigidBody

	// all the constraint solvers
	numSolvers int
	solvers    []IConstraintSolver

	// the constraint solvers use split impulse for position part
	numSolversSi int
	solversSi    []IConstraintSolver

	// the constraint solvers use nonlinear Gauss-Seidel for position part
	numSolversNgs int
	solversNgs    []IConstraintSolver
}

func NewIsland() *Island {
	return &Island{
		rigidBodies: make([]*RigidBody, Settings.IslandInitialRigidBodyArraySize),
		solvers:     make([]IConstraintSolver, Settings.IslandInitialConstraintArraySize),
		solversSi:   make([]IConstraintSolver, Settings.IslandInitialConstraintArraySize),
		solversNgs:  make([]IConstraintSolver, Settings.IslandInitialConstraintArraySize),
	}
}

func (island *Island) clear() {
	Array_free(island.rigidBodies, island.numRigidBodies)
	Array_free(island.solvers, island.numSolvers)
	Array_free(island.solversSi, island.numSolversSi)
	Array_free(island.solversNgs, island.numSolversNgs)
}

func fastInvExp(x float64) float64 {
	x2 := x * x
	return 1.0 / (1.0 + x + x2*(1.0/2.0+x*(1.0/6.0)+x2*(1.0/24.0)))
}

func (island *Island) SetGravity(gravity *Vec3) {
	island.gravity = *gravity
}

// steps the single rigid body
func (island *Island) StepSingleRigidBody(timeStep TimeStep, rb *RigidBody) {
	dt := timeStep.Dt

	// store previous transform
	rb.pTransform = rb.transform

	// clear linear/angular contact impulse
	rb.linearContactImpulse.Zero()
	rb.angularContactImpulse.Zero()

	// update sleep time
	if rb.IsSleepy() {
		rb.sleepTime += dt
		if rb.sleepTime >= rb.sleepingTimeThreshold {
			rb.Sleep()
		}
	} else {
		rb.sleepTime = 0
	}

	if !rb.sleeping {
		// the rigid body is awake
		if rb._type == _DYNAMIC {
			// damping
			linScale := fastInvExp(dt * rb.linearDamping)
			angScale := fastInvExp(dt * rb.angularDamping)

			// compute accelerations
			var linAcc Vec3
			var angAcc Vec3
			MathUtil.Vec3_scale(&linAcc, &island.gravity, rb.gravityScale)
			MathUtil.Vec3_addRhsScaled(&linAcc, &linAcc, &rb.force, rb.invMass)
			MathUtil.Vec3_mulMat3(&angAcc, &rb.torque, &rb.invInertia)

			// update velocity
			MathUtil.Vec3_addRhsScaled(&rb.vel, &rb.vel, &linAcc, dt)
			MathUtil.Vec3_scale(&rb.vel, &rb.vel, linScale)
			MathUtil.Vec3_addRhsScaled(&rb.angVel, &rb.angVel, &angAcc, dt)
			MathUtil.Vec3_scale(&rb.angVel, &rb.angVel, angScale)
		}
		rb.Integrate(dt)
		rb.SyncShapes()
	}
}

func (island *Island) AddRigidBody(rigidBody *RigidBody) {
	if island.numRigidBodies == len(island.rigidBodies) {
		island.rigidBodies = Array_expand(island.rigidBodies)
	}
	rigidBody.addedToIsland = true
	island.rigidBodies[island.numRigidBodies] = rigidBody
	island.numRigidBodies++
}

func (island *Island) _addConstraintSolverSI(solver IConstraintSolver) {
	if island.numSolversSi == len(island.solversSi) {
		island.solversSi = Array_expand(island.solversSi)
	}
	island.solversSi[island.numSolversSi] = solver
	island.numSolversSi++
}

func (island *Island) _addConstraintSolverNgs(solver IConstraintSolver) {
	if island.numSolversNgs == len(island.solversNgs) {
		island.solversNgs = Array_expand(island.solversNgs)
	}
	island.solversNgs[island.numSolversNgs] = solver
	island.numSolversNgs++
}

func (island *Island) AddConstraintSolver(solver IConstraintSolver, positionCorrection PositionCorrectionAlgorithm) {
	if island.numSolvers == len(island.solvers) {
		island.solvers = Array_expand(island.solvers)
	}
	solver.(*ConstraintSolver).addedToIsland = true
	island.solvers[island.numSolvers] = solver
	island.numSolvers++

	if positionCorrection == _SPLIT_IMPULSE {
		island._addConstraintSolverSI(solver)
	}
	if positionCorrection == _NGS {
		island._addConstraintSolverNgs(solver)
	}
}

// steps the island with multiple bodies and constraints
func (island *Island) step(timeStep TimeStep, numVelocityIterations int, numPositionIterations int) {
	dt := timeStep.Dt

	sleepIsland := true

	// sleep check and apply gravity
	for i := range island.numRigidBodies {
		rb := island.rigidBodies[i]

		// store previous transform
		rb.pTransform = rb.transform

		// clear linear/angular contact impulse
		rb.linearContactImpulse.Zero()
		rb.angularContactImpulse.Zero()

		// don't let the rigid body sleep
		rb.sleeping = false

		// update sleep time
		if rb.IsSleepy() {
			rb.sleepTime += dt
		} else {
			rb.sleepTime = 0
		}

		// check if the rigid body is awaken
		if rb.sleepTime < rb.sleepingTimeThreshold {
			// awaken the whole island
			sleepIsland = false
		}

		// apply forces
		if rb._type == _DYNAMIC {
			// damping
			linScale := fastInvExp(dt * rb.linearDamping)
			angScale := fastInvExp(dt * rb.angularDamping)

			// compute accelerations
			linAcc := island.gravity.Scale(rb.gravityScale)
			MathUtil.Vec3_addRhsScaled(&linAcc, &linAcc, &rb.force, rb.invMass)
			var angAcc Vec3
			MathUtil.Vec3_mulMat3(&angAcc, &rb.torque, &rb.invInertia)

			// update velocity
			MathUtil.Vec3_addRhsScaled(&rb.vel, &rb.vel, &linAcc, dt)
			MathUtil.Vec3_scale(&rb.vel, &rb.vel, linScale)
			MathUtil.Vec3_addRhsScaled(&rb.angVel, &rb.angVel, &angAcc, dt)
			MathUtil.Vec3_scale(&rb.angVel, &rb.angVel, angScale)
		}
	}

	if sleepIsland {
		// sleep whoe island
		for i := range island.numRigidBodies {
			island.rigidBodies[i].Sleep()
		}
		return
	}

	// ----------------- test (Oimo) --------------------
	// omitted as its commented out

	// solve velocity
	for i := range island.numSolvers {
		island.solvers[i].preSolveVelocity(timeStep)
	}
	for i := range island.numSolvers {
		island.solvers[i].warmStart(timeStep)
	}
	for range numVelocityIterations {
		for i := range island.numSolvers {
			island.solvers[i].solveVelocity()
		}
	}

	// post-solve (velocity)
	for i := range island.numSolvers {
		island.solvers[i].postSolveVelocity(timeStep)
	}

	// integrate
	for i := range island.numRigidBodies {
		island.rigidBodies[i].Integrate(dt)
	}

	// solve split impulse
	for i := range island.numSolversSi {
		island.solversSi[i].preSolvePosition(timeStep)
	}
	for range numPositionIterations {
		for i := range island.numSolversSi {
			island.solversSi[i].solvePositionSplitImpulse()
		}
	}

	// solve integrate pseudo velocity
	for i := range island.numRigidBodies {
		island.rigidBodies[i].integratePseudoVelocity()
	}

	// solve nonlinear Gauss-Seidel
	for i := range island.numSolversNgs {
		island.solversNgs[i].preSolvePosition(timeStep)
	}
	for range numPositionIterations {
		for i := range island.numSolversNgs {
			island.solversNgs[i].solvePositionNgs(timeStep)
		}
	}

	// post-solve (some constraints may be removed)
	for i := range island.numSolvers {
		island.solvers[i].postSolve()
	}

	// synchronize shapes
	for i := range island.numRigidBodies {
		island.rigidBodies[i].SyncShapes()
	}
}

package demos

////////////////////// World (oimo/dynamics/World.go)
// The physics simulation world. This manages entire the dynamic simulation. You can add rigid bodies and joints to the world to simulate them.

type World struct {
	rigidBodyList     *RigidBody
	rigidBodyListLast *RigidBody

	jointList     *Joint
	jointListLast *Joint

	broadPhase     IBroadPhase
	contactManager *ContactManager

	numRigidBodies int
	numJoints      int
	numShapes      int
	numIslands     int

	numVelocityIterations int
	numPositionIterations int

	gravity Vec3

	timeStep            *TimeStep
	island              *Island
	rigidBodyStack      []*RigidBody
	solversInIslands    []*ConstraintSolver
	numSolversInIslands int
	rayCastWrapper      *RayCastWrapper
	convexCastWrapper   *ConvexCastWrapper
	aabbTestWrapper     *AabbTestWrapper

	pool         *Pool
	shapeIdCount int
}

// Creates a new physics world, with broad-phase collision detection algorithm `broadPhaseType` and gravitational acceleration `gravity`.
// Defaults: _BVH, nil
func NewWorld(broadPhaseType BroadPhaseType, gravity *Vec3) *World {
	w := World{}
	switch broadPhaseType {
	case _BRUTE_FORCE:
		w.broadPhase = NewBruteForceBroadPhase()
	case _BVH:
		w.broadPhase = NewBvhBroadPhase()
	}

	w.contactManager = NewContactManager(w.broadPhase)

	if gravity == nil {
		gravity = &Vec3{x: 0, y: -9.80665, z: 0}
	}
	w.gravity = *gravity

	w.numVelocityIterations = 10
	w.numPositionIterations = 5

	w.rayCastWrapper = NewRayCastWrapper()
	w.convexCastWrapper = NewConvexCastWrapper()
	w.aabbTestWrapper = NewAabbTestWrapper()

	w.island = NewIsland()
	w.solversInIslands = make([]*ConstraintSolver, Settings.IslandInitialConstraintArraySize)
	w.rigidBodyStack = make([]*RigidBody, Settings.IslandInitialRigidBodyArraySize)

	w.timeStep = NewTimeStep()

	w.pool = NewPool()

	return &w
}

func (w *World) step(timeStep float64) {
	if w.timeStep.Dt > 0 {
		w.timeStep.DtRatio = timeStep / w.timeStep.Dt
	}
	w.timeStep.Dt = timeStep
	w.timeStep.InvDt = 1.0 / timeStep

	// Profile hook: totalTime
	w.updateContacts()
	w.solveIslands()
}

func (w *World) updateContacts() {
	// Profile hook: broadPhaseCollisionTime
	w.contactManager.updateContacts()
	// Profile hook: narrowPhaseCollisionTime
	w.contactManager.updateManifolds()
}

func (w *World) solveIslands() {
	// Profile hook: dynamicsTime
	// wake up all rigid bodies if sleeping is disabled
	if Settings.DisableSleeping {
		//b := w.rigidBodyList
		// iterate rigidBodyList til last and set
		// b.sleeping = false
		// b.sleepTime = 0
	}

	// TODO
}

func (w *World) AddRigidBody(rigidBody *RigidBody) {}

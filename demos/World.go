package demos

import "github.com/g3n/engine/math32"

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

	gravity math32.Vector3

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
func NewWorld(broadPhaseType BroadPhaseType, gravity *math32.Vector3) *World {
	w := World{}
	switch broadPhaseType {
	case _BRUTE_FORCE:
		w.broadPhase = NewBruteForceBroadPhase()
	case _BVH:
		w.broadPhase = NewBvhBroadPhase()
	}

	w.contactManager = NewContactManager(w.broadPhase)

	if gravity == nil {
		gravity = &math32.Vector3{X: 0, Y: -9.80665, Z: 0}
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

func (w *World) AddRigidBody(rigidBody *RigidBody) {}

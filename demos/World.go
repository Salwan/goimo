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
	solversInIslands    []IConstraintSolver
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
	w.solversInIslands = make([]IConstraintSolver, Settings.IslandInitialConstraintArraySize)
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
		for b := w.rigidBodyList; b != nil; b = b.next {
			b.sleeping = false
			b.sleepTime = 0
		}
	}

	// expand array size if needed
	if len(w.rigidBodyStack) < w.numRigidBodies {
		newStackSize := len(w.rigidBodyStack) << 1
		for newStackSize < w.numRigidBodies {
			newStackSize <<= 1
		}
		w.rigidBodyStack = make([]*RigidBody, newStackSize)
	}

	// build and solve islands
	w.numIslands = 0
	w.island.SetGravity(&w.gravity)
	w.numSolversInIslands = 0
	for b := w.rigidBodyList; b != nil; b = b.next {
		if b.addedToIsland || b.sleeping || b._type == _STATIC {
			// never be the base of an island
			continue
		}
		if b.IsAlone() {
			w.island.StepSingleRigidBody(*w.timeStep, b)
			w.numIslands++
			continue
		}

		w.buildIsland(b)

		w.island.step(*w.timeStep, w.numVelocityIterations, w.numPositionIterations)
		w.island.clear()
		w.numIslands++
	}

	w.contactManager.postSolve()

	// clear island flags
	// clear forces and torques
	// (OimoPhysics code loops two times for this)
	for b := w.rigidBodyList; b != nil; b = b.next {
		b.addedToIsland = false
		b.force.Zero()
		b.torque.Zero()
	}

	for w.numSolversInIslands > 0 {
		w.numSolversInIslands--
		w.solversInIslands[w.numSolversInIslands].(*ConstraintSolver).addedToIsland = false
		w.solversInIslands[w.numSolversInIslands] = nil
	}
}

func (w *World) buildIsland(base *RigidBody) {
	// begin DFS
	stackCount := 1
	w.island.AddRigidBody(base)
	w.rigidBodyStack[0] = base

	for stackCount > 0 {
		// pop a rigid body
		stackCount--
		rb := w.rigidBodyStack[stackCount]
		w.rigidBodyStack[stackCount] = nil

		// stop searching deeper
		if rb._type == _STATIC {
			continue
		}

		// searching contacts
		for cl := rb.contactLinkList; cl != nil; cl = cl.next {
			// ignore if not touching
			cc := cl.contact.contactConstraint
			ccs := cl.contact.contactConstraint.solver
			if cc.isTouching() && !ccs.(*ConstraintSolver).addedToIsland {

				// add to constraint array (to clear island flag later)
				if len(w.solversInIslands) == w.numSolversInIslands {
					w.solversInIslands = Array_expand(w.solversInIslands)
				}
				w.solversInIslands[w.numSolversInIslands] = ccs
				w.numSolversInIslands++

				// add to island
				w.island.AddConstraintSolver(ccs, cc.positionCorrectionAlgorithm)

				// push the other rigid body if not added
				other := cl.other
				if !other.addedToIsland {
					w.island.AddRigidBody(other)
					w.rigidBodyStack[stackCount] = other
					stackCount++
				}
			}
		}

		// searching joints
		for jl := rb.jointLinkList; jl != nil; jl = jl.next {
			j := jl.joint
			js := j.solver
			if !js.(*ConstraintSolver).addedToIsland {

				// add to constraint array (to clear island flag later)
				if len(w.solversInIslands) == w.numSolversInIslands {
					w.solversInIslands = Array_expand(w.solversInIslands)
				}
				w.solversInIslands[w.numSolversInIslands] = js
				w.numSolversInIslands++

				// add to island
				w.island.AddConstraintSolver(js, j.positionCorrectionAlgorithm)

				// push the other rigid body if not added
				other := jl.other
				if !other.addedToIsland {
					w.island.AddRigidBody(other)
					w.rigidBodyStack[stackCount] = other
					stackCount++
				}
			}
		}
	}
}

func (w *World) AddRigidBody(rigidBody *RigidBody) {}

// TODO

// convex cast wrapper (broadphase -> world)
type ConvexCastWrapper struct { // implements IBroadPhaseProxyCallback
	callback    IRayCastCallback
	begin       *Transform
	translation Vec3
	convex      IConvexGeometry

	rayCastHit *RayCastHit
	zero       Vec3
}

func NewConvexCastWrapper() *ConvexCastWrapper {
	return &ConvexCastWrapper{
		rayCastHit: NewRayCastHit(),
		begin:      NewTransform(),
	}
}

func (self *ConvexCastWrapper) Process(proxy IProxy) { // override
	shape := proxy.(*Proxy).userData.(*Shape)
	t := shape.geom.GetType()

	if t < _CONVEX_MIN || t > _CONVEX_MAX {
		return
	}

	geom := shape.geom
	if GjkEpaInstance.ConvexCast(self.convex, geom.(IConvexGeometry), self.begin, &shape.transform, self.translation, self.zero, self.rayCastHit) {
		self.callback.Process(shape, self.rayCastHit)
	}
}

// aabb test wrapper (broadphase -> world)
type AabbTestWrapper struct { // implements IBroadPhaseProxyCallback
	callback IAabbTestCallback
	aabb     *Aabb
}

func NewAabbTestWrapper() *AabbTestWrapper {
	return &AabbTestWrapper{
		aabb: NewAabb(),
	}
}

func (self *AabbTestWrapper) Process(proxy IProxy) { // override
	shape := proxy.(*Proxy).userData.(*Shape)
	shapeAabb := shape.aabb

	// check if aabbs overlap again as proxies can be fattened by broadphase
	if MathUtil.Aabb_overlap(&shapeAabb.Min, &shapeAabb.Max, &self.aabb.Min, &self.aabb.Max) {
		self.callback.Process(shape)
	}
}

// TODO

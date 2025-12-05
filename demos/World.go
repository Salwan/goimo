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

	debugDraw *DebugDraw

	rayCastWrapper    *RayCastWrapper
	convexCastWrapper *ConvexCastWrapper
	aabbTestWrapper   *AabbTestWrapper

	pool         *Pool
	shapeIdCount int
}

// Creates a new physics world, with broad-phase collision detection algorithm `broadPhaseType` and gravitational acceleration `gravity`.
// Defaults: BroadPhaseType_BVH, nil
func NewWorld(broadPhaseType BroadPhaseType, gravity *Vec3) *World {
	w := World{}
	switch broadPhaseType {
	case BroadPhaseType_BRUTE_FORCE:
		w.broadPhase = NewBruteForceBroadPhase()
	case BroadPhaseType_BVH:
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
	for b := w.rigidBodyList; b != nil; {
		next := b.next

		if b.addedToIsland || b.sleeping || b._type == RigidBodyType_STATIC {
			// never be the base of an island
			b = next
			continue
		}
		if b.isAlone() {
			w.island.StepSingleRigidBody(*w.timeStep, b)
			w.numIslands++
			b = next
			continue
		}

		w.buildIsland(b)

		w.island.Step(*w.timeStep, w.numVelocityIterations, w.numPositionIterations)
		w.island.clear()
		w.numIslands++

		b = next
	}

	w.contactManager.postSolve()

	// clear island flags
	// clear forces and torques
	// (OimoPhysics code loops two times for this)
	for b := w.rigidBodyList; b != nil; {
		next := b.next
		b.addedToIsland = false
		b.force.Zero()
		b.torque.Zero()
		b = next
	}

	for w.numSolversInIslands > 0 {
		// Use Go clearing, but this is kept for reference
		// w.numSolversInIslands--
		// w.solversInIslands[w.numSolversInIslands].(*ConstraintSolver).addedToIsland = false
		// w.solversInIslands[w.numSolversInIslands] = nil

		w.solversInIslands = w.solversInIslands[:0]
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
		if rb._type == RigidBodyType_STATIC {
			continue
		}

		// searching contacts
		for cl := rb.contactLinkList; cl != nil; {
			next := cl.next

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

			cl = next
		}

		// searching joints
		for jl := rb.jointLinkList; jl != nil; {
			next := jl.next

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

			jl = next
		}
	}
}

func (self *World) addShape(shape *Shape) {
	shape.proxy = self.broadPhase.CreateProxy(shape, &shape.aabb)
	shape.id = self.shapeIdCount
	self.shapeIdCount++

	self.numShapes++
}

func (self *World) removeShape(shape *Shape) {
	self.broadPhase.DestroyProxy(shape.proxy)
	shape.proxy = nil
	shape.id = -1

	// destroy linked contacts
	for cl := shape.rigidBody.contactLinkList; cl != nil; {
		next := cl.next
		c := cl.contact
		if c.s1 == shape || c.s2 == shape {
			cl.other.WakeUp()
			self.contactManager.destroyContact(c)
		}
		cl = next
	}

	self.numShapes--
}

// Debug Drawing

func (self *World) drawBvh(d *DebugDraw, tree *BvhTree) {
	// TODO
	panic("not impl")
}

func (self *World) drawBvhNode(d *DebugDraw, node *BvhNode, level int, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawRigidBodies(d *DebugDraw) {
	// TODO
	panic("not impl")
}

func (self *World) drawBasis(d *DebugDraw, tf *Transform) {
	// TODO
	panic("not impl")
}

func (self *World) drawShape(d *DebugDraw, geom *Geometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawSphere(d *DebugDraw, g *SphereGeometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawBox(d *DebugDraw, g *BoxGeometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawCylinder(d *DebugDraw, g *CylinderGeometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawCone(d *DebugDraw, g *ConeGeometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawCapsule(d *DebugDraw, g *CapsuleGeometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawConvexHull(d *DebugDraw, g *ConvexHullGeometry, tf *Transform, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawAabb(d *DebugDraw, aabb Aabb, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawConstraints(d *DebugDraw) {
	// TODO
	panic("not impl")
}

func (self *World) drawContactPoint(d *DebugDraw, c *ContactConstraint, p *ManifoldPoint) {
	// TODO
	panic("not impl")
}

func (self *World) drawPair(d *DebugDraw, c *Contact, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawJoint(d *DebugDraw, j *Joint) {
	// TODO
	panic("not impl")
}

func (self *World) drawRevolute(d *DebugDraw, j *RevoluteJoint, anchor1 Vec3, anchor2 Vec3, basisX1 Vec3, basisY1 Vec3, basisZ1 Vec3, basisX2 Vec3, basisY2 Vec3, basisZ2 Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawCylindrical(d *DebugDraw, j *CylindricalJoint, anchor1 Vec3, anchor2 Vec3, basisX1 Vec3, basisY1 Vec3, basisZ1 Vec3, basisX2 Vec3, basisY2 Vec3, basisZ2 Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawPrismatic(d *DebugDraw, j *PrismaticJoint, anchor1 Vec3, anchor2 Vec3, basisX1 Vec3, basisY1 Vec3, basisZ1 Vec3, basisX2 Vec3, basisY2 Vec3, basisZ2 Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawUniversal(d *DebugDraw, j *UniversalJoint, anchor1 Vec3, anchor2 Vec3, basisX1 Vec3, basisY1 Vec3, basisZ1 Vec3, basisX2 Vec3, basisY2 Vec3, basisZ2 Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawRagdoll(d *DebugDraw, j *RagdollJoint, anchor1 Vec3, anchor2 Vec3, basisX1 Vec3, basisY1 Vec3, basisZ1 Vec3, basisX2 Vec3, basisY2 Vec3, basisZ2 Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawGeneric(d *DebugDraw, j *GenericJoint, anchor1 Vec3, anchor2 Vec3, basisX1 Vec3, basisY1 Vec3, basisZ1 Vec3, basisX2 Vec3, basisY2 Vec3, basisZ2 Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawRotationalLimit(d *DebugDraw, center Vec3, ex Vec3, ey Vec3, needle Vec3, radius float64, min float64, max float64, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawTranslationalLimit(d *DebugDraw, center Vec3, ex Vec3, min float64, max float64, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawTranslationalLimit3D(d *DebugDraw, center Vec3, ex Vec3, ey Vec3, ez Vec3, xlm *TranslationalLimitMotor, ylm *TranslationalLimitMotor, zlm *TranslationalLimitMotor, color Vec3) {
	// TODO
	panic("not impl")
}

func (self *World) drawEllipseOnSphere(d *DebugDraw, center Vec3, normal Vec3, x Vec3, y Vec3, radiansX float64, radiansY float64, radius float64, color Vec3) {
	// TODO
	panic("not impl")
}

// --- public ---

func (w *World) Step(timeStep float64) {
	if w.timeStep.Dt > 0 {
		w.timeStep.DtRatio = timeStep / w.timeStep.Dt
	}
	w.timeStep.Dt = timeStep
	w.timeStep.InvDt = 1.0 / timeStep

	// Profile hook: totalTime
	w.updateContacts()
	w.solveIslands()
}

func (self *World) AddRigidBody(rigidBody *RigidBody) {
	if rigidBody.world != nil {
		panic("A rigid body cannot belong to multiple worlds.")
	}

	// first, add the rigid body to the world
	self.rigidBodyList, self.rigidBodyListLast = DoubleList_push(self.rigidBodyList, self.rigidBodyListLast, rigidBody)
	rigidBody.world = self

	// then add the shapes to the world
	for s := rigidBody.shapeList; s != nil; {
		next := s.next
		self.addShape(s)
		s = next
	}

	self.numRigidBodies++
}

// Removes the rigid body `rigidBody` from the simulation world.
func (self *World) RemoveRigidBody(rigidBody *RigidBody) {
	if rigidBody.world != self {
		panic("The rigid body doesn't belong to the world.")
	}
	// first, remove the rigid body from the world
	self.rigidBodyList, self.rigidBodyListLast = DoubleList_remove(self.rigidBodyList, self.rigidBodyListLast, rigidBody)
	rigidBody.world = nil

	// then remove the shapes from the world
	for s := rigidBody.shapeList; s != nil; {
		next := s.next
		self.removeShape(s)
		s = next
	}

	self.numRigidBodies--
}

// Adds the joint `joint` to the simulation world.
func (self *World) AddJoint(joint *Joint) {
	if joint.world != nil {
		panic("A joint cannot belong to multiple worlds.")
	}

	self.jointList, self.jointListLast = DoubleList_push(self.jointList, self.jointListLast, joint)
	joint.world = self
	joint.attachLinks()
	joint.syncAnchors()

	self.numJoints++
}

// Removes the joint `joint` from the simulation world.
func (self *World) RemoveJoint(joint *Joint) {
	if joint.world != self {
		panic("The joint doesn't belong to the world.")
	}
	self.jointList, self.jointListLast = DoubleList_remove(self.jointList, self.jointListLast, joint)
	joint.world = nil
	joint.detachLinks()

	self.numJoints--
}

// Sets the debug draw interface to `debugDraw`. Call `World.debugDraw` to draw the simulation world.
func (self *World) SetDebugDraw(debugDraw *DebugDraw) {
	self.debugDraw = debugDraw
}

// Returns the debug draw interface.
func (self *World) GetDebugDraw() *DebugDraw {
	return self.debugDraw
}

// Draws the simulation world for debugging. Call `World.setDebugDraw` to set the debug draw interface.
func (self *World) DrawDebug() {
	// TODO
	panic("not impl")
}

// Performs a ray casting. `callback.process` is called for all shapes the ray from `begin` to `end` hits.
func (self *World) RayCast(begin Vec3, end Vec3, callback IRayCastCallback) {
	self.rayCastWrapper.begin = begin
	self.rayCastWrapper.end = end
	self.rayCastWrapper.callback = callback

	self.broadPhase.RayCast(begin, end, self.rayCastWrapper)
}

// Performs a convex casting. `callback.process` is called for all shapes the convex geometry `convex` hits. The convex geometry translates by `translation` starting from the beginning transform `begin`.
func (self *World) ConvexCast(convex IConvexGeometry, begin *Transform, translation Vec3, callback IRayCastCallback) {
	self.convexCastWrapper.convex = convex
	self.convexCastWrapper.begin = *begin
	self.convexCastWrapper.translation = translation
	self.convexCastWrapper.callback = callback

	self.broadPhase.ConvexCast(convex, begin, translation, self.convexCastWrapper)
}

// Performs an AABB query. `callback.process` is called for all shapes that their AABB and `aabb` intersect.
func (self *World) AabbTest(aabb *Aabb, callback IAabbTestCallback) {
	*self.aabbTestWrapper.aabb = *aabb
	self.aabbTestWrapper.callback = callback

	self.broadPhase.AabbTest(aabb, self.aabbTestWrapper)
}

// Returns the list of the rigid bodies added to the world.
func (self *World) GetRigidBodyList() *RigidBody {
	return self.rigidBodyList
}

// Returns the list of the joints added to the world.
func (self *World) GetJointList() *Joint {
	return self.jointList
}

// Returns the broad-phase collision detection algorithm.
func (self *World) GetBroadPhase() IBroadPhase {
	return self.broadPhase
}

// Returns the contact manager.
func (self *World) GetContactManager() *ContactManager {
	return self.contactManager
}

// Returns the number of the rigid bodies added to the world.
func (self *World) GetNumRigidBodies() int {
	return self.numRigidBodies
}

// Returns the number of the joints added to the world.
func (self *World) GetNumJoints() int {
	return self.numJoints
}

// Returns the number of the shapes added to the world.
func (self *World) GetNumShapes() int {
	return self.numShapes
}

// Returns the number of simulation islands.
func (self *World) GetNumIslands() int {
	return self.numIslands
}

// Returns the number of velocity iterations of constraint solvers.
func (self *World) GetNumVelocityIterations() int {
	return self.numVelocityIterations
}

// Sets the number of velocity iterations of constraint solvers to `numVelocityIterations`.
func (self *World) SetNumVelocityIterations(numVelocityIterations int) {
	self.numVelocityIterations = numVelocityIterations
}

// Returns the number of position iterations of constraint solvers.
func (self *World) GetNumPositionIterations() int {
	return self.numPositionIterations
}

// Sets the number of position iterations of constraint solvers to `numPositionIterations`.
func (self *World) SetNumPositionIterations(numPositionIterations int) {
	self.numPositionIterations = numPositionIterations
}

// Returns the gravitational acceleration of the simulation world.
func (self *World) GetGravity() Vec3 {
	return self.gravity
}

// Sets the gravitational acceleration of the simulation world to `gravity`.
func (self *World) SetGravity(gravity Vec3) {
	self.gravity = gravity
}

// ray cast wrapper (broadphase -> world)
type RayCastWrapper struct { // implements IBroadPhaseProxyCallback
	callback IRayCastCallback
	begin    Vec3
	end      Vec3

	rayCastHit *RayCastHit
}

func NewRayCastWrapper() *RayCastWrapper {
	return &RayCastWrapper{
		rayCastHit: NewRayCastHit(),
	}
}

func (self *RayCastWrapper) Process(proxy IProxy) { // override
	shape := proxy.(*Proxy).userData.(*Shape)

	if shape.geom.RayCast(self.begin, self.end, &shape.transform, self.rayCastHit) {
		self.callback.Process(shape, self.rayCastHit)
	}
}

// convex cast wrapper (broadphase -> world)
type ConvexCastWrapper struct { // implements IBroadPhaseProxyCallback
	callback    IRayCastCallback
	begin       Transform
	translation Vec3
	convex      IConvexGeometry

	rayCastHit *RayCastHit
	zero       Vec3
}

func NewConvexCastWrapper() *ConvexCastWrapper {
	c := &ConvexCastWrapper{
		rayCastHit: NewRayCastHit(),
	}
	c.begin.Identity()
	return c
}

func (self *ConvexCastWrapper) Process(proxy IProxy) { // override
	shape := proxy.(*Proxy).userData.(*Shape)
	t := shape.geom.GetType()

	if t < GeometryType_CONVEX_MIN || t > GeometryType_CONVEX_MAX {
		return
	}

	geom := shape.geom
	if GjkEpaInstance.ConvexCast(self.convex, geom.(IConvexGeometry), &self.begin, &shape.transform, self.translation, self.zero, self.rayCastHit) {
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

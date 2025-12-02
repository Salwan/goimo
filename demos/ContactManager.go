package demos

import "github.com/Salwan/goimo/debug"

// ///////////////////////// ContactManager
// (oimo/dynamics/ContactManager.go)
// The manager of the contacts in the physics world. A contact of two shapes is created when the AABBs of them begin overlapping, and is destroyed when they end overlapping.

type ContactManager struct {
	numContacts     int
	contactList     *Contact
	contactListLast *Contact
	contactPool     *Contact

	broadPhase      IBroadPhase
	collisionMatrix *CollisionMatrix
}

func NewContactManager(broadPhase IBroadPhase) *ContactManager {
	cm := &ContactManager{}
	cm.broadPhase = broadPhase
	cm.collisionMatrix = NewCollisionMatrix()
	return cm
}

// --- private ---

func (cm *ContactManager) _createContacts() {
	for pp := cm.broadPhase.GetProxyPairList(); pp != nil; pp = pp.next {
		var s1 *Shape
		var s2 *Shape
		if debug.Debug && pp.p1.GetID() == pp.p2.GetID() {
			panic("OimoPhysics asserts here")
		}

		if pp.p1.GetID() < pp.p2.GetID() {
			s1 = pp.p1.GetUserData().(*Shape)
			s2 = pp.p2.GetUserData().(*Shape)
		} else {
			s1 = pp.p2.GetUserData().(*Shape)
			s2 = pp.p1.GetUserData().(*Shape)
		}

		// collision filtering
		if !cm._shouldCollide(s1, s2) {
			continue
		}

		// search for the same contact
		b1 := s1.rigidBody
		b2 := s2.rigidBody
		n1 := b1.numContactLinks
		n2 := b2.numContactLinks
		var l *ContactLink

		// select shorter linked list
		if n1 < n2 {
			l = b1.contactLinkList
		} else {
			l = b2.contactLinkList
		}

		id1, id2 := s1.id, s2.id
		found := false

		for ; l != nil; l = l.next {
			c := l.contact
			if c.s1.id == id1 && c.s2.id == id2 {
				// the same contact found
				c.latest = true
				found = true
				break
			}
		}

		// if not found, create a new contact
		if !found {
			// trying to pick an object up from the pool
			c := SingleList_pick(&cm.contactPool, NewContact)
			DoubleList_push(&cm.contactList, &cm.contactListLast, c)
			c.latest = true
			c.attach(s1, s2, cm.collisionMatrix.GetDetector(s1.geom.(*Geometry)._type, s2.geom.(*Geometry)._type))
			cm.numContacts++
		}
	}
}

func (self *ContactManager) _destroyOutdatedContacts() {
	// whether the broadphase returns only new overlapping pairs
	incremental := self.broadPhase.IsIncremental()

	for c := self.contactList; c != nil; c = c.next {
		if c.latest {
			// the contact is overlapping, make it old for the next step
			c.latest = false
			c.shouldBeSkipped = false
			continue
		}
		if !incremental {
			// the pair is separated, because the broad-phase algorithm collects
			// all the overlapping pairs and they are marked as latest
			self.destroyContact(c)
			continue
		}

		s1 := c.s1
		s2 := c.s2
		r1 := s1.rigidBody
		r2 := s2.rigidBody

		active1 := !r1.sleeping && r1._type != _STATIC
		active2 := !r2.sleeping && r2._type != _STATIC
		if !active1 && !active2 {
			// skip the pair if both rigid bodies are inactive
			c.shouldBeSkipped = true
			continue
		}

		aabb1 := s1.aabb
		aabb2 := s2.aabb
		if !self.broadPhase.IsOverlapping(s1.proxy, s2.proxy) || !self._shouldCollide(s1, s2) {
			// the proxy pair is separated or shouldn't collide
			self.destroyContact(c)
			continue
		}

		// the proxies are overlapping, but AABBs might be separated
		aabbOverlapping := MathUtil.Aabb_overlap(&aabb1.Min, &aabb1.Max, &aabb2.Min, &aabb2.Max)
		// needs narrow-phase collision detection if AABBs are overlapping
		c.shouldBeSkipped = !aabbOverlapping
	}
}

func (cm *ContactManager) _shouldCollide(s1, s2 *Shape) bool {
	r1 := s1.rigidBody
	r2 := s2.rigidBody

	if r1 == r2 {
		// they have the same parent
		return false
	}

	if r1._type != _DYNAMIC && r2._type != _DYNAMIC {
		// neither is dynamic
		return false
	}

	// collision filtering
	if s1.collisionGroup&s2.collisionMask == 0 || s2.collisionGroup&s1.collisionMask == 0 {
		return false
	}

	// search for joints the two bodies are connected to
	var jl *JointLink
	var other *RigidBody
	if r1.numJointLinks < r2.numJointLinks {
		jl = r1.jointLinkList
		other = r2
	} else {
		jl = r2.jointLinkList
		other = r1
	}

	for ; jl != nil; jl = jl.next {
		if jl.other == other && !jl.joint.allowCollision {
			// collisions between the two bodies are disabled
			return false
		}
	}

	return true
}

// --- internal ---

func (cm *ContactManager) updateContacts() {
	cm.broadPhase.CollectPairs()
	cm._createContacts()
	cm._destroyOutdatedContacts()
}

// send postSolve events
func (self *ContactManager) postSolve() {
	for c := self.contactList; c != nil; c = c.next {
		if c.touching {
			c.postSolve()
		}
	}
}

func (self *ContactManager) updateManifolds() {
	for c := self.contactList; c != nil; c = c.next {
		if !c.shouldBeSkipped {
			c.updateManifold()
		}
	}
}

func (self *ContactManager) destroyContact(contact *Contact) {
	DoubleList_remove(&self.contactList, &self.contactListLast, contact)
	contact.detach()

	// put it into the pool
	SingleList_pool(&self.contactPool, contact)

	self.numContacts--
}

// --- public ---

// Returns the number of the contacts in the world.
func (self *ContactManager) GetNumContacts() int {
	return self.numContacts
}

// Returns the linked list of the contacts in the world.
func (self *ContactManager) GetContactList() *Contact {
	return self.contactList
}

// TODO

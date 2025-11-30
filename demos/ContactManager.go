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

func (cm *ContactManager) createContacts() {
	for pp := cm.broadPhase.GetProxyPairList(); pp != nil; pp = pp.next {
		var s1 *Shape
		var s2 *Shape
		if debug.Debug && pp.p1.id == pp.p2.id {
			panic("OimoPhysics asserts here")
		}

		if pp.p1.id < pp.p2.id {
			s1 = pp.p1.userData.(*Shape)
			s2 = pp.p2.userData.(*Shape)
		} else {
			s1 = pp.p2.userData.(*Shape)
			s2 = pp.p1.userData.(*Shape)
		}

		// collision filtering
		if !cm.shouldCollide(s1, s2) {
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

func (cm *ContactManager) destroyOutdatedContacts() {
	// whether the broadphase returns only new overlapping pairs
	incremental := cm.broadPhase.(*BroadPhase).incremental

	for c := cm.contactList; c != nil; c = c.next {
		if c.latest {
			// the contact is overlapping, make it old for the next step
			c.latest = false
			c.shouldBeSkipped = false
			continue
		}
		if !incremental {
			// the pair is separated, because the broad-phase algorithm collects
			// all the overlapping pairs and they are marked as latest
			cm.destroyContact(c)
		}
	}

	// TODO
	panic("not impl")
}

func (cm *ContactManager) shouldCollide(s1, s2 *Shape) bool {
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

func (cm *ContactManager) updateContacts() {
	cm.broadPhase.collectPairs()
	cm.createContacts()
	cm.destroyOutdatedContacts()
}

func (cm *ContactManager) updateManifolds() {
	// TODO
	panic("not impl")
}

func (cm *ContactManager) postSolve() {
	for c := cm.contactList; c != nil; c = c.next {
		if c.touching {
			c.postSolve()
		}
	}
}

func (cm *ContactManager) destroyContact(contact *Contact) {
	DoubleList_remove(&cm.contactList, &cm.contactListLast, contact)
	contact.detach()
}

// TODO

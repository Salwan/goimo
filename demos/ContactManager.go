package demos

// ///////////////////////// ContactManager
// (oimo/dynamics/ContactManager.go)
// The manager of the contacts in the physics world. A contact of two shapes is created when the AABBs of them begin overlapping, and is destroyed when they end overlapping.

type ContactManager struct {
	numContacts     int
	contactList     *Contact
	contactListLast *Contact
	contactPool     *Contact

	broadPhase      IBroadPhase
	collisionMatrix CollisionMatrix
}

func NewContactManager(broadPhase IBroadPhase) *ContactManager {
	cm := &ContactManager{}
	cm.broadPhase = broadPhase
	cm.collisionMatrix = NewCollisionMatrix()
	cm.numContacts = 0
	return cm
}

func (cm *ContactManager) updateContacts()  {}
func (cm *ContactManager) updateManifolds() {}

func (cm *ContactManager) postSolve() {
	for c := cm.contactList; c != nil; c = c.next {
		if c.touching {
			c.postSolve()
		}
	}
}

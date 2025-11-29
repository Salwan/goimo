package demos

//////////////////////////////////////////// Contact
// (oimo/dynamics/Contact.go)
// A contact is a cached pair of overlapping shapes in the physics world. contacts are created by `ContactManager` when two AABBs of shapes begin overlapping.
// As AABBs are larger than its shapes, shapes of a contact don't always touching or colliding though their AABBs are overlapping.

type Contact struct {
	next *Contact
	prev *Contact

	link1 *ContactLink
	link2 *ContactLink

	s1 *Shape
	s2 *Shape
	b1 *RigidBody
	b2 *RigidBody

	// detector data
	detector           IDetector
	cachedDetectorData *CachedDetectorData
	detectorResult     *DetectorResult

	// tmp data
	latest          bool
	shouldBeSkipped bool

	// constraint/manifold data
	manifold          *Manifold
	updater           *ManifoldUpdater
	contactConstraint *ContactConstraint
	touching          bool
}

func NewContact() *Contact {
	m := NewManifold()
	c := &Contact{
		link1: NewContactLink(),
		link2: NewContactLink(),

		cachedDetectorData: NewCachedDetectorData(),
		detectorResult:     NewDetectorResult(),

		manifold:          m,
		updater:           NewManifoldUpdater(m),
		contactConstraint: NewContactConstraint(m),
	}
	return c
}

func (c *Contact) GetNext() *Contact {
	return c.next
}

func (c *Contact) SetNext(x *Contact) {
	c.next = x
}

func (c *Contact) GetPrev() *Contact {
	return c.prev
}

func (c *Contact) SetPrev(x *Contact) {
	c.prev = x
}

func (c *Contact) attachLinks() {
	DoubleList_push(&c.b1.contactLinkList, &c.b1.contactLinkListLast, c.link1)
	DoubleList_push(&c.b2.contactLinkList, &c.b2.contactLinkListLast, c.link2)
	c.b1.numContactLinks++
	c.b2.numContactLinks++
	c.link1.other = c.b2
	c.link2.other = c.b1
	c.link1.contact = c
	c.link2.contact = c
}

func (c *Contact) sendPostSolve() {
	cc1 := c.s1.contactCallback
	cc2 := c.s2.contactCallback
	if cc1 == cc2 {
		cc2 = nil // avoid calling twice
	}
	if cc1 != nil {
		cc1.postSolve(c)
	}
	if cc2 != nil {
		cc2.postSolve(c)
	}
}

func (c *Contact) postSolve() {
	c.sendPostSolve()
}

func (c *Contact) attach(s1, s2 *Shape, detector IDetector) {
	c.s1 = s1
	c.s2 = s2
	c.b1 = s1.rigidBody
	c.b2 = s2.rigidBody
	c.touching = false
	c.attachLinks()

	c.detector = detector

	c.contactConstraint.attach(s1, s2)
}

// TODO

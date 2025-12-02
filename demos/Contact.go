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

// --- double linked list interface ---

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

// --- private ---

func (c *Contact) _attachLinks() {
	c.b1.contactLinkList, c.b1.contactLinkListLast = DoubleList_push(c.b1.contactLinkList, c.b1.contactLinkListLast, c.link1)
	c.b2.contactLinkList, c.b2.contactLinkListLast = DoubleList_push(c.b2.contactLinkList, c.b2.contactLinkListLast, c.link2)

	c.b1.numContactLinks++
	c.b2.numContactLinks++
	c.link1.other = c.b2
	c.link2.other = c.b1
	c.link1.contact = c
	c.link2.contact = c
}

func (c *Contact) _detachLinks() {
	c.b1.contactLinkList, c.b1.contactLinkListLast = DoubleList_remove(c.b1.contactLinkList, c.b1.contactLinkListLast, c.link1)
	c.b2.contactLinkList, c.b2.contactLinkListLast = DoubleList_remove(c.b2.contactLinkList, c.b2.contactLinkListLast, c.link2)

	c.b1.numContactLinks--
	c.b2.numContactLinks--

	c.link1.other = nil
	c.link2.other = nil
	c.link1.contact = nil
	c.link2.contact = nil
}

func (c *Contact) _sendBeginContact() {
	cc1 := c.s1.contactCallback
	cc2 := c.s2.contactCallback
	if cc1 == cc2 {
		cc2 = nil // avoid calling twice
	}
	if cc1 != nil {
		cc1.beginContact(c)
	}
	if cc2 != nil {
		cc2.beginContact(c)
	}
}

func (c *Contact) _sendEndContact() {
	cc1 := c.s1.contactCallback
	cc2 := c.s2.contactCallback
	if cc1 == cc2 {
		cc2 = nil // avoid calling twice
	}
	if cc1 != nil {
		cc1.endContact(c)
	}
	if cc2 != nil {
		cc2.endContact(c)
	}
}

func (c *Contact) _sendPreSolve() {
	cc1 := c.s1.contactCallback
	cc2 := c.s2.contactCallback
	if cc1 == cc2 {
		cc2 = nil // avoid calling twice
	}
	if cc1 != nil {
		cc1.preSolve(c)
	}
	if cc2 != nil {
		cc2.preSolve(c)
	}
}

func (c *Contact) _sendPostSolve() {
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

// --- internal

func (c *Contact) attach(s1, s2 *Shape, detector IDetector) {
	c.s1 = s1
	c.s2 = s2
	c.b1 = s1.rigidBody
	c.b2 = s2.rigidBody
	c.touching = false
	c._attachLinks()

	c.detector = detector

	c.contactConstraint.attach(s1, s2)
}

func (c *Contact) detach() {
	if c.touching {
		// touching in the last frame
		c._sendEndContact()
	}

	c._detachLinks()
	c.s1, c.s2 = nil, nil
	c.b1, c.b2 = nil, nil
	c.touching = false

	c.cachedDetectorData.clear()
	c.manifold.clear()

	c.detector = nil

	c.contactConstraint.detach()
}

func (self *Contact) updateManifold() {
	if self.detector == nil {
		return
	}

	ptouching := self.touching

	result := self.detectorResult
	self.detector.Detect(result, self.s1.geom, self.s2.geom, &self.s1.transform, &self.s2.transform, self.cachedDetectorData)

	num := result.numPoints
	self.touching = num > 0

	if self.touching {
		// update manifold basis
		self.manifold.buildBasis(result.normal)

		// determine position correction algorithm
		if result.GetMaxDepth() > Settings.ContactUseAlternativePositionCorrectionAlgorithmDepthThreshold {
			// use alternative position correction method (split impulse by default) for deeply overlapped contacts
			self.contactConstraint.positionCorrectionAlgorithm = Settings.AlternativeContactPositionCorrectionAlgorithm
		} else {
			// use default position correction algorithm for slightly overlapped contacts
			self.contactConstraint.positionCorrectionAlgorithm = Settings.DefaultContactPositionCorrectionAlgorithm
		}

		// update contact manifold
		if result.incremental {
			// incremental manifold
			self.updater.incrementalUpdate(result, &self.b1.transform, &self.b2.transform)
		} else {
			// one-shot manifold
			self.updater.totalUpdate(result, &self.b1.transform, &self.b2.transform)
		}
	} else {
		self.manifold.clear()
	}

	if self.touching && !ptouching {
		self._sendBeginContact()
	}
	if !self.touching && ptouching {
		self._sendEndContact()
	}
	if self.touching {
		self._sendPreSolve()
	}
}

// called from the contact manager
func (c *Contact) postSolve() {
	c._sendPostSolve()
}

// --- public ---

// Returns the first shape of the contact.
func (self *Contact) GetShape1() *Shape {
	return self.s1
}

// Returns the second shape of the contact.
func (self *Contact) GetShape2() *Shape {
	return self.s2
}

// Returns whether the shapes are touching.
func (self *Contact) isTouching() bool {
	return self.touching
}

// Returns the contact manifold.
func (self *Contact) GetManifold() *Manifold {
	return self.manifold
}

// Returns the contact constraint.
func (self *Contact) GetContactConstraint() *ContactConstraint {
	return self.contactConstraint
}

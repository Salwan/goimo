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
	detector           *Detector
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

// TODO

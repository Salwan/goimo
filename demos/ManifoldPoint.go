package demos

// ////////////////////////////////////////// ManifoldPoint
// (oimo/dynamics/constraint/contact/ManifoldPoint.go)
// A manifold point is a contact point in a contact manifold. This holds detailed collision data (position, overlap depth, impulse, etc...) for collision response.
type ManifoldPoint struct {
	// manifold point relative to rigid bodies. NOT SHAPES.
	localPos1 Vec3
	localPos2 Vec3

	// local position with rotation
	relPos1 Vec3
	relPos2 Vec3

	// world position
	pos1  Vec3
	pos2  Vec3
	depth float64

	impulse ContactImpulse

	warmStarted bool

	// manifold points can be disabled for some reasons (separated, etc...)
	disabled bool

	id int
}

func NewManifoldPoint() *ManifoldPoint {
	return &ManifoldPoint{
		id: -1,
	}
}

// TODO

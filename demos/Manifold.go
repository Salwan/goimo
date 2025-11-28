package demos

// /////////////////////////////////////////// Manifold
// (oimo/dynamics/constraint/contact/Manifold.go)
// A contact manifold holds collision data of a pair of shapes. (size=80+b)
type Manifold struct {
	normal    Vec3
	tangent   Vec3
	binormal  Vec3
	numPoints int
	points    []*ManifoldPoint
}

func NewManifold() *Manifold {
	m := &Manifold{
		points: make([]*ManifoldPoint, Settings.MaxManifoldPoints),
	}
	for i := range len(m.points) {
		m.points[i] = NewManifoldPoint()
	}
	return m
}

// TODO

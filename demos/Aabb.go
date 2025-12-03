package demos

///////////////////////////////////// Aabb
// (oimo/collision/geometry/Aabb.go)
// The axis-aligned bounding box.

type Aabb struct {
	Min Vec3
	Max Vec3
}

// Creates an empty AABB. Minimum and maximum points are set to zero.
func NewAabb() *Aabb {
	return &Aabb{}
}

// Sets the minimum and maximum point and returns `this`.
// Equivallent to `setMin(min).setMax(max)`.
func (ab *Aabb) Set(min *Vec3, max *Vec3) *Aabb {
	ab.Min = *min
	ab.Max = *max
	return ab
}

// Returns the minimum point of the axis-aligned bounding box.
func (ab *Aabb) getMin() Vec3 {
	return ab.Min
}

// TODO

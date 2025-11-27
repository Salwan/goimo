package demos

///////////////////////////////// RayCastHit
// (oimo/collision/geometry/RayCastHit.go)
// A single ray cast hit data.

type RayCastHit struct {
	Position Vec3    // The position the ray hit at.
	Normal   Vec3    // The normal vector of the surface the ray hit.
	Fraction float64 // The ratio of the position the ray hit from the start point to the end point.
}

func NewRayCastHit() *RayCastHit {
	return &RayCastHit{}
}

package demos

// //////////////////////////////////////////// DetectorResultPoint
// (oimo/collision/narrowphase/DetectorResultPoint.go)
// The result point is a pair of the closest points of collision geometries detected by a collision detector. This holds relative closest points for each collision geometry and the amount of the overlap.

type DetectorResultPoint struct {
	// The first collision geometry's closest point.
	position1 Vec3

	// The second collision geometry's closest point.
	position2 Vec3

	// The amount of the overlap. This becomes negative if two geometries are separate.
	depth float64

	// The identification of the result point.
	id int
}

func NewDetectorResultPoint() *DetectorResultPoint {
	return &DetectorResultPoint{}
}

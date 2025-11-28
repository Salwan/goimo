package demos

////////////////////////////////////////////////// DetectorResult
// (oimo/collision/narrowphase/DetectorResult.go)
// The result of narrow-phase collision detection. This is used for generating contact points of a contact constraint at once or incrementally.

type DetectorResult struct {
	// number of result points
	numPoints int

	// The result points. Note that **only the first `DetectorResult.numPoints` points are computed by the collision detector**.
	points []*DetectorResultPoint

	// The normal vector of the contact plane.
	normal Vec3

	// Whether the result points are to be used for incremental menifold update.
	incremental bool // for GJK/EPA detector
}

func NewDetectorResult() *DetectorResult {
	dr := &DetectorResult{
		points: make([]*DetectorResultPoint, Settings.MaxManifoldPoints),
	}

	for i := range len(dr.points) {
		dr.points[i] = NewDetectorResultPoint()
	}

	return dr
}

// TODO

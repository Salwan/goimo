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

// --- public ---

// Returns the maximum depth of the result points. Returns `0.0` if no result points are available.
func (dr *DetectorResult) GetMaxDepth() float64 {
	max := 0.0
	for i := range dr.numPoints {
		if dr.points[i].depth > max {
			max = dr.points[i].depth
		}
	}
	return max
}

// Cleans up the result data.
func (dr *DetectorResult) Clear() {
	dr.numPoints = 0
	for _, p := range dr.points {
		p.position1.Zero()
		p.position2.Zero()
		p.depth = 0
		p.id = 0
	}
	dr.normal.Zero()
}

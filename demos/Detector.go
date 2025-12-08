package demos

// //////////////////////////////////////////// Detector
// (oimo/collision/narrowphase/detector/Detector.go)
// Interface of a collision detector for narrow-phase collision detection. (size=1)

type IDetector interface {
	detectImpl(result *DetectorResult, geom1, geom2 IGeometry, tf1, tf2 *Transform, cachedData *CachedDetectorData)

	Detect(result *DetectorResult, geom1, geom2 IGeometry, transform1, transform2 *Transform, cachedData *CachedDetectorData)
}

type Detector struct {
	swapped bool
}

func NewDetector(swapped bool) *Detector {
	return &Detector{
		swapped: swapped,
	}
}

func (d *Detector) setNormal(result *DetectorResult, n Vec3) {
	result.normal = n
	if d.swapped {
		result.normal.NegateEq()
	}
}

func (d *Detector) addPoint(result *DetectorResult, pos1, pos2 Vec3, depth float64, id int) {
	p := result.points[result.numPoints]
	result.numPoints++

	p.depth = depth
	p.id = id
	if d.swapped {
		p.position1 = pos2
		p.position2 = pos1
	} else {
		p.position1 = pos1
		p.position2 = pos2
	}
}

func (d *Detector) detectImpl(result *DetectorResult, geom1, geom2 IGeometry, tf1, tf2 *Transform, cachedData *CachedDetectorData) {
	panic("abstract call")
}

// --- public ---

func (d *Detector) Detect(result *DetectorResult, geom1, geom2 IGeometry, transform1, transform2 *Transform, cachedData *CachedDetectorData) { // override
	result.Clear()
	if d.swapped {
		d.detectImpl(result, geom2, geom1, transform2, transform1, cachedData)
	} else {
		d.detectImpl(result, geom1, geom2, transform1, transform2, cachedData)
	}
}

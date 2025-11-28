package demos

// //////////////////////////////////////////// Detector
// (oimo/collision/narrowphase/detector/Detector.go)
// Interface of a collision detector for narrow-phase collision detection. (size=1)
type Detector struct {
	swapped bool
}

func NewDetector(swapped bool) *Detector {
	return &Detector{
		swapped: swapped,
	}
}

// TODO

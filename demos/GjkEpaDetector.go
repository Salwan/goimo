package demos

//////////////////////////////////////////////// GjkEpaDetector
// (oimo/collision/narrowphase/detector/GjkEpaDetector.go)
// General convex collision detector using GJK/EPA

type GjkEpaDetector struct {
	*Detector
}

func NewGjkEpaDetector() *GjkEpaDetector {
	return &GjkEpaDetector{
		Detector: NewDetector(false),
	}
}

// TODO

package demos

//////////////////////////////////////////////// SphereBoxDetector
// (oimo/path)
// Sphere vs Box collision detector.

type SphereBoxDetector struct {
	*Detector
	//TODO
}

func NewSphereBoxDetector(swapped bool) *SphereBoxDetector {
	return &SphereBoxDetector{
		Detector: NewDetector(swapped),
	}
}

// TODO

package demos

//////////////////////////////////////////////// SphereCapsuleDetector
// (oimo/path)
// Sphere vs Capsule detector.

type SphereCapsuleDetector struct {
	*Detector
	//TODO
}

func NewSphereCapsuleDetector(swapped bool) *SphereCapsuleDetector {
	return &SphereCapsuleDetector{
		Detector: NewDetector(swapped),
	}
}

// TODO

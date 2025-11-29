package demos

//////////////////////////////////////////////// SphereSphereDetector
// (oimo/path)
// Sphere vs Sphere detector.

type SphereSphereDetector struct {
	*Detector
	//TODO
}

func NewSphereSphereDetector() *SphereSphereDetector {
	return &SphereSphereDetector{
		Detector: NewDetector(false),
	}
}

// TODO

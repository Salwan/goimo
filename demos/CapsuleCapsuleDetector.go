package demos

//////////////////////////////////////////////// CapsuleCapsuleDetector
// (oimo/path)
// Capsule vs Capsule detector.

type CapsuleCapsuleDetector struct {
	*Detector
	//TODO
}

func NewCapsuleCapsuleDetector() *CapsuleCapsuleDetector {
	return &CapsuleCapsuleDetector{
		Detector: NewDetector(false),
	}
}

// TODO

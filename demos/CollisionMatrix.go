package demos

//////////////////////////////////////////////// CollisionMatrix
// (oimo/collision/narrowphase/CollisionMatrix.go)
// CollisionMatrix provides corresponding collision detector for a pair of two geometries of given types.

type CollisionMatrix struct {
	detectors [][]IDetector
}

func NewCollisionMatrix() *CollisionMatrix {
	cm := &CollisionMatrix{
		detectors: make([][]IDetector, 8),
	}

	// TODO: This is to 6 in OimoPhysics which leaves 2 Detectors uninitialized
	for i := range 8 {
		cm.detectors[i] = make([]IDetector, 8)
	}

	gjkEpaDetector := NewGjkEpaDetector()

	sp := GeometryType_SPHERE
	bo := GeometryType_BOX
	cy := GeometryType_CYLINDER
	co := GeometryType_CONE
	ca := GeometryType_CAPSULE
	ch := GeometryType_CONVEX_HULL

	cm.detectors[sp][sp] = NewSphereSphereDetector()
	cm.detectors[sp][bo] = NewSphereBoxDetector(false)
	cm.detectors[sp][cy] = gjkEpaDetector
	cm.detectors[sp][co] = gjkEpaDetector
	cm.detectors[sp][ca] = NewSphereCapsuleDetector(false)
	cm.detectors[sp][ch] = gjkEpaDetector

	cm.detectors[bo][sp] = NewSphereBoxDetector(true)
	cm.detectors[bo][bo] = NewBoxBoxDetector()
	cm.detectors[bo][cy] = gjkEpaDetector
	cm.detectors[bo][co] = gjkEpaDetector
	cm.detectors[bo][ca] = gjkEpaDetector
	cm.detectors[bo][ch] = gjkEpaDetector

	cm.detectors[cy][sp] = gjkEpaDetector
	cm.detectors[cy][bo] = gjkEpaDetector
	cm.detectors[cy][cy] = gjkEpaDetector
	cm.detectors[cy][co] = gjkEpaDetector
	cm.detectors[cy][ca] = gjkEpaDetector
	cm.detectors[cy][ch] = gjkEpaDetector

	cm.detectors[co][sp] = gjkEpaDetector
	cm.detectors[co][bo] = gjkEpaDetector
	cm.detectors[co][cy] = gjkEpaDetector
	cm.detectors[co][co] = gjkEpaDetector
	cm.detectors[co][ca] = gjkEpaDetector
	cm.detectors[co][ch] = gjkEpaDetector

	cm.detectors[ca][sp] = NewSphereCapsuleDetector(true)
	cm.detectors[ca][bo] = gjkEpaDetector
	cm.detectors[ca][cy] = gjkEpaDetector
	cm.detectors[ca][co] = gjkEpaDetector
	cm.detectors[ca][ca] = NewCapsuleCapsuleDetector()
	cm.detectors[ca][ch] = gjkEpaDetector

	cm.detectors[ch][sp] = gjkEpaDetector
	cm.detectors[ch][bo] = gjkEpaDetector
	cm.detectors[ch][cy] = gjkEpaDetector
	cm.detectors[ch][co] = gjkEpaDetector
	cm.detectors[ch][ca] = gjkEpaDetector
	cm.detectors[ch][ch] = gjkEpaDetector

	return cm
}

// --- public ---

// Returns an appropriate collision detector of two geometries of types `geomType1` and `geomType2`.
// This method is **not symmetric**, so `getDetector(a, b)` may not be equal to `getDetector(b, a)`.
func (cm *CollisionMatrix) GetDetector(geomType1, geomType2 GeometryType) IDetector {
	return cm.detectors[geomType1][geomType2]
}

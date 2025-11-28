package demos

/////////////////////////////////////////////// ManifoldUpdater
// (oimo/dynamics/constraint/contact/ManifoldUpdater.go)
// Internal class.

type ManifoldUpdater struct {
	manifold *Manifold

	numOldPoints int
	oldPoints    []*ManifoldPoint
}

func NewManifoldUpdater(manifold *Manifold) *ManifoldUpdater {
	mu := &ManifoldUpdater{
		manifold:  manifold,
		oldPoints: make([]*ManifoldPoint, Settings.MaxManifoldPoints),
	}
	for i := range len(mu.oldPoints) {
		mu.oldPoints[i] = NewManifoldPoint()
	}
	return mu
}

// TODO
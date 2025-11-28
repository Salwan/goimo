package demos

// ////////////////////////////////////////// GjkCache
// (oimo/collision/narrowphase/detector/gjkepa/GjkCache.go)
// Internal class.
type GjkCache struct {
	prevClosestDir Vec3
}

func NewGjkCache() *GjkCache {
	return &GjkCache{}
}

func (g *GjkCache) clear() {
	g.prevClosestDir.Zero()
}

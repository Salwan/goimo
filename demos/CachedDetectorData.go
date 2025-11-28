package demos

// ///////////////////////////////////////////// CachedDetectorData
// (oimo/collision/narrowphase/detector/CachedDetectorData.go)
// This is used for caching narrow-phase data of a pair of collision geometries.
type CachedDetectorData struct {
	gjkCache *GjkCache
}

func NewCachedDetectorData() *CachedDetectorData {
	return &CachedDetectorData{}
}

func (cdd *CachedDetectorData) clear() {
	if cdd.gjkCache != nil {
		cdd.gjkCache.clear()
	}
}

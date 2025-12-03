package demos

//////////////////////////////////////////////// BvhInsertionStrategy
// (oimo/collision/broadphase/bvh/BvhInsertionStrategy.go)
// Internal class.
// Strategies of leaf insertion.

type BvhInsertionStrategy int

const (
	BvhInsertionStrategy_SIMPLE BvhInsertionStrategy = iota
	BvhInsertionStrategy_MINIMIZE_SURFACE_AREA
)

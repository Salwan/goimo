package demos

//////////////////////////////////////////////// BvhInsertionStrategy
// (oimo/collision/broadphase/bvh/BvhInsertionStrategy.go)
// Internal class.
// Strategies of leaf insertion.

type BvhInsertionStrategy int

const (
	_SIMPLE BvhInsertionStrategy = iota
	_MINIMIZE_SURFACE_AREA
)

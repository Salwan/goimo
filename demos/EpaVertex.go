package demos

////////////////////////////////////////// EpaVertex
// (oimo/collision/narrowphase/detector/gjkepa/EpaVertex.go)
// Internal class.

import "math/rand"

type EpaVertex struct {
	next *EpaVertex // for object pooling

	v  Vec3
	w1 Vec3
	w2 Vec3

	tmpEdgeLoopNext          *EpaVertex
	tmpEdgeLoopOuterTriangle *EpaTriangle

	randInt int
}

func NewEpaVertex() *EpaVertex {
	return &EpaVertex{
		randInt: rand.Intn(100000),
	}
}

// TODO

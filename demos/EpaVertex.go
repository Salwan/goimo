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

	randId int
}

func NewEpaVertex() *EpaVertex {
	return &EpaVertex{
		randId: rand.Intn(100000),
	}
}

// --- single linked list interface ---

func (c *EpaVertex) GetNext() *EpaVertex {
	return c.next
}

func (c *EpaVertex) SetNext(x *EpaVertex) {
	c.next = x
}

func (self *EpaVertex) Set(v, w1, w2 Vec3) *EpaVertex {
	self.v = v
	self.w1 = w1
	self.w2 = w2
	self.next = nil
	self.tmpEdgeLoopNext = nil
	self.tmpEdgeLoopOuterTriangle = nil
	return self
}

func (self *EpaVertex) removeReferences() {
	self.next = nil
	self.tmpEdgeLoopNext = nil
	self.tmpEdgeLoopOuterTriangle = nil
}

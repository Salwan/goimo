package demos

//////////////////////////////////////////////// BvhBroadPhase
// (oimo/collision/broadphase/bvh/BvhBroadPhase.go)
// The broad-phase collision detection algorithm based on bounding volume hierarchy (BVH). Average time complexity is O(NlogN) or lower.

type BvhBroadPhase struct {
	*BroadPhase

	// TODO
}

func NewBvhBroadPhase() *BvhBroadPhase {
	return &BvhBroadPhase{
		BroadPhase: NewBroadPhase(_BVH),
	}
}

// TODO

package demos

// Stubs for objects

// ///////////////////////// BroadPhase
type BruteForceBroadPhase struct {
	*BroadPhase
}

func NewBruteForceBroadPhase() *BruteForceBroadPhase {
	return &BruteForceBroadPhase{
		BroadPhase: NewBroadPhase(_BRUTE_FORCE),
	}
}

type BvhBroadPhase struct {
	*BroadPhase
}

func NewBvhBroadPhase() *BvhBroadPhase {
	return &BvhBroadPhase{
		BroadPhase: NewBroadPhase(_BVH),
	}
}

//////////////////////////// CollisionMatrix

type CollisionMatrix struct{}

func NewCollisionMatrix() CollisionMatrix {
	return CollisionMatrix{}
}

// ///////////////////////// RayCastWrapper
type RayCastWrapper struct{}

func NewRayCastWrapper() *RayCastWrapper {
	return &RayCastWrapper{}
}

// ///////////////////////// ConvexCastWrapper
type ConvexCastWrapper struct{}

func NewConvexCastWrapper() *ConvexCastWrapper {
	return &ConvexCastWrapper{}
}

// ///////////////////////// AabbTestWrapper
type AabbTestWrapper struct{}

func NewAabbTestWrapper() *AabbTestWrapper {
	return &AabbTestWrapper{}
}

// ///////////////////////// Pool
type Pool struct{}

func NewPool() *Pool {
	return &Pool{}
}

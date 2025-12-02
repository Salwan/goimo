package demos

//////////////////////////////////////////////// BruteForceBroadPhase
// (oimo/collision/broadphase/bruteforce/BruteForceBroadPhase.go)
// Brute force implementation of broad-phase collision detection. Time complexity is O(n^2).

type BruteForceBroadPhase struct {
	*BroadPhase

	// TODO
}

func NewBruteForceBroadPhase() *BruteForceBroadPhase {
	return &BruteForceBroadPhase{
		BroadPhase: NewBroadPhase(BroadPhaseType_BRUTE_FORCE),
	}
}

// TODO

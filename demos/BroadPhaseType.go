package demos

///////////////////////////////// BroadPhaseType
// (oimo/collision/broadphase/BroadPhaseType.go)
// Types of broad-phase algorithms.

type BroadPhaseType int

const (
	// The brute force algorithm searches all the possible pairs of the proxies every time. This is **very slow** and so users should not choose this algorithm without exceptional reasons.
	BroadPhaseType_BRUTE_FORCE BroadPhaseType = iota + 1
	// The BVH algorithm uses bounding volume hierarchy for detecting overlapping pairs of proxies efficiently.
	BroadPhaseType_BVH
)

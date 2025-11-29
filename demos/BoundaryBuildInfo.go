package demos

////////////////////////////////////////////// BoundaryBuildInfo
// (oimo/dynamics/constraint/solver/direct/BoundaryBuildInfo.go)
// Internal class.

type BoundaryBuildInfo struct {
	size int // dimension

	numBounded int
	iBounded   []int // indices
	signs      []int

	numUnbounded int
	iUnbounded   []int // indices

	// numBounded + numUnbounded <= n
}

func NewBoundaryBuildInfo(size int) *BoundaryBuildInfo {
	return &BoundaryBuildInfo{
		size:       size,
		iBounded:   make([]int, size),
		signs:      make([]int, size),
		iUnbounded: make([]int, size),
	}
}

// TODO

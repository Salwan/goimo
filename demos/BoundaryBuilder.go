package demos

////////////////////////////////////////////// BoundaryBuilder
// (oimo/dynamics/constraint/solver/direct/BoundaryBuilder.go)
// Internal class.

type BoundaryBuilder struct {
	numBoundaries int
	boundaries    []*Boundary

	maxRows int
	bbInfo  *BoundaryBuildInfo
}

func NewBoundaryBuilder(maxRows int) *BoundaryBuilder {
	// TODO(oimo): O(2^N) is inefficient?
	return &BoundaryBuilder{
		maxRows:    maxRows,
		boundaries: make([]*Boundary, 1<<maxRows),
		bbInfo:     NewBoundaryBuildInfo(maxRows),
	}
}

// TODO

package demos

//////////////////////////////////////////////// BoundarySelector
// (oimo/dynamics/constraint/solver/direct/BoundarySelector.go)
// Internal Class

type BoundarySelector struct {
	n          int
	indices    []int
	tmpIndices []int
}

func NewBoundarySelector(n int) *BoundarySelector {
	bc := &BoundarySelector{
		n:          n,
		indices:    make([]int, n),
		tmpIndices: make([]int, n),
	}
	for i := range n {
		bc.indices[i] = i
	}
	return bc
}

// TODO

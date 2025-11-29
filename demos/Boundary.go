package demos

////////////////////////////////////////////// Boundary
// (oimo/dynamics/constraint/solver/direct/Boundary.go)
// Internal class

type Boundary struct {
	// number of impulses which are at lower or upper limits
	numBounded int
	// indices of impulses which are at lower or upper limits
	iBounded []int
	// -1: at lower, 1: at upper
	signs []int

	// number of impulses which are not at limits
	numUnbounded int
	// indices of impulses which are not at lower or upper limits
	iUnbounded []int

	// used for impulse computation:
	//     impulse = massMatrix * b
	b []float64

	// the id of mass matrix
	matrixId int
}

func NewBoundary(maxRows int) *Boundary {
	b := &Boundary{
		iBounded:   make([]int, maxRows),
		iUnbounded: make([]int, maxRows),
		signs:      make([]int, maxRows),
		b:          make([]float64, maxRows),
	}
	return b
}

// TODO

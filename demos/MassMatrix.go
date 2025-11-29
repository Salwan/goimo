package demos

//////////////////////////////////////// MassMatrix
// (oimo/dynamics/constraint/solver/direct/MassMatrix.go)
// Internal class.

type MassMatrix struct {
	// matrix size
	size int

	// inverse matrix elements
	invMass           [][]float64
	invMassWithoutCfm [][]float64

	massData []*JointSolverMassDataRow

	cachedSubmatrices [][][]float64
	cachedComputed    []bool
	maxSubatrixId     int

	// temp matrix used for computing an inverse matrix
	tmpMatrix [][]float64
}

func NewMassMatrix(size int) *MassMatrix {
	mm := &MassMatrix{
		size:              size,
		tmpMatrix:         make([][]float64, size),
		invMass:           make([][]float64, size),
		invMassWithoutCfm: make([][]float64, size),
		maxSubatrixId:     1 << size,
		cachedComputed:    make([]bool, 1<<size),
		cachedSubmatrices: make([][][]float64, 1<<size),
	}

	for i := range size {
		mm.tmpMatrix[i] = make([]float64, size)
		mm.invMass[i] = make([]float64, size)
		mm.invMassWithoutCfm[i] = make([]float64, size)
	}

	for i := range mm.maxSubatrixId {
		// popcount (assuming the size of the matrix is less than 0x100 = 256)
		t := i
		t = (t & 0x55) + (t >> 1 & 0x55)
		t = (t & 0x33) + (t >> 2 & 0x33)
		t = (t & 0xf) + (t >> 4 & 0xf)
		matrixSize := t

		subMatrix := make([][]float64, matrixSize)
		for j := range matrixSize {
			subMatrix[j] = make([]float64, matrixSize)
		}
		mm.cachedSubmatrices[i] = subMatrix
	}

	return mm
}

// TODO

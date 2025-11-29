package demos

///////////////////////////////////////////////// DirectJointConstraintSolver
// (oimo/dynamics/constraint/solver/direct/DirectJointConstraintSolver.go)
// The direct solver of a mixed linear complementality problem (MLCP) for joint constraints.

type DirectJointConstraintSolver struct {
	*ConstraintSolver

	info     *JointSolverInfo
	massData []*JointSolverMassDataRow

	relVels        []float64
	impulses       []float64
	dImpulses      []float64
	dTotalImpulses []float64

	joint *Joint

	massMatrix *MassMatrix

	boundaryBuilder *BoundaryBuilder

	velBoundarySelector *BoundarySelector
	posBoundarySelector *BoundarySelector
}

func NewDirectJointConstraintSolver(joint *Joint) *DirectJointConstraintSolver {
	maxRows := Settings.MaxJacobianRows
	d := &DirectJointConstraintSolver{
		ConstraintSolver: NewConstraintSolver(),
		joint:            joint,
		info:             NewJointSolverInfo(),
		massMatrix:       NewMassMatrix(maxRows),
		boundaryBuilder:  NewBoundaryBuilder(maxRows),
		massData:         make([]*JointSolverMassDataRow, maxRows),
		relVels:          make([]float64, maxRows),
		impulses:         make([]float64, maxRows),
		dImpulses:        make([]float64, maxRows),
		dTotalImpulses:   make([]float64, maxRows),
	}

	for i := range maxRows {
		d.massData[i] = NewJointSolverMassDataRow()
	}

	numBounds := len(d.boundaryBuilder.boundaries)
	d.velBoundarySelector = NewBoundarySelector(numBounds)
	d.posBoundarySelector = NewBoundarySelector(numBounds)

	return d
}

// TODO

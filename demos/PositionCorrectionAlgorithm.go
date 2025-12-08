package demos

///////////////////// PositionCorrectAlgorithm
// (oimo/dynamics/constraint/PositionCorrectionAlgorithm.go)
// The list of the algorithms for position corretion.

type PositionCorrectionAlgorithm int

const (
	PositionCorrectionAlgorithm_BAUMGARTE PositionCorrectionAlgorithm = iota
	PositionCorrectionAlgorithm_SPLIT_IMPULSE
	PositionCorrectionAlgorithm_NGS
)

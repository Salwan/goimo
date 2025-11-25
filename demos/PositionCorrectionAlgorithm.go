package demos

///////////////////// PositionCorrectAlgorithm
// (oimo/dynamics/constraint/PositionCorrectionAlgorithm.go)
// The list of the algorithms for position corretion.

type PositionCorrectionAlgorithm int

const (
	_BAUMGARTE PositionCorrectionAlgorithm = iota
	_SPLIT_IMPULSE
	_NGS
)

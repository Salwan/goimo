package demos

//////////////////////////////////////////////// EpaPolyhedronState
// (oimo/collision/narrowphase/detector/gjkepa/EpaPolyhedronState.go)
// Internal class.

type EpaPolyhedronState int

const (
	EpaPolyhedronState_OK EpaPolyhedronState = iota
	EpaPolyhedronState_INVALID_TRIANGLE
	EpaPolyhedronState_NO_ADJACENT_PAIR_INDEX
	EpaPolyhedronState_NO_ADJACENT_TRIANGLE
	EpaPolyhedronState_EDGE_LOOP_BROKEN
	EpaPolyhedronState_NO_OUTER_TRIANGLE
	EpaPolyhedronState_TRIANGLE_INVISIBLE
)

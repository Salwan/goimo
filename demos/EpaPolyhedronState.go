package demos

//////////////////////////////////////////////// EpaPolyhedronState
// (oimo/collision/narrowphase/detector/gjkepa/EpaPolyhedronState.go)
// Internal class.

type EpaPolyhedronState int

const (
	_OK EpaPolyhedronState = iota
	_INVALID_TRIANGLE
	_NO_ADJACENT_PAIR_INDEX
	_NO_ADJACENT_TRIANGLE
	_EDGE_LOOP_BROKEN
	_NO_OUTER_TRIANGLE
	_TRIANGLE_INVISIBLE
)

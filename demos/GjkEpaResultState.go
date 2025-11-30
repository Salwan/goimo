package demos

//////////////////////////////////////////////// GjkEpaResultState
// (oimo/collision/narrowphase/detector/gjkepa/GjkEpaResultState.go)
// The list of the state of a result of `GjkEpa.computeClosestPoints`.

type GjkEpaResultState int

const (
	_SUCCEEDED                      GjkEpaResultState = 0x000
	_GJK_FAILED_TO_MAKE_TETRAHEDRON GjkEpaResultState = 0x001
	_GJK_DID_NOT_CONVERGE           GjkEpaResultState = 0x002
	_EPA_FAILED_TO_INIT             GjkEpaResultState = 0x101
	_EPA_FAILED_TO_ADD_VERTEX       GjkEpaResultState = 0x102
	_EPA_DID_NOT_CONVERGE           GjkEpaResultState = 0x103
)

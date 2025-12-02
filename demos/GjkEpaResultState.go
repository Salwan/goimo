package demos

//////////////////////////////////////////////// GjkEpaResultState
// (oimo/collision/narrowphase/detector/gjkepa/GjkEpaResultState.go)
// The list of the state of a result of `GjkEpa.computeClosestPoints`.

type GjkEpaResultState int

const (
	GjkEpaResultState_SUCCEEDED                      GjkEpaResultState = 0x000
	GjkEpaResultState_GJK_FAILED_TO_MAKE_TETRAHEDRON GjkEpaResultState = 0x001
	GjkEpaResultState_GJK_DID_NOT_CONVERGE           GjkEpaResultState = 0x002
	GjkEpaResultState_EPA_FAILED_TO_INIT             GjkEpaResultState = 0x101
	GjkEpaResultState_EPA_FAILED_TO_ADD_VERTEX       GjkEpaResultState = 0x102
	GjkEpaResultState_EPA_DID_NOT_CONVERGE           GjkEpaResultState = 0x103
)

package demos

/////////////////////////////////////// EpaTriangle
// (oimo/collision/narrowphase/detector/gjkepa/EpaTriangle.go)
// Internal class.

type EpaTriangle struct {
	next *EpaTriangle
	prev *EpaTriangle

	vertices          []EpaVertex
	adjacentTriangles []EpaTriangle
	adjacentPairIndex []int
	normal            Vec3
	distanceSq        float64

	nextIndex []int // (0, 1, 2) -> (1, 2, 0)

	tmpDfsId      int
	tmpDfsVisible bool

	tmp Vec3
	id  int
}

var _epaTriangle_nextIndex int = 0

func NewEpaTriangle() *EpaTriangle {
	et := EpaTriangle{
		vertices:          make([]EpaVertex, 3),
		adjacentTriangles: make([]EpaTriangle, 3),
		adjacentPairIndex: make([]int, 3),
		nextIndex:         []int{1, 2, 0},
		id:                _epaTriangle_nextIndex,
	}
	_epaTriangle_nextIndex++
	return &et
}

// TODO

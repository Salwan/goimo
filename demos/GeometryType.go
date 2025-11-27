package demos

/////////////////////////////////// GeometryType
// (oimo/collision/geometry/GeometryType.go)
// The list of collision geometry types.

type GeometryType int

const (
	_SPHERE GeometryType = iota
	_BOX
	_CYLINDER
	_CONE
	_CAPSULE
	_CONVEX_HULL
)

const _CONVEX_MIN = 0
const _CONVEX_MAX = 5

package demos

/////////////////////////////////// GeometryType
// (oimo/collision/geometry/GeometryType.go)
// The list of collision geometry types.

type GeometryType int

const (
	GeometryType_SPHERE GeometryType = iota
	GeometryType_BOX
	GeometryType_CYLINDER
	GeometryType_CONE
	GeometryType_CAPSULE
	GeometryType_CONVEX_HULL
)

const GeometryType_CONVEX_MIN = 0
const GeometryType_CONVEX_MAX = 5

package demos

// ///////////////////////////////////// ConvexGeometry
// (oimo/collision/geometry/ConvexGeometry.go)
// Abstract class of the convex collision geometries supported by GJK/EPA collision detection.

type ConvexGeometry struct {
	*Geometry

	// TODO(Oimo): divide margin into "inner" margin and "outer" margin

	gjkMargin     float64 // Gjk Margin should not < 0, use SetGjkMargin()
	useGjkRayCast bool
}

func NewConvexGeometry(_type_ GeometryType) *ConvexGeometry {
	return &ConvexGeometry{
		Geometry:  NewGeometry(_type_),
		gjkMargin: Settings.DefaultGJKMargin,
	}
}

// Gjk Margin should not < 0
func (cg *ConvexGeometry) SetGjkMargin(gjk_margin float64) {
	if gjk_margin < 0 {
		gjk_margin = 0
	}
	cg.gjkMargin = gjk_margin
}

// Computes supporting vertex of the "core" of the geometry in local coordinates. Note that the direction vector `dir` might not be normalized. `out` is set to the computed supporting vertex.
func (cg *ConvexGeometry) ComputeLocalSupportingVertex(dir Vec3, out *Vec3) {}

func (cg *ConvexGeometry) RayCast(begin, end *Vec3, transform *Transform, hit *RayCastHit) bool {
	if cg.useGjkRayCast {
		return GjkEpaInstance.RayCast(cg, transform, begin, end, hit)
	} else {
		return cg.Geometry.Raycast(begin, end, transform, hit)
	}
}

package demos

// ////////////////////// Geometry
// (oimo/collision/geometry/Geometry.go)

type IGeometry interface {
	UpdateMass()
	ComputeAabb(aabb *Aabb, tf *Transform)
	RayCastLocal(begin, end Vec3, hit *RayCastHit) bool
	RayCast(begin, end Vec3, transform *Transform, hit *RayCastHit) bool
	GetType() GeometryType
	GetInertiaCoeff() *Mat3
	GetVolume() float64
}

type Geometry struct { // implements IGeometry
	_type        GeometryType
	volume       float64
	inertiaCoeff Mat3 // I / mass
}

func NewGeometry(_type_ GeometryType) *Geometry {
	g := &Geometry{
		_type: _type_,
	}
	g.inertiaCoeff.Identity()
	return g
}

func (geo *Geometry) UpdateMass() { // override

}

func (geo *Geometry) ComputeAabb(aabb *Aabb, tf *Transform) { // override

}

func (geo *Geometry) RayCastLocal(begin, end Vec3, hit *RayCastHit) bool { // override
	return false
}

// Performs ray casting. Returns `true` and sets the result information to `hit` if
// the line segment from `begin` to `end` and the geometry transformed by `transform`
// intersect. Returns `false` if the line segment and the geometry do not intersect.
func (geo *Geometry) RayCast(begin, end Vec3, transform *Transform, hit *RayCastHit) bool { // override
	beginLocal := begin
	endLocal := end

	MathUtil.Vec3_sub(&beginLocal, &beginLocal, &transform.position)
	MathUtil.Vec3_sub(&endLocal, &endLocal, &transform.position)

	MathUtil.Vec3_mulMat3Transposed(&beginLocal, &beginLocal, &transform.rotation)
	MathUtil.Vec3_mulMat3Transposed(&endLocal, &endLocal, &transform.rotation)

	if geo.RayCastLocal(beginLocal, endLocal, hit) {
		// local -> global
		MathUtil.Vec3_mulMat3(&hit.Position, &hit.Position, &transform.rotation)
		MathUtil.Vec3_mulMat3(&hit.Normal, &hit.Normal, &transform.rotation)
		MathUtil.Vec3_add(&hit.Position, &hit.Position, &transform.position)
		return true
	}

	return false
}

func (geo *Geometry) GetType() GeometryType { // override
	return geo._type
}

func (self *Geometry) GetInertiaCoeff() *Mat3 { // override
	return &self.inertiaCoeff
}

func (self *Geometry) GetVolume() float64 { // override
	return self.volume
}

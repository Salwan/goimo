package demos

// /////////////////////////////////////////// Manifold
// (oimo/dynamics/constraint/contact/Manifold.go)
// A contact manifold holds collision data of a pair of shapes. (size=80+b)
type Manifold struct {
	normal    Vec3
	tangent   Vec3
	binormal  Vec3
	numPoints int
	points    []*ManifoldPoint
}

func NewManifold() *Manifold {
	m := &Manifold{
		points: make([]*ManifoldPoint, Settings.MaxManifoldPoints),
	}
	for i := range len(m.points) {
		m.points[i] = NewManifoldPoint()
	}
	return m
}

// --- internal ---

func (m *Manifold) clear() {
	for i := range m.numPoints {
		m.points[i].clear()
	}
	m.numPoints = 0
}

func (self *Manifold) buildBasis(normal Vec3) {
	self.normal = normal
	nx, ny, nz := normal.x, normal.y, normal.z
	nx2, ny2, nz2 := nx*nx, ny*ny, nz*nz
	var tx, ty, tz, bx, by, bz float64

	if nx2 < ny2 && nx2 < nz2 {
		invL := 1.0 / MathUtil.Sqrt(ny2+nz2)
		tx = 0
		ty = -nz * invL
		tz = ny * invL
		bx = ny*tz - nz*ty
		by = -nx * tz
		bz = nx * ty
	} else if nx2 >= ny2 && ny2 < nz2 {
		invL := 1.0 / MathUtil.Sqrt(nx2+nz2)
		tx = nz * invL
		ty = 0
		tz = -nx * invL
		bx = ny * tz
		by = nz*tx - nx*tz
		bz = -ny * tx
	} else {
		invL := 1.0 / MathUtil.Sqrt(nx2+ny2)
		tx = -ny * invL
		ty = nx * invL
		tz = 0
		bx = -nz * ty
		by = nz * tx
		bz = nx*ty - ny*tx
	}
	self.tangent = Vec3{tx, ty, tz}
	self.binormal = Vec3{bx, by, bz}
}

func (self *Manifold) updateDepthsAndPositions(tf1, tf2 *Transform) {
	for i := range self.numPoints {
		p := self.points[i]
		p.relPos1 = p.localPos1.MulMat3(&tf1.rotation)
		p.relPos2 = p.localPos2.MulMat3(&tf2.rotation)
		p.pos1 = p.relPos1.Add(tf1.position)
		p.pos2 = p.relPos2.Add(tf2.position)

		diff := p.pos1.Sub(p.pos2)
		dotN := diff.Dot(self.normal)
		p.depth = -dotN
	}
}

// --- public ---

// Returns the normal vector of the contact manifold. The normal vector has unit length and is perpendicular to the contact plane.
func (self *Manifold) GetNormal() Vec3 {
	return self.normal
}

// Sets `normal` to the normal vector of the contact manifold. The normal vector has unit length and is perpendicular to the contact plane.
// This does not create a new instance of `Vec3`.
func (self *Manifold) GetNormalTo(normal *Vec3) {
	*normal = self.normal
}

// Returns the tangent vector of the contact manifold. The tangent vector has unit length and is perpendicular to the normal vector.
func (self *Manifold) GetTangent() Vec3 {
	return self.tangent
}

// Sets `tangent` to the tangent vector of the contact manifold. The tangent vector has unit length and is perpendicular to the normal vector.
// This does not create a new instance of `Vec3`.
func (self *Manifold) GetTangentTo(tangent *Vec3) {
	*tangent = self.tangent
}

// Returns the binormal vector of the contact manifold. The binormal vector has unit length and is perpendicular to both the normal and the tangent vector.
func (self *Manifold) GetBinormal() Vec3 {
	return self.binormal
}

// Sets `binormal` to the binormal vector of the contact manifold. The binormal vector has unit length and is perpendicular to both the normal and the tangent vector.
// This does not create a new instance of `Vec3`.
func (self *Manifold) GetBinormalTo(binormal *Vec3) {
	*binormal = self.binormal
}

// Returns the manifold point vector of the contact manifold. Note that **only the first `Manifold.getNumPoints` elements of the vector are in use**, and the manifold points may be disabled (see `ManifoldPoint.isEnabled`).
func (self *Manifold) GetPoints() []*ManifoldPoint {
	return self.points
}

// Returns the number of existing manifold points.
func (self *Manifold) GetNumPoints() int {
	return self.numPoints
}

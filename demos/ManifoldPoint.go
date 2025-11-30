package demos

// ////////////////////////////////////////// ManifoldPoint
// (oimo/dynamics/constraint/contact/ManifoldPoint.go)
// A manifold point is a contact point in a contact manifold. This holds detailed collision data (position, overlap depth, impulse, etc...) for collision response.
type ManifoldPoint struct {
	// manifold point relative to rigid bodies. NOT SHAPES.
	localPos1 Vec3
	localPos2 Vec3

	// local position with rotation
	relPos1 Vec3
	relPos2 Vec3

	// world position
	pos1  Vec3
	pos2  Vec3
	depth float64

	impulse ContactImpulse

	warmStarted bool

	// manifold points can be disabled for some reasons (separated, etc...)
	disabled bool

	id int
}

func NewManifoldPoint() *ManifoldPoint {
	return &ManifoldPoint{
		id: -1,
	}
}

// --- internal ---

func (self *ManifoldPoint) clear() {
	self.localPos1.Zero()
	self.localPos2.Zero()
	self.relPos1.Zero()
	self.relPos1.Zero()
	self.pos1.Zero()
	self.pos2.Zero()
	self.depth = 0
	self.impulse.clear()
	self.warmStarted = false
	self.disabled = false
	self.id = -1
}

func (self *ManifoldPoint) initialize(result *DetectorResultPoint, tf1, tf2 *Transform) {
	// world position
	self.pos1 = result.position1
	self.pos2 = result.position2

	// local position w rotation
	self.relPos1 = self.pos1.Sub(tf1.position)
	self.relPos2 = self.pos2.Sub(tf2.position)

	// local position
	self.localPos1 = self.relPos1.MulMat3Transposed(&tf1.rotation)
	self.localPos2 = self.relPos2.MulMat3Transposed(&tf2.rotation)

	self.depth = result.depth

	self.impulse.clear()

	self.id = result.id
	self.warmStarted = false
	self.disabled = false
}

func (self *ManifoldPoint) updateDepthAndPositions(result *DetectorResultPoint, tf1, tf2 *Transform) {
	// world position
	self.pos1 = result.position1
	self.pos2 = result.position2

	// local position w rotation
	self.relPos1 = self.pos1.Sub(tf1.position)
	self.relPos2 = self.pos2.Sub(tf2.position)

	// local position
	self.localPos1 = self.relPos1.MulMat3Transposed(&tf1.rotation)
	self.localPos2 = self.relPos2.MulMat3Transposed(&tf2.rotation)

	self.depth = result.depth
}

func (self *ManifoldPoint) copyFrom(cp *ManifoldPoint) {
	self.localPos1 = cp.localPos1
	self.localPos2 = cp.localPos2
	self.relPos1 = cp.relPos1
	self.relPos2 = cp.relPos2
	self.pos1 = cp.pos1
	self.pos2 = cp.pos2
	self.depth = cp.depth
	self.impulse.copyFrom(&cp.impulse)
	self.id = cp.id
	self.warmStarted = cp.warmStarted
	self.disabled = false
}

// --- public ---

// Returns the first rigid body's manifold point in world coordinate.
func (self *ManifoldPoint) GetPosition1() Vec3 {
	return self.pos1
}

// Sets `position` to the first rigid body's manifold point in world coordinate. This does not create a new instance of `Vec3`.
func (self *ManifoldPoint) GetPosition1To(position *Vec3) {
	*position = self.pos1
}

// Returns the second rigid body's manifold point in world coordinate.
func (self *ManifoldPoint) getPosition2() Vec3 {
	return self.pos2
}

// Sets `position` to the second rigid body's manifold point in world coordinate. This does not create a new instance of `Vec3`.
func (self *ManifoldPoint) GetPosition2To(position *Vec3) {
	*position = self.pos2
}

// Returns the amount of the overlap. If the manifold point is separate, a negative value is returned.
func (self *ManifoldPoint) GetDepth() float64 {
	return self.depth
}

// Returns whether the manifold point has existed for more than two steps.
func (self *ManifoldPoint) IsWarmStarted() bool {
	return self.warmStarted
}

// Returns the normal impulse of the manifold point.
func (self *ManifoldPoint) GetNormalImpulse() float64 {
	return self.impulse.impulseN
}

// Returns the tangent impulse of the manifold point.
func (self *ManifoldPoint) GetTangentImpulse() float64 {
	return self.impulse.impulseT
}

// Returns the binormal impulse of the manifold point.
func (self *ManifoldPoint) GetBinormalImpulse() float64 {
	return self.impulse.impulseB
}

// Returns whether the manifold point is enabled.
func (self *ManifoldPoint) IsEnabled() bool {
	return !self.disabled
}

package demos

// ////////////////////// RigidBody
// (oimo/dynamics/rigidbody/RigidBody.go)
// A rigid body. To add a rigid body to a physics world, create a `RigidBody` instance, create and add shapes via `RigidBody.addShape`, and add the rigid body to the physics world through `World.addRigidBody`. Rigid bodies have three motion types: dynamic, static, and kinematic. See `RigidBodyType` for details of motion types. (size>64)
type RigidBody struct {
	next *RigidBody
	prev *RigidBody

	shapeList     *Shape
	shapeListLast *Shape
	numShapes     int

	vel    Vec3
	angVel Vec3

	pseudoVel    Vec3
	angPseudoVel Vec3

	pTransform Transform
	transform  Transform

	_type RigidBodyType

	sleepTime                        float64
	sleeping                         bool
	autoSleep                        bool
	sleepingVelocityThreshold        float64
	sleepingAngularVelocityThreshold float64
	sleepingTimeThreshold            float64

	mass                            float64
	invMass                         float64
	localInertia                    Mat3
	rotFactor                       Vec3
	invLocalInertia                 Mat3
	invLocalInertiaWithoutRotFactor Mat3
	invInertia                      Mat3

	linearDamping  float64
	angularDamping float64

	force  Vec3
	torque Vec3

	linearContactImpulse  Vec3
	angularContactImpulse Vec3

	world *World

	contactLinkList     *ContactLink
	contactLinkListLast *ContactLink
	numContactLinks     int

	jointLinkList     *JointLink
	jointLinkListLast *JointLink
	numJointLinks     int

	addedToIsland bool
	gravityScale  float64

	userData any
}

func NewRigidBody(config *RigidBodyConfig) *RigidBody {
	rb := &RigidBody{
		vel:                              config.LinearVelocity,
		angVel:                           config.AngularVelocity,
		_type:                            config.Type,
		autoSleep:                        config.AutoSleep,
		sleepingVelocityThreshold:        config.SleepingVelocityThreshold,
		sleepingAngularVelocityThreshold: config.SleepingAngularVelocityThreshold,
		sleepingTimeThreshold:            config.SleepingTimeThreshold,
		linearDamping:                    config.LinearDamping,
		angularDamping:                   config.AngularDamping,
		rotFactor:                        Vec3{1, 1, 1},
		gravityScale:                     1.0,
	}

	rb.pTransform.position = config.Position
	rb.pTransform.rotation = config.Rotation
	rb.transform = rb.pTransform

	return rb
}

// --- double linked list interface ---

func (c *RigidBody) GetNext() *RigidBody {
	return c.next
}

func (c *RigidBody) SetNext(x *RigidBody) {
	c.next = x
}

func (c *RigidBody) GetPrev() *RigidBody {
	return c.prev
}

func (c *RigidBody) SetPrev(x *RigidBody) {
	c.prev = x
}

// --- internal ---

func (rb *RigidBody) integrate(dt float64) {
	switch rb._type {
	case RigidBodyType_DYNAMIC, RigidBodyType_KINEMATIC:
		var translation Vec3
		var rotation Vec3
		MathUtil.Vec3_scale(&translation, &rb.vel, dt)
		MathUtil.Vec3_scale(&rotation, &rb.angVel, dt)

		translationLengthSq := MathUtil.Vec3_dot(&translation, &translation)
		rotationLengthSq := MathUtil.Vec3_dot(&rotation, &rotation)

		if translationLengthSq == 0 && rotationLengthSq == 0 {
			return // no need for integration
		}

		// limit linear velocity
		if translationLengthSq > Settings.MaxTranslationPerStep*Settings.MaxTranslationPerStep {
			lim := Settings.MaxTranslationPerStep / MathUtil.Sqrt(translationLengthSq)
			MathUtil.Vec3_scale(&rb.vel, &rb.vel, lim)
			MathUtil.Vec3_scale(&translation, &translation, lim)
		}

		// limit angular velocity
		if rotationLengthSq > Settings.MaxRotationPerStep*Settings.MaxRotationPerStep {
			lim := Settings.MaxRotationPerStep / MathUtil.Sqrt(rotationLengthSq)
			MathUtil.Vec3_scale(&rb.angVel, &rb.angVel, lim)
			MathUtil.Vec3_scale(&rotation, &rotation, lim)
		}

		// update the transform
		rb.applyTranslation(translation)
		rb.applyRotation(rotation)

	case RigidBodyType_STATIC:
		rb.vel.Zero()
		rb.angVel.Zero()
		rb.pseudoVel.Zero()
		rb.angPseudoVel.Zero()
	}
}

func (rb *RigidBody) integratePseudoVelocity() {
	pseudoVelLengthSq := rb.pseudoVel.Dot(rb.pseudoVel)
	angPseudoVelLengthSq := rb.angPseudoVel.Dot(rb.angPseudoVel)
	if pseudoVelLengthSq == 0 && angPseudoVelLengthSq == 0 {
		return // no need of integration
	}

	switch rb._type {
	case RigidBodyType_DYNAMIC, RigidBodyType_KINEMATIC:
		translation := rb.pseudoVel
		rotation := rb.angPseudoVel

		// clear pseudo velocity
		rb.pseudoVel.Zero()
		rb.angPseudoVel.Zero()

		// update the transform
		rb.applyTranslation(translation)
		rb.applyRotation(rotation)
	case RigidBodyType_STATIC:
		rb.pseudoVel.Zero()
		rb.angPseudoVel.Zero()
	}
}

func (rb *RigidBody) isSleepy() bool {
	return rb.autoSleep &&
		rb.vel.Dot(rb.vel) < rb.sleepingVelocityThreshold*rb.sleepingVelocityThreshold &&
		rb.angVel.Dot(rb.angVel) < rb.sleepingAngularVelocityThreshold*rb.sleepingAngularVelocityThreshold
}

func (rb *RigidBody) isAlone() bool {
	return rb.numContactLinks == 0 && rb.numJointLinks == 0
}

func (rb *RigidBody) applyTranslation(translation Vec3) {
	MathUtil.Vec3_add(&rb.transform.position, &rb.transform.position, &translation)
}

func (rb *RigidBody) applyRotation(rotation Vec3) {
	// compute derivative of the quaternion
	theta := rotation.Length()
	halfTheta := theta * 0.5
	var rotationToSinAxisFactor float64 // sin(halfTheta) / theta;
	var cosHalfTheta float64            // cos(halfTheta)

	if halfTheta < 0.5 {
		// use Maclaurin expansion
		ht2 := halfTheta * halfTheta
		rotationToSinAxisFactor = (1.0 / 2.0) * (1.0 - ht2*(1.0/6.0) + ht2*ht2*(1.0/120.0))
		cosHalfTheta = 1.0 - ht2*(1.0/2.0) + ht2*ht2*(1.0/24.0)
	} else {
		rotationToSinAxisFactor = MathUtil.Sin(halfTheta) / theta
		cosHalfTheta = MathUtil.Cos(halfTheta)
	}

	var sinAxis Vec3
	MathUtil.Vec3_scale(&sinAxis, &rotation, rotationToSinAxisFactor)
	var dq Quat
	MathUtil.Quat_fromVec3AndFloat(&dq, &sinAxis, cosHalfTheta)

	// integrate quaternion
	var q Quat
	MathUtil.Quat_fromMat3(&q, &rb.transform.rotation)
	MathUtil.Quat_mul(&q, &dq, &q)
	MathUtil.Quat_normalize(&q, &q)

	// update rotation
	MathUtil.Mat3_fromQuat(&rb.transform.rotation, &q)

	// update inertia tensor
	rb.updateInvInertia()
}

// call when added/removed/modified shapes
func (self *RigidBody) shapeModified() {
	self._updateMass()
	self.syncShapes()
}

func (rb *RigidBody) syncShapes() {
	for s := rb.shapeList; s != nil; {
		next := s.next

		s.sync(&rb.pTransform, &rb.transform)

		s = next
	}
}

func (self *RigidBody) applyLinearPositionImpulse(imp Vec3) {
	translation := imp.Scale(self.invMass)
	self.applyTranslation(translation)
}

func (self *RigidBody) applyAngularPositionImpulse(imp Vec3) {
	rotation := imp.MulMat3(&self.invInertia)
	self.applyRotation(rotation)
}

// --- private ---

func (self *RigidBody) _updateMass() {
	var totalInertia Mat3
	totalMass := 0.0

	for s := self.shapeList; s != nil; {
		next := s.next

		g := s.geom
		g.UpdateMass()

		mass := s.density * g.GetVolume()
		var inertia Mat3

		// I_transformed = (R * I_localCoeff * R^T) * mass
		MathUtil.Mat3_transformInertia(&inertia, g.GetInertiaCoeff(), &s.localTransform.rotation)
		inertia.ScaleEq(mass)

		// I_cog = |  y*y+z*z  -x*y      -x*z     |
		//         | -x*y       x*x+z*z  -y*z     | * mass
		//         | -x*z      -y*z       x*x+y*y |
		// I = I_transformed + I_cog
		var cogInertia Mat3
		MathUtil.Mat3_inertiaFromCOG(&cogInertia, &s.localTransform.position)
		MathUtil.Mat3_addRhsScaled(&inertia, &inertia, &cogInertia, mass)

		// add mass data
		totalMass += mass
		MathUtil.Mat3_add(&totalInertia, &totalInertia, &inertia)

		s = next
	}

	self.mass = totalMass
	self.localInertia = totalInertia

	self.completeMassData()

	// wake up the rigid body
	self.WakeUp()
}

// compute inverse mass and inertias from _mass and _localInertia
func (self *RigidBody) completeMassData() {
	det := MathUtil.Mat3_det(&self.localInertia)
	if self.mass > 0 && det > 0 && self._type == RigidBodyType_DYNAMIC {
		self.invMass = 1.0 / self.mass
		MathUtil.Mat3_inv(&self.invLocalInertia, &self.localInertia)
		self.invLocalInertiaWithoutRotFactor = self.invLocalInertia
		MathUtil.Mat3_scaleRows(&self.invLocalInertia, &self.invLocalInertiaWithoutRotFactor, self.rotFactor.x, self.rotFactor.y, self.rotFactor.z)
	} else {
		// set mass and inertia zero
		self.invMass = 0
		self.invLocalInertia.Zero()
		self.invLocalInertiaWithoutRotFactor.Zero()

		// ensure minimum mass so as not to diverge into NaN immediately
		if self._type == RigidBodyType_DYNAMIC {
			self.invMass = 1e-9
			MathUtil.Mat3_diagonal(&self.invLocalInertiaWithoutRotFactor, 1e-9, 1e-9, 1e-9)
			MathUtil.Mat3_scaleRows(&self.invLocalInertia, &self.invLocalInertiaWithoutRotFactor, self.rotFactor.x, self.rotFactor.y, self.rotFactor.z)
		}
	}
	self.updateInvInertia()

}

func (rb *RigidBody) updateInvInertia() {
	MathUtil.Mat3_transformInertia(&rb.invInertia, &rb.invLocalInertia, &rb.transform.rotation)
	MathUtil.Mat3_scaleRows(&rb.invInertia, &rb.invInertia, rb.rotFactor.x, rb.rotFactor.y, rb.rotFactor.z)
}

// call when the transform is externally updated
func (self *RigidBody) updateTransformExt() {
	self.pTransform = self.transform
	self.syncShapes()
	self.WakeUp()
}

// --- public ---

// Returns the world position of the rigid body.
func (self *RigidBody) GetPosition() Vec3 {
	return self.transform.position
}

// Sets `position` to the world position of the rigid body.
// This does not create a new instance of `Vec3`.
func (self *RigidBody) GetPositionTo(position *Vec3) {
	*position = self.transform.position
}

// Sets the world position of the rigid body to `position`.
func (self *RigidBody) SetPosition(position Vec3) {
	self.transform.position = position
	self.updateTransformExt()
}

// Translates the position of the rigid body by `translation`.
func (self *RigidBody) Translate(translation Vec3) {
	diff := translation
	MathUtil.Vec3_add(&self.transform.position, &self.transform.position, &diff)
	self.updateTransformExt()
}

// Returns the rotation matrix of the rigid body.
func (self *RigidBody) GetRotation() Mat3 {
	return self.transform.rotation
}

// Sets `rotation` to the rotation matrix of the rigid body.
// This does not create a new instance of `Mat3`.
func (self *RigidBody) GetRotationTo(rotation *Mat3) {
	*rotation = self.transform.rotation
}

// Sets the rotation matrix of the rigid body to `rotation`.
func (self *RigidBody) SetRotation(rotation *Mat3) {
	self.transform.rotation = *rotation

	self.updateInvInertia()
	self.updateTransformExt()
}

// Sets the rotation of the rigid body by Euler angles `eulerAngles` in radians.
func (self *RigidBody) SetRotationXyz(eulerAngles Vec3) {
	MathUtil.Mat3_fromEulerXyz(&self.transform.rotation, &eulerAngles)

	self.updateInvInertia()
	self.updateTransformExt()
}

// Rotates the rigid body by the rotation matrix `rotation`.
func (self *RigidBody) Rotate(rotation *Mat3) {
	MathUtil.Mat3_mul(&self.transform.rotation, rotation, &self.transform.rotation)

	self.updateInvInertia()
	self.updateTransformExt()
}

// Rotates the rigid body by Euler angles `eulerAngles` in radians.
func (self *RigidBody) RotateXyz(eulerAngles Vec3) {
	var rot Mat3
	MathUtil.Mat3_fromEulerXyz(&rot, &eulerAngles)
	MathUtil.Mat3_mul(&self.transform.rotation, &rot, &self.transform.rotation)

	self.updateInvInertia()
	self.updateTransformExt()
}

// Returns the rotation of the rigid body as a quaternion.
func (self *RigidBody) GetOrientation() Quat {
	var q Quat
	MathUtil.Quat_fromMat3(&q, &self.transform.rotation)
	return q
}

// Sets `orientation` to the rotation quaternion of the rigid body.
// This does not create a new instance of `Quat`.
func (self *RigidBody) GetOrientationTo(orientation *Quat) {
	MathUtil.Quat_fromMat3(orientation, &self.transform.rotation)
}

// Sets the rotation of the rigid body from a quaternion `quaternion`.
func (self *RigidBody) SetOrientation(quaternion Quat) {
	MathUtil.Mat3_fromQuat(&self.transform.rotation, &quaternion)

	self.updateInvInertia()
	self.updateTransformExt()
}

// Returns the transform of the rigid body (copy)
func (self *RigidBody) GetTransform() Transform {
	return self.transform
}

// Sets `transform` to the transform of the rigid body.
// This does not create a new instance of `Transform`.
func (self *RigidBody) GetTransformTo(transform *Transform) {
	*transform = self.transform
}

// Sets the transform of the rigid body to `transform`.
// This does not keep any references to `transform`.
func (self *RigidBody) SetTransform(transform *Transform) {
	self.transform = *transform

	self.updateInvInertia()
	self.updateTransformExt()
}

// Returns the mass of the rigid body.
// If the rigid body has infinite mass, `0` will be returned.
func (self *RigidBody) GetMass() float64 {
	return self.mass
}

// Returns the moment of inertia tensor in local space. (copy)
func (self *RigidBody) GetLocalInertia() Mat3 {
	return self.localInertia
}

// Sets `inertia` to the moment of inertia tensor in local space.
// This does not create a new instance of `Mat3`
func (self *RigidBody) GetLocalInertiaTo(inertia *Mat3) {
	*inertia = self.localInertia
}

// Returns the mass data of the rigid body.
func (self *RigidBody) GetMassData() *MassData {
	md := NewMassData()
	md.Mass = self.mass
	md.LocalInertia = self.localInertia
	return md
}

// Sets `massData` to the mass data of the rigid body.
// This does not create a new instance of `MassData`.
func (self *RigidBody) GetMassDataTo(massData *MassData) {
	massData.Mass = self.mass
	massData.LocalInertia = self.localInertia
}

// Sets the mass and moment of inertia of the rigid body by the mass data `massData`.
// The properties set by this will be overwritten when
// - some shapes are added or removed
// - the type of the rigid body is changed
func (self *RigidBody) SetMassData(massData *MassData) {
	self.mass = massData.Mass
	self.localInertia = massData.LocalInertia
	self.completeMassData()
	self.WakeUp()
}

// Returns the rotation factor of the rigid body.
func (self *RigidBody) GetRotationFactor() Vec3 {
	return self.rotFactor
}

// Sets the rotation factor of the rigid body to `rotationFactor`.
// This changes moment of inertia internally, so that the change of
// angular velocity in **global space** along X, Y and Z axis will scale by `rotationFactor.x`,
// `rotationFactor.y` and `rotationFactor.z` times respectively.
func (self *RigidBody) SetRotationFactor(rotationFactor Vec3) {
	self.rotFactor.CopyFrom(rotationFactor)

	self.updateInvInertia()
	self.WakeUp()
}

// Returns the linear velocity of the rigid body.
func (self *RigidBody) GetLinearVelocity() Vec3 {
	return self.vel
}

// Sets `linearVelocity` to the linear velocity of the rigid body.
// This does not create a new intrance of `Vec3`.
func (self *RigidBody) GetLinearVelocityTo(linearVelocity *Vec3) {
	*linearVelocity = self.vel
}

// Sets the linear velocity of the rigid body.
func (self *RigidBody) SetLinearVelocity(linearVelocity Vec3) {
	if self._type == RigidBodyType_STATIC {
		self.vel.Zero()
	} else {
		self.vel = linearVelocity
	}
	self.WakeUp()
}

// Returns the angular velocity of the rigid body.
func (self *RigidBody) GetAngularVelocity() Vec3 {
	return self.angVel
}

// Sets `angularVelocity` to the angular velocity of the rigid body.
// This does not create a new intrance of `Vec3`.
func (self *RigidBody) GetAngularVelocityTo(angularVelocity *Vec3) {
	// FIX: There's a mistake in OimoPhysics code, it's returning _vel not _angVel
	*angularVelocity = self.angVel
}

// Sets the angular velocity of the rigid body.
func (self *RigidBody) SetAngularVelocity(angularVelocity Vec3) {
	if self._type == RigidBodyType_STATIC {
		self.angVel.Zero()
	} else {
		self.angVel = angularVelocity
	}
	self.WakeUp()
}

// Adds `linearVelocityChange` to the linear velcity of the rigid body.
func (self *RigidBody) AddLinearVelocity(linearVelocityChange Vec3) {
	if self._type != RigidBodyType_STATIC {
		self.vel.AddEq(linearVelocityChange)
	}
	self.WakeUp()
}

// Adds `angularVelocityChange` to the angular velcity of the rigid body.
func (self *RigidBody) AddAngularVelocity(angularVelocityChange Vec3) {
	if self._type != RigidBodyType_STATIC {
		self.angVel.AddEq(angularVelocityChange)
	}
	self.WakeUp()
}

// Applies the impulse `impulse` to the rigid body at `positionInWorld` in world position.
// This changes both the linear velocity and the angular velocity.
func (self *RigidBody) ApplyImpulse(impulse Vec3, positionInWorld Vec3) {
	// linear
	self.vel = self.vel.AddRhsScaled(impulse, self.invMass)

	// angular
	pos := positionInWorld.Sub(self.transform.position)
	aimp := pos.Cross(impulse)
	MathUtil.Vec3_mulMat3(&aimp, &aimp, &self.invInertia)
	self.angVel.AddEq(aimp)

	self.WakeUp()
}

// Applies the linear impulse `impulse` to the rigid body.
// This does not change the angular velocity.
func (self *RigidBody) ApplyLinearImpulse(impulse Vec3) {
	self.vel = self.vel.AddRhsScaled(impulse, self.invMass)
	self.WakeUp()
}

// Applies the angular impulse `impulse` to the rigid body.
// This does not change the linear velocity.
func (self *RigidBody) ApplyAngularImpulse(impulse Vec3) {
	MathUtil.Vec3_mulMat3(&impulse, &impulse, &self.invInertia)
	self.angVel.AddEq(impulse)

	self.WakeUp()
}

// Applies the force `force` to `positionInWorld` in world position.
func (self *RigidBody) ApplyForce(force Vec3, positionInWorld Vec3) {
	// linear
	self.force.AddEq(force)

	// angular
	positionInWorld.SubEq(self.transform.position)
	torque := positionInWorld.Cross(force)
	self.torque.AddEq(torque)

	self.WakeUp()
}

// Applies the force `force` to the center of mass.
func (self *RigidBody) ApplyForceToCenter(force Vec3) {
	// linear
	self.force.AddEq(force)

	self.WakeUp()
}

// Applies the torque `torque`.
func (self *RigidBody) ApplyTorque(torque Vec3) {
	// angular
	self.torque.AddEq(torque)

	self.WakeUp()
}

// Returns the total linear impulse applied by contact constraints.
func (self *RigidBody) GetLinearContactImpulse() Vec3 {
	return self.linearContactImpulse
}

// Sets `linearContactImpulse` to the total linear impulse applied by contact constraints.
// This does not create a new instance of `Vec3`.
func (self *RigidBody) GetLinearContactImpulseTo(linearContactImpulse *Vec3) {
	*linearContactImpulse = self.linearContactImpulse
}

// Returns the total angular impulse applied by contact constraints.
func (self *RigidBody) GetAngularContactImpulse() Vec3 {
	return self.angularContactImpulse
}

// Sets `angularContactImpulse` to the total angular impulse applied by contact constraints.
// This does not create a new instance of `Vec3`.
func (self *RigidBody) GetAngularContactImpulseTo(angularContactImpulse *Vec3) {
	*angularContactImpulse = self.angularContactImpulse
}

// Returns the gravity scaling factor of the rigid body.
func (self *RigidBody) GetGravityScale() float64 {
	return self.gravityScale
}

// Sets the gravity scaling factor of the rigid body to `gravityScale`.
// If `0` is set, the rigid body will not be affected by gravity.
func (self *RigidBody) SetGravityScale(gravityScale float64) {
	self.gravityScale = gravityScale
	self.WakeUp()
}

// Returns the local coordinates of the point `worldPoint` in world coodinates.
func (self *RigidBody) GetLocalPoint(worldPoint Vec3) Vec3 {
	v := worldPoint.Sub(self.transform.position)
	return v.MulMat3Transposed(&self.transform.rotation)
}

// Sets `localPoint` to the local coordinates of the point `worldPoint` in world coodinates.
// This does not create a new instance of `Vec3`.
func (rb *RigidBody) GetLocalPointTo(worldPoint Vec3, localPoint *Vec3) {
	v := worldPoint.Sub(rb.transform.position)
	MathUtil.Vec3_mulMat3Transposed(&v, &v, &rb.transform.rotation)
	*localPoint = v
}

// Returns the local coordinates of the vector `worldVector` in world coodinates.
func (self *RigidBody) GetLocalVector(worldVector Vec3) Vec3 {
	return worldVector.MulMat3Transposed(&self.transform.rotation)
}

// Sets `localVector` to the local coordinates of the vector `worldVector` in world coodinates.
// This does not create a new instance of `Vec3`.
func (self *RigidBody) GetLocalVectorTo(worldVector Vec3, localVector *Vec3) {
	*localVector = worldVector.MulMat3Transposed(&self.transform.rotation)
}

// Returns the world coordinates of the point `localPoint` in local coodinates.
func (self *RigidBody) GetWorldPoint(localPoint Vec3) Vec3 {
	v := localPoint.MulMat3(&self.transform.rotation)
	v.AddEq(self.transform.position)
	return v
}

// Sets `worldPoint` to the world coordinates of the point `localPoint` in local coodinates.
// This does not create a new instance of `Vec3`.
func (self *RigidBody) GetWorldPointTo(localPoint Vec3, worldPoint *Vec3) {
	v := localPoint.MulMat3(&self.transform.rotation)
	*worldPoint = v.Add(self.transform.position)
}

// Returns the world coordinates of the vector `localVector` in local coodinates.
func (self *RigidBody) GetWorldVector(localVector Vec3) Vec3 {
	return localVector.MulMat3(&self.transform.rotation)
}

// Sets `worldVector` to the world coordinates of the vector `localVector` in local coodinates.
// This does not create a new instance of `Vec3`.
func (self *RigidBody) GetWorldVectorTo(localVector Vec3, worldVector *Vec3) {
	*worldVector = localVector.MulMat3(&self.transform.rotation)
}

// Returns the number of the shapes added.
func (self *RigidBody) GetNumShapes() int {
	return self.numShapes
}

// Returns the list of the shapes of the rigid body.
func (self *RigidBody) GetShapeList() *Shape {
	return self.shapeList
}

// Returns the number of the contact lists the rigid body is involved.
func (self *RigidBody) GetNumContactLinks() int {
	return self.numContactLinks
}

// Returns the list of the contact links the rigid body is involved.
func (self *RigidBody) GetContactLinkList() *ContactLink {
	return self.contactLinkList
}

// Returns the number of the joint links the rigid body is attached.
func (self *RigidBody) GetNumJointLinks() int {
	return self.numJointLinks
}

// Returns the list of the joint links the rigid body is attached.
func (self *RigidBody) GetJointLinkList() *JointLink {
	return self.jointLinkList
}

// Adds the shape to the rigid body.
func (self *RigidBody) AddShape(shape *Shape) {
	// first, add the shape to the linked list so that it will be considered
	self.shapeList, self.shapeListLast = DoubleList_push(self.shapeList, self.shapeListLast, shape)
	self.numShapes++
	shape.rigidBody = self

	// then add the shape to the world
	if self.world != nil {
		self.world.addShape(shape)
	}

	self.shapeModified()
}

// Removes the shape from the rigid body.
func (self *RigidBody) RemoveShape(shape *Shape) {
	// first remove the shape from the world
	if self.world != nil {
		self.world.removeShape(shape)
	}

	// then, remove the shape from the linked list so that it will be ignored
	self.shapeList, self.shapeListLast = DoubleList_remove(self.shapeList, self.shapeListLast, shape)
	self.numShapes--
	shape.rigidBody = nil

	self.shapeModified()
}

// Returns the rigid body's type of behaviour.
// See `RigidBodyType` class for details.
func (self *RigidBody) GetType() RigidBodyType {
	return self._type
}

// Sets the rigid body's type of behaviour.
// See `RigidBodyType` class for details.
func (self *RigidBody) SetType(_type_ RigidBodyType) {
	self._type = _type_
	self._updateMass()
}

// Sets the rigid body's sleep flag false.
// This also resets the sleeping timer of the rigid body.
func (self *RigidBody) WakeUp() {
	self.sleeping = false
	self.sleepTime = 0
}

// Sets the rigid body's sleep flag true.
// This also resets the sleeping timer of the rigid body.
func (rb *RigidBody) Sleep() {
	rb.sleeping = true
	rb.sleepTime = 0
}

// Returns whether the rigid body is sleeping.
func (self *RigidBody) IsSleeping() bool {
	return self.sleeping
}

// Returns how long the rigid body is stopping moving. This returns `0` if the body
// has already slept.
func (self *RigidBody) GetSleepTime() float64 {
	return self.sleepTime
}

// Sets the rigid body's auto sleep flag.
// If auto sleep is enabled, the rigid body will automatically sleep when needed.
func (self *RigidBody) SetAutoSleep(autoSleepEnabled bool) {
	self.autoSleep = autoSleepEnabled
	self.WakeUp()
}

// Returns the linear damping.
func (self *RigidBody) GetLinearDamping() float64 {
	return self.linearDamping
}

// Sets the linear damping to `damping`.
func (self *RigidBody) SetLinearDamping(damping float64) {
	self.linearDamping = damping
}

// Returns the angular damping.
func (self *RigidBody) GetAngularDamping() float64 {
	return self.angularDamping
}

// Sets the angular damping to `damping`.
func (self *RigidBody) SetAngularDamping(damping float64) {
	self.angularDamping = damping
}

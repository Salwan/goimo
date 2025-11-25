package demos

import "github.com/g3n/engine/math32"

// ////////////////////// RigidBody
// (oimo/dynamics/rigidbody/RigidBody.go)
// A rigid body. To add a rigid body to a physics world, create a `RigidBody` instance, create and add shapes via `RigidBody.addShape`, and add the rigid body to the physics world through `World.addRigidBody`. Rigid bodies have three motion types: dynamic, static, and kinematic. See `RigidBodyType` for details of motion types.

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

func (rb *RigidBody) Integrate(dt float64) {
	switch rb._type {
	case _DYNAMIC, _KINEMATIC:
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
		rb.applyTranslation(&translation)
		rb.applyRotation(&rotation)
		// TODO
	case _STATIC:
		rb.vel.Zero()
		rb.angVel.Zero()
		rb.pseudoVel.Zero()
		rb.angPseudoVel.Zero()
	}
	// TODO
}

func (rb *RigidBody) applyTranslation(translation *Vec3) {
	MathUtil.Vec3_add(&rb.transform.position, &rb.transform.position, translation)
}

func (rb *RigidBody) applyRotation(rotation *Vec3) {
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
	MathUtil.Vec3_scale(&sinAxis, rotation, rotationToSinAxisFactor)
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
	// updateInvInertia()

	// TODO
}

func (rb *RigidBody) IsSleepy() bool {
	return rb.autoSleep &&
		rb.vel.Dot(rb.vel) < rb.sleepingVelocityThreshold*rb.sleepingVelocityThreshold &&
		rb.angVel.Dot(rb.angVel) < rb.sleepingAngularVelocityThreshold*rb.sleepingAngularVelocityThreshold
}

func (rb *RigidBody) IsAlone() bool {
	return rb.numContactLinks == 0 && rb.numJointLinks == 0
}

func (rb *RigidBody) SyncShapes() {
	// TODO
}

func (rb *RigidBody) AddShape(shape *Shape)                             {}
func (rb *RigidBody) SetAngularVelocity(angularVelocity math32.Vector3) {}
func (rb *RigidBody) Sleep() {
	rb.sleeping = true
	rb.sleepTime = 0
}

package demos

// ////////////////////// Shape
// (oimo/dynamics/rigidbody/Shape.go)
// A shape is a component of a rigid body. It attaches a collision geometry to the parent rigid body with some physical properties such as coefficients of friction and restitution. The collision geometry can locally be transformed relative to the parent rigid body's center of gravity. (size>64)
type Shape struct {
	id int

	prev      *Shape
	next      *Shape
	rigidBody *RigidBody
	geom      IGeometry

	localTransform Transform
	pTransform     Transform
	transform      Transform

	restitution float64
	friction    float64
	density     float64

	aabb Aabb

	proxy *Proxy

	collisionGroup int
	collisionMask  int

	contactCallback IContactCallback

	displacement Vec3

	userData any // Extra field that users can use for their own purposes.
}

func NewShape(config *ShapeConfig) *Shape {
	s := &Shape{
		id:              -1,
		restitution:     config.Restitution,
		friction:        config.Friction,
		density:         config.Density,
		geom:            config.Geometry,
		localTransform:  Transform{config.Position, config.Rotation},
		collisionGroup:  config.CollisionGroup,
		collisionMask:   config.CollisionMask,
		contactCallback: config.ContactCallback,
	}
	s.pTransform = s.localTransform
	s.transform = s.localTransform
	return s
}

func (sh *Shape) sync(tf1 *Transform, tf2 *Transform) {
	MathUtil.Transform_mul(&sh.pTransform, &sh.localTransform, tf1)
	MathUtil.Transform_mul(&sh.transform, &sh.localTransform, tf2)

	sh.geom.ComputeAabb(&sh.aabb, &sh.pTransform)
	min := sh.aabb.Min
	max := sh.aabb.Max

	sh.geom.ComputeAabb(&sh.aabb, &sh.transform)
	MathUtil.Vec3_min(&sh.aabb.Min, &min, &sh.aabb.Min)
	MathUtil.Vec3_max(&sh.aabb.Max, &max, &sh.aabb.Max)

	if sh.proxy != nil {
		MathUtil.Vec3_sub(&sh.displacement, &sh.transform.position, &sh.pTransform.position)
		sh.rigidBody.world.broadPhase.MoveProxy(sh.proxy, &sh.aabb, sh.displacement)
	}
}

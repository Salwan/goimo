package demos

////////////////////////////////////// ShapeConfig
// (oimo/dynamics/rigidbody/ShapeConfig.go)
// A shape configuration is used for construction of shapes. An instance of this class can safely be reused as a shape will not have any references of a field of this class.

type ShapeConfig struct {
	Position    Vec3      // The shape's local position relative to the parent rigid body's origin.
	Rotation    Mat3      // The shape's local rotation matrix relative to the parent rigid body's rotation.
	Friction    float64   // The coefficient of friction of the shape.
	Restitution float64   // The coefficient of restitution of the shape.
	Density     float64   // The density of the shape, usually in Kg/m^3.
	Geometry    IGeometry // The collision geometry of the shape.

	// The collision group bits the shape belongs to. This is used for collision filtering.
	// Two shapes `shape1` and `shape2` will collide only if both
	// `shape1.collisionGroup & shape2.collisionMask` and
	// `shape2.collisionGroup & shape1.collisionMask` are not zero.
	CollisionGroup int

	// The collision mask bits of the shape. This is used for collision filtering.
	// Two shapes `shape1` and `shape2` will collide only if both
	// `shape1.collisionGroup & shape2.collisionMask` and
	// `shape2.collisionGroup & shape1.collisionMask` are not zero.
	CollisionMask int

	// The contact callback of the shape. The callback methods are called when contact events the shape is involved occurred.
	ContactCallback IContactCallback
}

func NewShapeConfig() *ShapeConfig {
	return &ShapeConfig{
		Friction:       Settings.DefaultFriction,
		Restitution:    Settings.DefaultRestitution,
		Density:        Settings.DefaultDensity,
		CollisionGroup: Settings.DefaultCollisionGroup,
		CollisionMask:  Settings.DefaultCollisionMask,
	}
}

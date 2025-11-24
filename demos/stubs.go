package demos

import "github.com/g3n/engine/math32"

// Stubs for objects

type RigidBodyType int

const (
	STATIC RigidBodyType = iota
	DYNAMIC
)

// ////////////////////// World (oimo/dynamics/World.go)
type World struct{}

func (w *World) AddRigidBody(rigidBody *RigidBody) {}

// ////////////////////// RigidBodyConfig
type RigidBodyConfig struct {
	Type     RigidBodyType
	Position math32.Vector3
}

func NewRigidBodyConfig() *RigidBodyConfig {
	return &RigidBodyConfig{}
}

// ////////////////////// RigidBody (oimo/dynamics/rigidbody/RigidBody.go)
type RigidBody struct{}

func NewRigidBody(config *RigidBodyConfig) *RigidBody {
	return &RigidBody{}
}

func (rb *RigidBody) AddShape(shape *Shape)                             {}
func (rb *RigidBody) SetAngularVelocity(angularVelocity math32.Vector3) {}

// ////////////////////// ShapeConfig
type ShapeConfig struct {
	Geom IGeometry
}

func NewShapeConfig() *ShapeConfig {
	return &ShapeConfig{}
}

// ////////////////////// Shape
type Shape struct{}

func NewShape(config *ShapeConfig) *Shape {
	return &Shape{}
}

// ////////////////////// Geometry
type IGeometry interface{}

// ////////////////////// BoxGeometry
type BoxGeometry struct {
	IGeometry
	HalfExtents math32.Vector3
}

func NewBoxGeometry(halfExtents *math32.Vector3) *BoxGeometry {
	return &BoxGeometry{
		HalfExtents: *halfExtents,
	}
}

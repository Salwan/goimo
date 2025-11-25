package demos

import "github.com/g3n/engine/math32"

// Stubs for objects

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
	HalfExtents math32.Vector3
}

func NewBoxGeometry(halfExtents *math32.Vector3) *BoxGeometry {
	return &BoxGeometry{
		HalfExtents: *halfExtents,
	}
}

// //////////////////////// Joint
type Joint struct{}

// ///////////////////////// BroadPhase
type IBroadPhase interface{}

type BroadPhase struct{}

func NewBroadPhase() *BroadPhase {
	return &BroadPhase{}
}

type BruteForceBroadPhase struct {
	*BroadPhase
}

func NewBruteForceBroadPhase() *BruteForceBroadPhase {
	return &BruteForceBroadPhase{
		BroadPhase: NewBroadPhase(),
	}
}

type BvhBroadPhase struct {
	*BroadPhase
}

func NewBvhBroadPhase() *BvhBroadPhase {
	return &BvhBroadPhase{
		BroadPhase: NewBroadPhase(),
	}
}

//////////////////////////// CollisionMatrix

type CollisionMatrix struct{}

func NewCollisionMatrix() CollisionMatrix {
	return CollisionMatrix{}
}

// ////////////////////////// Contact
type Contact struct{}

// ///////////////////////// RayCastWrapper
type RayCastWrapper struct{}

func NewRayCastWrapper() *RayCastWrapper {
	return &RayCastWrapper{}
}

// ///////////////////////// ConvexCastWrapper
type ConvexCastWrapper struct{}

func NewConvexCastWrapper() *ConvexCastWrapper {
	return &ConvexCastWrapper{}
}

// ///////////////////////// AabbTestWrapper
type AabbTestWrapper struct{}

func NewAabbTestWrapper() *AabbTestWrapper {
	return &AabbTestWrapper{}
}

// ///////////////////////// Pool
type Pool struct{}

func NewPool() *Pool {
	return &Pool{}
}

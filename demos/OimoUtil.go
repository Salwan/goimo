package demos

import (
	"github.com/g3n/engine/math32"
)

// Defines some shortcuts to creating and adding objects to a world.

type OimoUtilNamespace struct{}

var OimoUtil OimoUtilNamespace

func (OimoUtilNamespace) AddBox(w *World, center *Vec3, halfExtents *Vec3, wall bool) (*RigidBody, *DemoBox) {
	bg := NewBoxGeometry(*halfExtents)
	rb := OimoUtil.AddRigidBody(w, center, bg, wall)

	gpos := math32.Vector3{X: float32(center.x), Y: float32(center.y), Z: float32(center.z)}
	gsize := math32.Vector3{X: float32(halfExtents.x), Y: float32(halfExtents.y), Z: float32(halfExtents.z)}
	db := NewDemoBox(rb, gpos, gsize)

	return rb, db
}

func (OimoUtilNamespace) AddRigidBody(w *World, center *Vec3, geom IGeometry, wall bool) *RigidBody {
	shapec := NewShapeConfig()
	shapec.Geometry = geom
	bodyc := NewRigidBodyConfig()
	if wall {
		bodyc.Type = RigidBodyType_STATIC
	} else {
		bodyc.Type = RigidBodyType_DYNAMIC
	}
	bodyc.Position = *center
	body := NewRigidBody(bodyc)
	body.AddShape(NewShape(shapec))
	w.AddRigidBody(body)
	return body
}

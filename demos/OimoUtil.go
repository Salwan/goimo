package demos

import "github.com/g3n/engine/math32"

// Defines some shortcuts to creating and adding objects to a world.

type OimoUtilNamespace struct{}

var OimoUtil OimoUtilNamespace

func (OimoUtilNamespace) AddBox(w *World, center *math32.Vector3, halfExtents *math32.Vector3, wall bool) *RigidBody {
	return OimoUtil.AddRigidBody(w, center, NewBoxGeometry(halfExtents), wall)
}

func (OimoUtilNamespace) AddRigidBody(w *World, center *math32.Vector3, geom IGeometry, wall bool) *RigidBody {
	shapec := NewShapeConfig()
	shapec.Geom = geom
	bodyc := NewRigidBodyConfig()
	if wall {
		bodyc.Type = STATIC
	} else {
		bodyc.Type = DYNAMIC
	}
	bodyc.Position = *center
	body := NewRigidBody(bodyc)
	body.AddShape(NewShape(shapec))
	w.AddRigidBody(body)
	return body
}

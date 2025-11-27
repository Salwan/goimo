package demos

// Defines some shortcuts to creating and adding objects to a world.

type OimoUtilNamespace struct{}

var OimoUtil OimoUtilNamespace

func (OimoUtilNamespace) AddBox(w *World, center *Vec3, halfExtents *Vec3, wall bool) *RigidBody {
	bg := NewBoxGeometry(*halfExtents)
	return OimoUtil.AddRigidBody(w, center, bg, wall)
}

func (OimoUtilNamespace) AddRigidBody(w *World, center *Vec3, geom IGeometry, wall bool) *RigidBody {
	shapec := NewShapeConfig()
	shapec.Geometry = geom
	bodyc := NewRigidBodyConfig()
	if wall {
		bodyc.Type = _STATIC
	} else {
		bodyc.Type = _DYNAMIC
	}
	bodyc.Position = *center
	body := NewRigidBody(bodyc)
	body.AddShape(NewShape(shapec))
	w.AddRigidBody(body)
	return body
}

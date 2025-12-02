package demos

//////////////////////////////////////////////// MassData
// (oimo/dynamics/rigidbody/MassData.go)
// This class holds mass and moment of inertia for a rigid body.

type MassData struct {
	Mass         float64 // Mass. `0` for a non-dynamic rigid body.
	LocalInertia Mat3    // Inertia tensor in local space. Zero matrix for a non-dynamic rigid body.
}

func NewMassData() *MassData {
	return &MassData{}
}

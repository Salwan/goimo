package demos

// ///////////////////////////////////////// ContactImpulse
// (oimo/dynamics/constraint/contact/ContactImpulse.go)
// Internal class. (size=56 bytes)
type ContactImpulse struct {
	// normal impulse
	impulseN float64

	// tangent impulse
	impulseT float64

	// binormal impulse
	impulseB float64

	// position impulse
	impulseP float64

	// lateral impulse
	impulseL Vec3
}

func NewContactImpulse() *ContactImpulse {
	return &ContactImpulse{}
}

// copyFrom() isn't copying impulseP, is this on purpose?
func (imp *ContactImpulse) copyFrom(other *ContactImpulse) {
	imp.impulseN = other.impulseN
	imp.impulseT = other.impulseT
	imp.impulseB = other.impulseB
	imp.impulseL = other.impulseL
}

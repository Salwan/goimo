package demos

///////////////////////////////////////////////// JointImpulse
// (oimo/dynamics/constraint/joint/JointImpulse.go)
// Internal class

type JointImpulse struct {
	impulse  float64
	impulseM float64
	impulseP float64
}

func NewJointImpulse() *JointImpulse {
	return &JointImpulse{}
}

func (j *JointImpulse) clear() {
	j.impulse, j.impulseM, j.impulseP = 0.0, 0.0, 0.0
}

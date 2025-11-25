package demos

////////////////////////// TimeStep
// (oimo/dynamics/TimeStep.go)
// Information of time-step sizes of the simulation.

type TimeStep struct {
	Dt      float64
	InvDt   float64
	DtRatio float64
}

func NewTimeStep() *TimeStep {
	return &TimeStep{
		Dt:      0,
		InvDt:   0,
		DtRatio: 1,
	}
}

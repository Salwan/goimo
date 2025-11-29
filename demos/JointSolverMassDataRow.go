package demos

//////////////////////////////////////////////// JointSolverMassDataRow
// (oimo/dynamics/constraint/solver/common/JointSolverMassDataRow.go)
// Internal class.

type JointSolverMassDataRow struct {
	// impulse ->linear/angular velocity change
	invMLin1 Vec3
	invMLin2 Vec3
	invMAng1 Vec3
	invMAng2 Vec3

	mass           float64
	massWithoutCfm float64
}

func NewJointSolverMassDataRow() *JointSolverMassDataRow {
	return &JointSolverMassDataRow{}
}

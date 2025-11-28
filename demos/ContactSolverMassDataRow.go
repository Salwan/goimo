package demos

//////////////////////////////////////////// ContactSolverMassDataRow
// (oimo/dynamics/constraint/solver/common/ContactSolverMassDataRow.go)
// Internal class.

type ContactSolverMassDataRow struct {
	// normal impulse -> linear/angular velocity change
	invMLinN1 Vec3
	invMLinN2 Vec3
	invMAngN1 Vec3
	invMAngN2 Vec3

	// tangent impulse -> linear/angular velocity change
	invMLinT1 Vec3
	invMLinT2 Vec3
	invMAngT1 Vec3
	invMAngT2 Vec3

	// binormal impulse -> linear/angular velocity change
	invMLinB1 Vec3
	invMLinB2 Vec3
	invMAngB1 Vec3
	invMAngB2 Vec3

	// normal mass
	massN float64

	// tangent/binormal mass matrix for cone friction
	massTB00 float64
	massTB01 float64
	massTB10 float64
	massTB11 float64
}

func NewContactSolverMassDataRow() *ContactSolverMassDataRow {
	return &ContactSolverMassDataRow{}
}

package demos

// /////////////////////////////////////// ConstraintSolverInfo
// (oimo/dynamics/constraint/info/contact/ContactSolverInfo.go)
// Internal class.
type ContactSolverInfo struct {
	b1 *RigidBody
	b2 *RigidBody

	numRows int
	rows    []ContactSolverInfoRow
}

func NewContactSolverInfo() *ContactSolverInfo {
	csi := &ContactSolverInfo{
		rows: make([]ContactSolverInfoRow, Settings.MaxManifoldPoints),
	}
	return csi
}

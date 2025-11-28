package demos

////////////////////////////////////////////// JointSolverInfo
// (oimo/dynamics/constraint/info/joint/JointSolverInfo.go)
// Internal class.

type JointSolverInfo struct {
	b1 *RigidBody
	b2 *RigidBody

	numRows int
	rows    []*JointSolverInfoRow
}

func NewJointSolverInfo() *JointSolverInfo {
	j := &JointSolverInfo{
		rows: make([]*JointSolverInfoRow, Settings.MaxJacobianRows),
	}
	for i := range len(j.rows) {
		j.rows[i] = NewJointSolverInfoRow()
	}
	return j
}

func (j *JointSolverInfo) addRow(impulse *JointImpulse) *JointSolverInfoRow {
	row := j.rows[j.numRows]
	j.numRows++
	row.clear()
	row.impulse = impulse
	return row
}

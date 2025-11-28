package demos

////////////////////////////////////////// JacobianRow
// (oimo/dynamics/constraint/info/JacobianRow.go)
// The row of a Jacobian matrix.

const (
	BIT_LINEAR_SET  = 1
	BIT_ANGULAR_SET = 2
)

type JacobianRow struct {
	lin1 Vec3
	lin2 Vec3
	ang1 Vec3
	ang2 Vec3

	flag int // sparsity flag
}

func NewJacobianRow() *JacobianRow {
	return &JacobianRow{}
}

func (j *JacobianRow) clear() {
	j.lin1.Zero()
	j.lin2.Zero()
	j.ang1.Zero()
	j.ang2.Zero()
}

// TODO

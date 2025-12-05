package demos

///////////////////////////////// Quat
// (oimo/common/Quat.go)
// Quaternion class.

type Quat struct {
	x, y, z, w float64
}

// Creates a new quaternion. The quaternion is identity by default.
func NewQuat(X, Y, Z, W float64) *Quat {
	return &Quat{X, Y, Z, W}
}

// Sets all values at once and returns `this`.
func (q *Quat) Init(x, y, z, w float64) *Quat {
	q.x, q.y, q.z, q.w = x, y, z, w
	return q
}

// Sets the quaternion to identity quaternion and returns `this`.
func (q *Quat) Identity() *Quat {
	q.x, q.y, q.z, q.w = 0, 0, 0, 1
	return q
}

// Sets this quaternion to the representation of the matrix `m`, and returns `this`.
// The matrix `m` must be a rotation matrix, that is, must be orthogonalized and have determinant 1.
func (q *Quat) fromMat3(m *Mat3) *Quat {
	e00, e11, e22 := m.e00, m.e11, m.e22
	t := e00 + e11 + e22
	var s float64
	if t > 0 {
		s = MathUtil.Sqrt(t + 1)
		q.w = 0.5 * s
		s = 0.5 / s
		q.x = (m.e21 - m.e12) * s
		q.y = (m.e02 - m.e20) * s
		q.z = (m.e10 - m.e01) * s
	} else {
		if e00 > e11 && e00 > e22 {
			// e00 is the largest
			s = MathUtil.Sqrt(e00 - e11 - e22 + 1)
			q.x = 0.5 * s
			s = 0.5 / s
			q.y = (m.e01 + m.e10) * s
			q.z = (m.e02 + m.e20) * s
			q.w = (m.e21 - m.e12) * s
		} else if e00 <= e11 && e11 > e22 {
			// e11 is the largest
			s = MathUtil.Sqrt(e11 - e22 - e00 + 1)
			q.y = 0.5 * s
			s = 0.5 / s
			q.x = (m.e01 + m.e10) * s
			q.z = (m.e12 + m.e21) * s
			q.w = (m.e02 - m.e20) * s
		} else {
			// e22 is the largest
			s = MathUtil.Sqrt(e22 - e00 - e11 + 1)
			q.z = 0.5 * s
			s = 0.5 / s
			q.x = (m.e02 + m.e20) * s
			q.y = (m.e12 + m.e21) * s
			q.w = (m.e10 - m.e01) * s
		}
	}
	return q
}

// TODO

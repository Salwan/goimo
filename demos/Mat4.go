package demos

///////////////////////////// Mat4
// (oimo/common/Mat4.go)
// 4x4 Matrix class.
// Note that columns and rows are 0-indexed.

type Mat4 struct {
	e00, e01, e02, e03 float64
	e10, e11, e12, e13 float64
	e20, e21, e22, e23 float64
	e30, e31, e32, e33 float64
}

// Creates a new matrix. The matrix is identity by default.
func NewMat4() *Mat4 {
	return &Mat4{
		e00: 1, e01: 0, e02: 0, e03: 0,
		e10: 0, e11: 1, e12: 0, e13: 0,
		e20: 0, e21: 0, e22: 1, e23: 0,
		e30: 0, e31: 0, e32: 0, e33: 1,
	}
}

// Sets all elements at once and returns `this`.
func (m *Mat4) Set(e00, e01, e02, e03 float64,
	e10, e11, e12, e13 float64,
	e20, e21, e22, e23 float64,
	e30, e31, e32, e33 float64) *Mat4 {
	m.e00, m.e01, m.e02, m.e03 = e00, e01, e02, e03
	m.e10, m.e11, m.e12, m.e13 = e10, e11, e12, e13
	m.e20, m.e21, m.e22, m.e23 = e20, e21, e22, e23
	m.e30, m.e31, m.e32, m.e33 = e30, e31, e32, e33
	return m
}

// Sets this matrix to identity matrix and returns `this`.
func (m *Mat4) Identity() *Mat4 {
	return m.Set(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	)
}

// TODO

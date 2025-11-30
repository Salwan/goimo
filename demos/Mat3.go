package demos

import "fmt"

///////////////////////////// Mat3
// (oimo/common/Mat3.go)
// 3x3 Matrix class.
// Note that columns and rows are 0-indexed.

type Mat3 struct {
	e00, e01, e02 float64
	e10, e11, e12 float64
	e20, e21, e22 float64
}

// Creates a new matrix. The matrix is identity by default.
func NewMat3() *Mat3 {
	return &Mat3{
		e00: 1, e01: 0, e02: 0,
		e10: 0, e11: 1, e12: 0,
		e20: 0, e21: 0, e22: 1,
	}
}

// Sets all elements at once and returns `this`.
func (m *Mat3) Set(e00, e01, e02 float64,
	e10, e11, e12 float64,
	e20, e21, e22 float64) *Mat3 {
	m.e00, m.e01, m.e02 = e00, e01, e02
	m.e10, m.e11, m.e12 = e10, e11, e12
	m.e20, m.e21, m.e22 = e20, e21, e22
	return m
}

// Sets this matrix to identity matrix and returns `this`.
func (m *Mat3) Identity() *Mat3 {
	return m.Set(
		1, 0, 0,
		0, 1, 0,
		0, 0, 1,
	)
}

// Extract a column from Mat3, returns Vec3
func (m *Mat3) GetCol(index int) Vec3 {
	var dst Vec3
	switch index {
	case 0:
		dst.x = m.e00
		dst.y = m.e10
		dst.z = m.e20
	case 1:
		dst.x = m.e01
		dst.y = m.e11
		dst.z = m.e21
	case 2:
		dst.x = m.e02
		dst.y = m.e12
		dst.z = m.e22
	default:
		panic(fmt.Errorf("Mat3.GetCol: invalid index %d", index))
	}
	return dst
}

// TODO

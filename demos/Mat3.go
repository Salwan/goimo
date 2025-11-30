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

// Multiplies 2 Mat3, Implements M.mat3_mul(). Returns Mat3 result
func (m *Mat3) Mul(a, b *Mat3) Mat3 {
	var result Mat3
	result.e00 = a.e00*b.e00 + a.e01*b.e10 + a.e02*b.e20
	result.e01 = a.e00*b.e01 + a.e01*b.e11 + a.e02*b.e21
	result.e02 = a.e00*b.e02 + a.e01*b.e12 + a.e02*b.e22
	result.e10 = a.e10*b.e00 + a.e11*b.e10 + a.e12*b.e20
	result.e11 = a.e10*b.e01 + a.e11*b.e11 + a.e12*b.e21
	result.e12 = a.e10*b.e02 + a.e11*b.e12 + a.e12*b.e22
	result.e20 = a.e20*b.e00 + a.e21*b.e10 + a.e22*b.e20
	result.e21 = a.e20*b.e01 + a.e21*b.e11 + a.e22*b.e21
	result.e22 = a.e20*b.e02 + a.e21*b.e12 + a.e22*b.e22
	return result
}

// Implements M.mat3_mulRhsTransposed(). Returns Mat3 with result
func (m *Mat3) MulRhsTransposed(a, b *Mat3) Mat3 {
	var result Mat3
	result.e00 = a.e00*b.e00 + a.e01*b.e01 + a.e02*b.e02
	result.e01 = a.e00*b.e10 + a.e01*b.e11 + a.e02*b.e12
	result.e02 = a.e00*b.e20 + a.e01*b.e21 + a.e02*b.e22
	result.e10 = a.e10*b.e00 + a.e11*b.e01 + a.e12*b.e02
	result.e11 = a.e10*b.e10 + a.e11*b.e11 + a.e12*b.e12
	result.e12 = a.e10*b.e20 + a.e11*b.e21 + a.e12*b.e22
	result.e20 = a.e20*b.e00 + a.e21*b.e01 + a.e22*b.e02
	result.e21 = a.e20*b.e10 + a.e21*b.e11 + a.e22*b.e12
	result.e22 = a.e20*b.e20 + a.e21*b.e21 + a.e22*b.e22
	return result
}

// TODO

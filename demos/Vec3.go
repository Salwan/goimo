package demos

import "fmt"

////////////////////////////////////////// Vec3
// (oimo/common/Vec3.go)
// 3D Vector class (float64)

type Vec3 struct {
	x, y, z float64
}

// Creates a new vector. The vector is zero vector by default.
func NewVec3(X, Y, Z float64) *Vec3 {
	return &Vec3{X, Y, Z}
}

// Sets all values at once and returns `this`.
func (v *Vec3) Set(x, y, z float64) *Vec3 {
	v.x, v.y, v.z = x, y, z
	return v
}

// Sets this vector to zero vector and returns `this`.
func (v *Vec3) Zero() *Vec3 {
	v.x, v.y, v.z = 0.0, 0.0, 0.0
	return v
}

// Returns `this` + `v`.
func (v *Vec3) Add(other Vec3) Vec3 {
	return Vec3{v.x + other.x, v.y + other.y, v.z + other.z}
}

// Returns (`this.x` + `vx`, `this.y` + `vy`, `this.z` + `vz`).
func (v *Vec3) Add3(vx, vy, vz float64) Vec3 {
	return Vec3{v.x + vx, v.y + vy, v.z + vz}
}

// Returns `this` + `v` * `s`.
func (v *Vec3) AddScaled(other Vec3, s float64) Vec3 {
	return Vec3{v.x + other.x*s, v.y + other.y*s, v.z + other.z*s}
}

// Returns `this` - `v`.
func (v *Vec3) Sub(other Vec3) Vec3 {
	return Vec3{v.x - other.x, v.y - other.y, v.z - other.z}
}

// Returns (`this.x` - `vx`, `this.y` - `vy`, `this.z` - `vz`).
func (v *Vec3) Sub3(vx, vy, vz float64) Vec3 {
	return Vec3{v.x - vx, v.y - vy, v.z - vz}
}

// Returns `this` * `s`.
func (v *Vec3) Scale(s float64) Vec3 {
	return Vec3{v.x * s, v.y * s, v.z * s}
}

// Returns (`this.x` * `sx`, `this.y` * `sy`, `this.z` * `sz`).
func (v *Vec3) Scale3(sx, sy, sz float64) Vec3 {
	return Vec3{v.x * sx, v.y * sy, v.z * sz}
}

// Returns the dot product of `this` and `v`.
func (v *Vec3) Dot(other Vec3) float64 {
	return v.x*other.x + v.y*other.y + v.z*other.z
}

// Returns the cross product of `this` and `v`.
func (v *Vec3) Cross(other Vec3) Vec3 {
	return Vec3{
		v.y*other.z - v.z*other.y,
		v.z*other.x - v.x*other.z,
		v.x*other.y - v.y*other.x,
	}
}

// Sets this vector to `this` + `v` and returns `this`.
func (v *Vec3) AddEq(other Vec3) *Vec3 {
	v.x += other.x
	v.y += other.y
	v.z += other.z
	return v
}

// Sets this vector to (`this.x` + `vx`, `this.y` + `vy`, `this.z` + `vz`) and returns `this`.
func (v *Vec3) Add3Eq(vx, vy, vz float64) *Vec3 {
	v.x += vx
	v.y += vy
	v.z += vz
	return v
}

// Sets this vector to `this` + `v` * `s` and returns `this`.
func (v *Vec3) AddScaledEq(other Vec3, s float64) *Vec3 {
	v.x += other.x * s
	v.y += other.y * s
	v.z += other.z * s
	return v
}

// Returns Vec3 = this + b * s, implements M.vec3_addRhsScaled()
func (v *Vec3) AddRhsScaled(b Vec3, s float64) Vec3 {
	return Vec3{
		v.x + b.x*s,
		v.y + b.y*s,
		v.z + b.z*s,
	}
}

// Sets this vector to `this` - `v` and returns `this`.
func (v *Vec3) SubEq(other Vec3) *Vec3 {
	v.x -= other.x
	v.y -= other.y
	v.z -= other.z
	return v
}

// Sets this vector to (`this.x` - `vx`, `this.y` - `vy`, `this.z` - `vz`) and returns `this`.
func (v *Vec3) Sub3Eq(vx, vy, vz float64) *Vec3 {
	v.x -= vx
	v.y -= vy
	v.z -= vz
	return v
}

// Sets this vector to `this` * `s` and returns `this`.
func (v *Vec3) ScaleEq(s float64) *Vec3 {
	v.x *= s
	v.y *= s
	v.z *= s
	return v
}

// Sets this vector to (`this.x` * `sx`, `this.y` * `sy`, `this.z` * `sz`) and returns `this`.
func (v *Vec3) Scale3Eq(sx, sy, sz float64) *Vec3 {
	v.x *= sx
	v.y *= sy
	v.z *= sz
	return v
}

// Sets this vector to the cross product of `this` and `s`, and returns `this`.
func (v *Vec3) CrossEq(other Vec3) *Vec3 {
	v.x = v.y*other.z - v.z*other.y
	v.y = v.z*other.x - v.x*other.z
	v.z = v.x*other.y - v.y*other.x
	return v
}

// Returns the transformed vector by `m`.
func (v *Vec3) MulMat3(m *Mat3) Vec3 {
	return Vec3{
		v.x*m.e00 + v.y*m.e01 + v.z*m.e02,
		v.x*m.e10 + v.y*m.e11 + v.z*m.e12,
		v.x*m.e20 + v.y*m.e21 + v.z*m.e22,
	}
}

func (v *Vec3) MulMat3Transposed(m *Mat3) Vec3 {
	return Vec3{
		v.x*m.e00 + v.y*m.e10 + v.z*m.e20,
		v.x*m.e01 + v.y*m.e11 + v.z*m.e21,
		v.x*m.e02 + v.y*m.e12 + v.z*m.e22,
	}
}

// Returns the transformed vector by `m`.
func (v *Vec3) MulMat4(m *Mat4) Vec3 {
	return Vec3{
		v.x*m.e00 + v.y*m.e01 + v.z*m.e02 + m.e03,
		v.x*m.e10 + v.y*m.e11 + v.z*m.e12 + m.e13,
		v.x*m.e20 + v.y*m.e21 + v.z*m.e22 + m.e23,
	}
}

// Returns the transformed vector by `tf`.
func (v *Vec3) MulTransform(tf *Transform) Vec3 {
	t := v.MulMat3(&tf.rotation)
	t = t.Add(tf.position)
	return t
}

// Sets this vector to the transformed vector by `m` and returns `this`.
func (v *Vec3) MulMat3Eq(m *Mat3) *Vec3 {
	return v.Set(
		v.x*m.e00+v.y*m.e01+v.z*m.e02,
		v.x*m.e10+v.y*m.e11+v.z*m.e12,
		v.x*m.e20+v.y*m.e21+v.z*m.e22,
	)
}

// Sets this vector to the transformed vector by `m` and returns `this`.
func (v *Vec3) MulMat4Eq(m *Mat4) *Vec3 {
	return v.Set(
		v.x*m.e00+v.y*m.e01+v.z*m.e02+m.e03,
		v.x*m.e10+v.y*m.e11+v.z*m.e12+m.e13,
		v.x*m.e20+v.y*m.e21+v.z*m.e22+m.e23,
	)
}

// Sets this vector to the transformed vector by `tf` and returns `this`.
func (v *Vec3) MulTransformEq(tf *Transform) *Vec3 {
	v.MulMat3Eq(&tf.rotation)
	v.AddEq(tf.position)
	return v
}

// Returns the length of the vector.
func (v *Vec3) Length() float64 {
	return MathUtil.Sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
}

// Returns the squared length of the vector.
func (v *Vec3) LengthSq() float64 {
	return v.x*v.x + v.y*v.y + v.z*v.z
}

// Returns the normalized vector.
// If the length is zero, zero vector is returned.
func (v *Vec3) Normalized() Vec3 {
	invLen := v.Length()
	if invLen > 0.0 {
		invLen = 1.0 / invLen
	}
	return Vec3{v.x * invLen, v.y * invLen, v.z * invLen}
}

// Normalize this vector and returns `this`.
// If the length is zero, this vector is set to zero vector.
func (v *Vec3) Normalize() *Vec3 {
	invLen := v.Length()
	if invLen > 0.0 {
		invLen = 1.0 / invLen
	}
	v.x *= invLen
	v.y *= invLen
	v.z *= invLen
	return v
}

// Returns the nagated vector.
func (v *Vec3) Negate() Vec3 {
	return Vec3{-v.x, -v.y, -v.z}
}

// Negate the vector and returns `this`.
func (v *Vec3) NegateEq() *Vec3 {
	v.x = -v.x
	v.y = -v.y
	v.z = -v.z
	return v
}

// Copies values from `v` and returns `this`.
func (v *Vec3) CopyFrom(other Vec3) *Vec3 {
	v.x = other.x
	v.y = other.y
	v.z = other.z
	return v
}

// Returns a clone of the vector.
// func (v *Vec3) Clone():Vec3 {
// 	return new Vec3(x, y, z);
// }

// Returns the string representation of the vector.
func (v *Vec3) String() string {
	return fmt.Sprintf("Vec3[%f, %f, %f]", MathUtil.ToFixed8(v.x), MathUtil.ToFixed8(v.y), MathUtil.ToFixed8(v.z))
}

// Returns component-wise multiply of this by given vector
func (v *Vec3) CompWiseMul(other Vec3) Vec3 {
	return Vec3{v.x * other.x, v.y * other.y, v.z * other.z}
}

// Component-wise multiply this by given vector and returns this
func (v *Vec3) CompWiseMulEq(other Vec3) *Vec3 {
	v.x = v.x * other.x
	v.y = v.y * other.y
	v.z = v.z * other.z
	return v
}

// Returns result of components multiplied by each other
func (v *Vec3) MulHorizontal() float64 {
	return v.x * v.y * v.z
}

func (v *Vec3) IsZero() bool {
	return v.x == 0 && v.y == 0 && v.z == 0
}

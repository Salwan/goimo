package demos

import (
	"math"
	"math/rand"

	"github.com/g3n/engine/math32"
)

// Provides mathematical operations for internal purposes.

type MathUtilNamespace struct {
	POSITIVE_INFINITY float64
	NEGATIVE_INFINITY float64
	PI                float64
	TWO_PI            float64
	HALF_PI           float64
	TO_RADIANS        float64
	TO_DEGREES        float64
}

var MathUtil = MathUtilNamespace{
	POSITIVE_INFINITY: math.MaxFloat64,
	NEGATIVE_INFINITY: -math.MaxFloat64,
	PI:                math.Pi,
	TWO_PI:            math.Pi * 2,
	HALF_PI:           math.Pi / 2,
	TO_RADIANS:        math.Pi / 180.0,
	TO_DEGREES:        180.0 / math.Pi,
}

func (MathUtilNamespace) Rand() float32 {
	return rand.Float32()
}

// Returns a random `Vec3` from `(min, min, min)` inclusive to `(max, max, max)` exclusive.
func (MathUtilNamespace) RandVec3In(min, max float32) math32.Vector3 {
	return math32.Vector3{
		X: min + MathUtil.Rand()*(max-min),
		Y: min + MathUtil.Rand()*(max-min),
		Z: min + MathUtil.Rand()*(max-min)}
}

func (MathUtilNamespace) Sin(x float64) float64 {
	return math.Sin(x)
}

func (MathUtilNamespace) Cos(x float64) float64 {
	return math.Cos(x)
}

// Returns `Math.sqrt(x)`.
func (MathUtilNamespace) Sqrt(x float64) float64 {
	return math.Sqrt(x)
}

// Returns (float64) -1.0 if x<0, 1.0 otherwise
func (MathUtilNamespace) Sign(x float64) float64 {
	if x < 0 {
		return -1.0
	} else {
		return 1.0
	}
}

// Returns (int) 1 if x>0, -1 otherwise (negative or zero)
func (MathUtilNamespace) SignBiased(x float64) int {
	if x > 0 {
		return 1
	} else {
		return -1
	}
}

// M functions
func (MathUtilNamespace) ToFixed8(x float64) float64 {
	return math.Round(x*1e8) / 1e8
}

////////////////////////////////////////// Vec3

// Multiply (scale) src1 by src2 and store in dst
// dst = src1 * src2
func (MathUtilNamespace) Vec3_scale(dst *Vec3, src1 *Vec3, src2 float64) {
	*dst = src1.Scale(src2)
}

// Multiply (scale) src2 by src3, Add src1 to scaled src2, store in dst
// dst = src1 + src2 * src3
func (MathUtilNamespace) Vec3_addRhsScaled(dst *Vec3, src1 *Vec3, src2 *Vec3, src3 float64) {
	dst.x = src1.x + src2.x*src3
	dst.y = src1.y + src2.y*src3
	dst.z = src1.z + src2.z*src3
}

// Multiply src1 by src2, store in dst
// dst = src1 * src2
func (MathUtilNamespace) Vec3_mulMat3(dst *Vec3, src1 *Vec3, src2 *Mat3) {
	// LLM
	// Load components
	x, y, z := src1.x, src1.y, src1.z

	// Compute temps (avoids aliasing issues)
	tmpX := src2.e00*x + src2.e01*y + src2.e02*z
	tmpY := src2.e10*x + src2.e11*y + src2.e12*z
	tmpZ := src2.e20*x + src2.e21*y + src2.e22*z

	// Store to dst
	dst.x, dst.y, dst.z = tmpX, tmpY, tmpZ
}

// Multiple src1 by src2 (transposed), store in dst
func (MathUtilNamespace) Vec3_mulMat3Transposed(dst *Vec3, src1 *Vec3, src2 *Mat3) {
	x, y, z := src1.x, src1.y, src1.z

	tmpX := src2.e00*x + src2.e10*y + src2.e20*z
	tmpY := src2.e01*x + src2.e11*y + src2.e21*z
	tmpZ := src2.e02*x + src2.e12*y + src2.e22*z

	dst.x, dst.y, dst.z = tmpX, tmpY, tmpZ
}

// Calculate Dot product between src1 and src2
func (MathUtilNamespace) Vec3_dot(src1 *Vec3, src2 *Vec3) float64 {
	return src1.x*src2.x + src1.y*src2.y + src1.z*src2.z
}

// Adds src1 to src2, stores in dst
func (MathUtilNamespace) Vec3_add(dst *Vec3, src1 *Vec3, src2 *Vec3) {
	dst.x = src1.x + src2.x
	dst.y = src1.y + src2.y
	dst.z = src1.z + src2.z
}

// Sub (src1 - src2) store in dst
func (MathUtilNamespace) Vec3_sub(dst *Vec3, src1 *Vec3, src2 *Vec3) {
	dst.x, dst.y, dst.z = src1.x-src2.x, src1.y-src2.y, src1.z-src2.z
}

// Stores smaller src components into dst
func (MathUtilNamespace) Vec3_min(dst *Vec3, src1 *Vec3, src2 *Vec3) {
	if src1.x < src2.x {
		dst.x = src1.x
	} else {
		dst.x = src2.x
	}
	if src1.y < src2.y {
		dst.y = src1.y
	} else {
		dst.y = src2.y
	}
	if src1.z < src2.z {
		dst.z = src1.z
	} else {
		dst.z = src2.z
	}
}

// Stores larger src components into dst
func (MathUtilNamespace) Vec3_max(dst *Vec3, src1 *Vec3, src2 *Vec3) {
	if src1.x > src2.x {
		dst.x = src1.x
	} else {
		dst.x = src2.x
	}
	if src1.y > src2.y {
		dst.y = src1.y
	} else {
		dst.y = src2.y
	}
	if src1.z > src2.z {
		dst.z = src1.z
	} else {
		dst.z = src2.z
	}
}

// set all components to absolute value
func (MathUtilNamespace) Vec3_abs(dst *Vec3, src *Vec3) {
	dst.x = math.Abs(src.x)
	dst.y = math.Abs(src.y)
	dst.z = math.Abs(src.z)
}

// /////////////////////////////////////// Quat

// Creates Quat from x,y,z of src1 Vec3 and w from src2 float
func (MathUtilNamespace) Quat_fromVec3AndFloat(dst *Quat, src1 *Vec3, src2 float64) {
	dst.x, dst.y, dst.z, dst.w = src1.x, src1.y, src1.z, src2
}

// Creates Quat from a Mat3 rotation
func (MathUtilNamespace) Quat_fromMat3(dst *Quat, m *Mat3) {
	trace := m.e00 + m.e11 + m.e22

	if trace > 0 {
		s := MathUtil.Sqrt(trace + 1.0)
		dst.w = 0.5 * s
		s = 0.5 / s
		dst.x = (m.e21 - m.e12) * s
		dst.y = (m.e02 - m.e20) * s
		dst.z = (m.e10 - m.e01) * s
	} else if m.e00 > m.e11 && m.e00 > m.e22 { // m.e00 largest
		s := MathUtil.Sqrt(m.e00 - m.e11 - m.e22 + 1.0)
		dst.x = 0.5 * s
		s = 0.5 / s
		dst.y = (m.e01 + m.e10) * s
		dst.z = (m.e02 + m.e20) * s
		dst.w = (m.e21 - m.e12) * s
	} else if m.e11 > m.e22 { // m.e11 largest
		s := MathUtil.Sqrt(m.e11 - m.e00 - m.e22 + 1.0)
		dst.y = 0.5 * s
		s = 0.5 / s
		dst.x = (m.e01 + m.e10) * s
		dst.z = (m.e12 + m.e21) * s
		dst.w = (m.e02 - m.e20) * s
	} else { // m.e22 largest
		s := MathUtil.Sqrt(m.e22 - m.e00 - m.e11 + 1.0)
		dst.z = 0.5 * s
		s = 0.5 / s
		dst.x = (m.e02 + m.e20) * s
		dst.y = (m.e12 + m.e21) * s
		dst.w = (m.e10 - m.e01) * s
	}
}

// Multiplies 2 quats
func (MathUtilNamespace) Quat_mul(dst *Quat, src1 *Quat, src2 *Quat) {
	ax, ay, az, aw := src1.x, src1.y, src1.z, src1.w
	bx, by, bz, bw := src2.x, src2.y, src2.z, src2.w

	tx := aw*bx + ax*bw + ay*bz - az*by
	ty := aw*by - ax*bz + ay*bw + az*bx
	tz := aw*bz + ax*by - ay*bx + az*bw
	tw := aw*bw - ax*bx - ay*by - az*bz

	dst.x = tx
	dst.y = ty
	dst.z = tz
	dst.w = tw
}

// Normalizes a Quaternion
func (MathUtilNamespace) Quat_normalize(dst *Quat, src *Quat) {
	l := MathUtil.Quat_lengthSq(src)
	if l > 1e-32 {
		l = 1.0 / MathUtil.Sqrt(1.0)
	}
	MathUtil.Quat_scale(dst, src, l)
}

// Length^2 of Quat
func (MathUtilNamespace) Quat_lengthSq(src *Quat) float64 {
	return src.x*src.x + src.y*src.y + src.z*src.z + src.w*src.w
}

// Scale a quat
func (MathUtilNamespace) Quat_scale(dst *Quat, src1 *Quat, src2 float64) {
	dst.x = src1.x * src2
	dst.y = src1.y * src2
	dst.z = src1.z * src2
	dst.w = src1.w * src2
}

// /////////////////////////////////////// Mat3

// Set rotation dst Mat3 from given src Quat
func (MathUtilNamespace) Mat3_fromQuat(dst *Mat3, src *Quat) {
	x, y, z, w := src.x, src.y, src.z, src.w
	x2, y2, z2 := 2*x, 2*y, 2*z
	xx, yy, zz := x*x2, y*y2, z*z2
	xy, yz, xz := x*y2, y*z2, x*z2
	wx, wy, wz := w*x2, w*y2, w*z2

	dst.Set(
		1-yy-zz, xy-wz, xz+wy,
		xy+wz, 1-xx-zz, yz-wx,
		xz-wy, yz+wx, 1-xx-yy,
	)
}

func (MathUtilNamespace) Mat3_transformInertia(dst *Mat3, inertia *Mat3, rotation *Mat3) {
	MathUtil.Mat3_mul(dst, rotation, inertia)
	MathUtil.Mat3_mulRhsTransposed(dst, dst, rotation)
}

// Multiplies two Mat3s
func (MathUtilNamespace) Mat3_mul(dst *Mat3, src1 *Mat3, src2 *Mat3) {
	a, b := src1, src2
	t00 := a.e00*b.e00 + a.e01*b.e10 + a.e02*b.e20
	t01 := a.e00*b.e01 + a.e01*b.e11 + a.e02*b.e21
	t02 := a.e00*b.e02 + a.e01*b.e12 + a.e02*b.e22
	t10 := a.e10*b.e00 + a.e11*b.e10 + a.e12*b.e20
	t11 := a.e10*b.e01 + a.e11*b.e11 + a.e12*b.e21
	t12 := a.e10*b.e02 + a.e11*b.e12 + a.e12*b.e22
	t20 := a.e20*b.e00 + a.e21*b.e10 + a.e22*b.e20
	t21 := a.e20*b.e01 + a.e21*b.e11 + a.e22*b.e21
	t22 := a.e20*b.e02 + a.e21*b.e12 + a.e22*b.e22
	dst.Set(t00, t01, t02, t10, t11, t12, t20, t21, t22)
}

func (MathUtilNamespace) Mat3_mulRhsTransposed(dst *Mat3, src1 *Mat3, src2 *Mat3) {
	a, b := src1, src2
	t00 := a.e00*b.e00 + a.e01*b.e01 + a.e02*b.e02
	t01 := a.e00*b.e10 + a.e01*b.e11 + a.e02*b.e12
	t02 := a.e00*b.e20 + a.e01*b.e21 + a.e02*b.e22
	t10 := a.e10*b.e00 + a.e11*b.e01 + a.e12*b.e02
	t11 := a.e10*b.e10 + a.e11*b.e11 + a.e12*b.e12
	t12 := a.e10*b.e20 + a.e11*b.e21 + a.e12*b.e22
	t20 := a.e20*b.e00 + a.e21*b.e01 + a.e22*b.e02
	t21 := a.e20*b.e10 + a.e21*b.e11 + a.e22*b.e12
	t22 := a.e20*b.e20 + a.e21*b.e21 + a.e22*b.e22
	dst.Set(t00, t01, t02, t10, t11, t12, t20, t21, t22)
}

func (MathUtilNamespace) Mat3_scaleRows(dst *Mat3, src *Mat3, sx, sy, sz float64) {
	dst.e00 = src.e00 * sx
	dst.e01 = src.e01 * sx
	dst.e02 = src.e02 * sx

	dst.e10 = src.e10 * sy
	dst.e11 = src.e11 * sy
	dst.e12 = src.e12 * sy

	dst.e20 = src.e20 * sz
	dst.e21 = src.e21 * sz
	dst.e22 = src.e22 * sz
}

func (MathUtilNamespace) Mat3_scaleCols(dst *Mat3, src *Mat3, sx, sy, sz float64) {
	dst.e00 = src.e00 * sx
	dst.e01 = src.e01 * sy
	dst.e02 = src.e02 * sz

	dst.e10 = src.e10 * sx
	dst.e11 = src.e11 * sy
	dst.e12 = src.e12 * sz

	dst.e20 = src.e20 * sx
	dst.e21 = src.e21 * sy
	dst.e22 = src.e22 * sz
}

// Sets given Mat3 to zeroes and diagonal x y z
func (MathUtilNamespace) Mat3_diagonal(dst *Mat3, x, y, z float64) {
	dst.Set(
		x, 0, 0,
		0, y, 0,
		0, 0, z,
	)
}

///////////////////////////////////////////////// Transform

// Multiply two transforms and store in dst
func (MathUtilNamespace) Transform_mul(dst *Transform, src1 *Transform, src2 *Transform) {
	MathUtil.Mat3_mul(&dst.rotation, &src2.rotation, &src1.rotation)
	MathUtil.Vec3_mulMat3(&dst.position, &src1.position, &src2.rotation)
	MathUtil.Vec3_add(&dst.position, &dst.position, &src2.position)
}

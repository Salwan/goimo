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

// /////////////////////////////////////// Quat

// Creates Quat from x,y,z of src1 Vec3 and w from src2 float
func (MathUtilNamespace) Quat_fromVec3AndFloat(dst *Quat, src1 *Vec3, src2 float64) {
	dst.x, dst.y, dst.z, dst.w = src1.x, src1.y, src1.z, src2
}

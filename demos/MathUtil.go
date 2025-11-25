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

// Returns `Math.sqrt(x)`.
func (MathUtilNamespace) Sqrt(x float64) float64 {
	return math.Sqrt(x)
}

func (MathUtilNamespace) toFixed8(x float64) float64 {
	return math.Round(x*1e8) / 1e8
}

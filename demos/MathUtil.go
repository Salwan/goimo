package demos

import (
	"math/rand"

	"github.com/g3n/engine/math32"
)

// Provides mathematical operations for internal purposes.

type MathUtilNamespace struct{}

var MathUtil MathUtilNamespace

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

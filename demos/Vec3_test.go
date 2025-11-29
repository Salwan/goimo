package demos

import (
	"testing"
)

func TestVec3(t *testing.T) {
	t.Run("Vec3 equal", func(t *testing.T) {
		v0 := Vec3{0, 1, 2}
		v1 := Vec3{0, 1, 2}
		got := v0 == v1
		want := true
		testCheckEqual(t, got, want)
	})

	t.Run("Vec3 add", func(t *testing.T) {
		v1 := Vec3{0, 1, 2}
		v2 := Vec3{3, 4, 5}
		got := v1.Add(v2)
		want := Vec3{3, 5, 7}
		testCheckEqualV3(t, got, want)
	})

	t.Run("Vec3 add3", func(t *testing.T) {
		v0 := Vec3{0, 1, 2}
		v1 := Vec3{0, 1, 2}
		got := v1.Add3(2, 1, -1)
		want := Vec3{2, 2, 1}
		testCheckEqualV3(t, got, want)
		testCheckEqualV3(t, v0, v1)
	})

	// TODO: AddScaled, Sub, Sub3, Scale, Scale3, Dot, Cross, AddEq, Add3Eq, AddScaledEq, SubEq, Sub3Eq, ScaleEq, Scale3Eq, CrossEq, MulMat3, MulMat4, MulTransform, MulMat3Eq, MulMat4Eq, MulTransformEq, Length, LengthSq, Normalized, Normalize, Negate, NegateEq
}

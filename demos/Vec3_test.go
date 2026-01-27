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

	t.Run("Vec3 addScaled", func(t *testing.T) {
		v0 := Vec3{0, 1, 2}
		v1 := Vec3{1, 2, 3}
		got := v0.AddScaled(v1, 0.5)
		want := Vec3{v0.x + v1.x*0.5, v0.y + v1.y*0.5, v0.z + v1.z*0.5}
		testCheckEqualV3(t, got, want)
		v0.AddScaledEq(v1, 0.5)
		testCheckEqualV3(t, v0, want)
	})

	t.Run("Vec3 addEq", func(t *testing.T) {
		v0 := Vec3{0, 1, 2}
		v1 := Vec3{1, 2, 3}
		v2 := Vec3{0, 1, 2}
		want := Vec3{v0.x + v1.x, v0.y + v1.y, v0.z + v1.z}
		got1 := v0.AddEq(v1)
		testCheckEqualV3(t, *got1, want)
		got2 := v2.Add3Eq(1, 2, 3)
		testCheckEqualV3(t, *got2, want)
	})

	tests := []struct {
		name string
		v, b Vec3
		s    float64
		want Vec3
	}{
		{
			name: "Vec3 AddRhsScaled positive scale",
			v:    Vec3{1, 2, 3},
			b:    Vec3{4, 5, 6},
			s:    2,
			want: Vec3{9, 12, 15},
		},
		{
			name: "Vec3 AddRhsScaled zero scale",
			v:    Vec3{1, 2, 3},
			b:    Vec3{4, 5, 6},
			s:    0,
			want: Vec3{1, 2, 3},
		},
		{
			name: "Vec3 AddRhsScaled negative scale",
			v:    Vec3{1, 2, 3},
			b:    Vec3{1, 1, 1},
			s:    -1,
			want: Vec3{0, 1, 2},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := tt.v.AddRhsScaled(tt.b, tt.s)
			testCheckEqualV3(t, got, tt.want)
		})
	}
}

// Fails if any allocations happen inside given code
func TestVec3_NoAllocs(t *testing.T) {
	v := Vec3{1, 2, 3}
	b := Vec3{4, 5, 6}
	s := 0.5

	allocs := testing.AllocsPerRun(100, func() {
		_ = v.AddRhsScaled(b, s)
	})

	if allocs != 0 {
		t.Fatalf("Expected zero allocations, got %f allocs/op", allocs)
	}
}

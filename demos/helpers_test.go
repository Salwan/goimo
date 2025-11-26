package demos

import (
	"math"
	"reflect"
	"testing"

	"github.com/g3n/engine/math32"
)

const (
	Epsilon32 = float32(1e-6)
	Epsilon64 = float64(1e-9)
)

func float32AlmostEqual(t *testing.T, a, b float32) bool {
	t.Helper()
	return math32.Abs(a-b) <= Epsilon32
}

func float64AlmostEqual(t *testing.T, a, b float64) bool {
	t.Helper()
	return math.Abs(a-b) <= Epsilon64
}

// Determine equality between comparables
func testCheckEqual[T comparable](t *testing.T, want, got T) {
	t.Helper()
	if got != want {
		t.Errorf("want=%v got=%v", want, got)
	}
}

// Determine equality for composites recursively
func testCheckEqualDeep(t *testing.T, want, got any) {
	t.Helper()
	if !reflect.DeepEqual(got, want) {
		t.Errorf("want=%v got=%v", want, got)
	}
}

// Determine equality for slices
func testCheckEqualSlices[T any](t *testing.T, want, got []T) {
	t.Helper()

	// LLM
	// Fast path for identical slice header (including nil vs non-nil)
	if &want == &got {
		return
	}

	// Use length check first for clearer message and quick failure
	if len(want) != len(got) {
		t.Errorf("slice length mismatch: want=%d got=%d\nwant=%#v\ngot =%#v", len(want), len(got), want, got)
	}

	// If elements are comparable, use direct comparison element-wise (faster)
	var zero T
	if reflect.TypeOf(zero) != nil && reflect.TypeOf(zero).Comparable() {
		for i := range want {
			// convert to any for generic equality; safe because Comparable() returned true
			if any(want[i]) != any(got[i]) {
				t.Errorf("slice mismatch at index %d: want=%#v got=%#v\nwant=%#v\ngot =%#v", i, want[i], got[i], want, got)
			}
		}
		return
	}

	// Fallback to deep equality for non-comparable element types
	if !reflect.DeepEqual(want, got) {
		t.Errorf("slices not equal\nwant=%#v\ngot =%#v", want, got)
	}
}

func testCheckEqualV3(t *testing.T, want, got Vec3) {
	t.Helper()
	if !float64AlmostEqual(t, want.x, got.x) || !float64AlmostEqual(t, want.y, got.y) || !float64AlmostEqual(t, want.z, got.z) {
		t.Errorf("Vec3 not equal: want=%v got=%v", want, got)
	}
}

// suppress linter unused
var _ = float32AlmostEqual
var _ = float64AlmostEqual
var _ = testCheckEqual[any]
var _ = testCheckEqualDeep
var _ = testCheckEqualSlices[any]
var _ = testCheckEqualV3

package demos

//////////////////////////////////////////// Array utilities
// from M.array_* in OimoPhysics
// LLM

// double the length of a slice and returns the new slice. Copies over values and sets old to zero value.
// Caution: in original macro M.array_expand, size of array is passed explicitly, why?
func Array_expand[T any](arr []T) []T {
	curLength := len(arr)
	newLength := curLength << 1
	newArray := make([]T, newLength)

	for i := range curLength {
		newArray[i] = arr[i]
		var zero T
		arr[i] = zero

	}

	return newArray
}

// Iterates array num of elements setting them to the zero value
func Array_free[T any](arr []T, num *int) {
	var zero T
	for *num > 0 {
		*num--
		arr[*num] = zero
	}
}

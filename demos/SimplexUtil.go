package demos

/////////////////////////////////////// SimplexUtil
// (oimo/collision/narrowphase/detector/gjkepa/SimplexUtil.go)
// Simplex utilities for GJK/EPA computations.

type SimplexUtilNamespace struct{}

var SimplexUtil = SimplexUtilNamespace{}

// Sets `out` to the minimum length point on the line (`vec1`, `vec2`) and returns the index of the voronoi region.
func (SimplexUtilNamespace) projectOrigin2(v1 Vec3, v2 Vec3, out *Vec3) int {
	var v12 Vec3
	MathUtil.Vec3_sub(&v12, &v1, &v2)

	d := v12.Dot(v12)
	t := v12.Dot(v1)
	t = -t / d

	if t < 0 {
		*out = v1
		return 1
	}
	if t > 1 {
		*out = v2
		return 2
	}

	MathUtil.Vec3_addRhsScaled(out, &v1, &v12, t)
	return 3
}

// Sets `out` to the minimum length point on the triangle (`vec1`, `vec2`, `vec3`) and returns the index of the voronoi region.
func (SimplexUtilNamespace) projectOrigin3(v1 Vec3, v2 Vec3, v3 Vec3, out *Vec3) int {
	v12 := v2.Sub(v1)
	v23 := v3.Sub(v2)
	v31 := v1.Sub(v3)

	n := v12.Cross(v23)

	n12 := v12.Cross(n)
	n23 := v23.Cross(n)
	n31 := v31.Cross(n)
	d12 := v1.Dot(n12)
	d23 := v2.Dot(n23)
	d31 := v3.Dot(n31)

	mind := -1.0
	var minv Vec3
	mini := 0 // index of voroni region

	if d12 < 0 {
		b := SimplexUtil.projectOrigin2(v1, v2, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		mini = b
		mind = d
		minv = *out
	}
	if d23 < 0 {
		b := SimplexUtil.projectOrigin2(v2, v3, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		if mind < 0 || d < mind {
			mini = b << 1 // 00000021 -> 00000210
			mind = d
			minv = *out
		}
	}
	if d31 < 0 {
		b := SimplexUtil.projectOrigin2(v1, v3, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		if mind < 0 || d < mind {
			mini = b&1 | (b&2)<<1 // 00000021 -> 00000201
			mind = d
			minv = *out
		}
	}
	if mind > 0 {
		*out = minv
		return mini
	}

	n.Normalize()
	dn := v1.Dot(n)
	l2 := n.Dot(n)
	l2 = dn / l2
	MathUtil.Vec3_scale(&minv, &n, l2)
	*out = minv
	return 7
}

// Sets `out` to the minimum length point on the tetrahedron (`vec1`, `vec2`, `vec3`, `vec4`) and returns the index of the voronoi region.
func (SimplexUtilNamespace) projectOrigin4(v1 Vec3, v2 Vec3, v3 Vec3, v4 Vec3, out *Vec3) int {
	v12 := v2.Sub(v1)
	v13 := v3.Sub(v1)
	v14 := v4.Sub(v1)
	v23 := v3.Sub(v2)
	v24 := v4.Sub(v2)
	// v34 := v4.Sub(v3) // unused

	n123 := v12.Cross(v13)
	n134 := v13.Cross(v14)
	n142 := v14.Cross(v12)
	n243 := v24.Cross(v23)

	var sign float64
	if v12.Dot(n243) > 0 {
		sign = 1.0
	} else {
		sign = -1.0
	}
	d123 := v1.Dot(n123)
	d134 := v1.Dot(n134)
	d142 := v1.Dot(n142)
	d243 := v2.Dot(n243)

	mind := -1.0
	var minv Vec3
	mini := 0 // index of voronoi region

	if d123*sign < 0 {
		b := SimplexUtil.projectOrigin3(v1, v2, v3, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		mini = b
		mind = d
		minv = *out
	}
	if d134*sign < 0 {
		b := SimplexUtil.projectOrigin3(v1, v3, v4, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		if mind < 0 || d < mind {
			mini = b&1 | (b&6)<<1 // 00000321 -> 00003201
			mind = d
			minv = *out
		}
	}
	if d142*sign < 0 {
		b := SimplexUtil.projectOrigin3(v1, v2, v4, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		if mind < 0 || d < mind {
			mini = b&3 | (b&4)<<1 // 00000321 -> 00003021
			mind = d
			minv = *out
		}
	}
	if d243*sign < 0 {
		b := SimplexUtil.projectOrigin3(v2, v3, v4, out)
		d := out.x*out.x + out.y*out.y + out.z*out.z
		if mind < 0 || d < mind {
			mini = b << 1 // 00000321 -> 00003210
			mind = d
			minv = *out
		}
	}

	if mind > 0 {
		*out = minv
		return mini
	}

	// the origin is inside the tetrahedron
	out.Zero()
	return 15
}

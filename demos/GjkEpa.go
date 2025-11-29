package demos

import "github.com/Salwan/goimo/debug"

//////////////////////////////////////////// GjkEpa
// (oimo/collision/narrowphase/detector/gjkepa/GjkEpa.go)
// GJK algorithm and EPA for narrow-phase collision detection.

var GjkEpaInstance *GjkEpa = _newGjkEpa()

type GjkEpa struct {
	c1  *ConvexGeometry
	c2  *ConvexGeometry
	tf1 *Transform
	tf2 *Transform

	// for GJK

	// simplex
	s           []Vec3
	simplexSize int

	// witness points
	w1 []Vec3
	w2 []Vec3

	tempVec3s     []Vec3
	tempTransform Transform

	dir      Vec3   // direction
	closest  Vec3   // closest point
	baseDirs []Vec3 // base directions used to expand simplex

	// for convex casting
	tl1  Vec3
	tl2  Vec3
	rayX Vec3
	rayR Vec3

	// for EPA

	depth      float64
	polyhedron EpaPolyhedron

	// public vars

	ClosestPoint1 Vec3    // Computed closest point of the first geometry in world coordinate system.
	ClosestPoint2 Vec3    // Computed closest point of the second geometry in world coordinate system.
	Distance      float64 // Computed distance between two geometries. This value may be negative if two geometries are overlapping.
}

// Should not be called directly as GjkEpa is a singleton, access with: GjkEpaInstance
func _newGjkEpa() *GjkEpa {
	return &GjkEpa{
		s:  make([]Vec3, 4),
		w1: make([]Vec3, 4),
		w2: make([]Vec3, 4),

		baseDirs: []Vec3{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
}

func (ge *GjkEpa) convexCastImpl(c1, c2 *ConvexGeometry, tf1, tf2 *Transform, tl1, tl2 *Vec3, hit *RayCastHit) bool {
	debug.GjkLog("----------- GJK convex casting begin -----------")

	ge.c1 = c1
	ge.c2 = c2
	ge.tf1 = tf1
	ge.tf2 = tf2

	// simplex: ge.s

	// witness points // unused
	// w1 := &ge.w1
	// w2 := &ge.w2

	closest := &ge.closest

	dir := &ge.dir

	// Compute first vertex of the simplex
	MathUtil.Vec3_sub(dir, &tf2.position, &tf1.position)
	if dir.LengthSq() < 1e-6 {
		dir.Set(1, 0, 0)
	}

	ge.simplexSize = 0
	ge.computeConvexCastSupportingVertex()
	ge.simplexSize = 1

	// loop count and max iteration of the loop
	count := 0
	max := 40

	lambda := 0.0
	rayX := &ge.rayX // origin
	rayR := &ge.rayR // relative translation

	rayX.Zero()
	MathUtil.Vec3_sub(rayR, tl2, tl1)

	eps := 1e-4
	eps2 := eps * eps

	for count < max {
		// project the origin to the simplex and compute index of voronoi region of the origin.
		v := 0
		debug.GjkLog("simplex size: %v", ge.simplexSize)
		debug.GjkLog("projecting the origin to the simplex...")
		debug.GjkLog("x: %v", rayX)
		debug.GjkLog("lambda: %v", lambda)

		switch ge.simplexSize {
		case 1:
			*closest = ge.s[0]
			debug.GjkLog("%v", ge.s[0])
			v = 1
		case 2:
			v = SimplexUtil.projectOrigin2(ge.s[0], ge.s[1], closest)
			debug.GjkLog("%v", ge.s[0])
			debug.GjkLog("%v", ge.s[1])
		case 3:
			v = SimplexUtil.projectOrigin3(ge.s[0], ge.s[1], ge.s[2], closest)
			debug.GjkLog("%v", ge.s[0])
			debug.GjkLog("%v", ge.s[1])
			debug.GjkLog("%v", ge.s[2])
		case 4:
			v = SimplexUtil.projectOrigin4(ge.s[0], ge.s[1], ge.s[2], ge.s[3], closest)
			debug.GjkLog("%v", ge.s[0])
			debug.GjkLog("%v", ge.s[1])
			debug.GjkLog("%v", ge.s[2])
			debug.GjkLog("%v", ge.s[3])
		}

		debug.GjkLog("projected origin: pos = %v, voronoi index = %v", closest, v)

		// shrink the simplex according to the voronoi index of the origin
		ge.shrinkSimplex(v)

		// check if the origin is touching or inside the simplex
		if closest.LengthSq() < eps2 {
			if lambda == 0 || ge.simplexSize == 4 {
				debug.GjkLog("overlapping ... closest: %v", closest)
				hit.Fraction = lambda
				return false // overlapping
			}

			ge.interpolateClosestPoints()

			hit.Fraction = lambda
			hit.Normal = dir.Normalized()
			hit.Position = ge.ClosestPoint1
			hit.Position.AddScaledEq(*tl1, lambda)
			debug.GjkLog("GJK convex cast succeeded")
			return true
		}

		// compute the next vertex
		*dir = *closest
		dir.NegateEq()
		ge.computeConvexCastSupportingVertex()
		ge.s[ge.simplexSize].SubEq(*rayX) // translate origin

		if dir.LengthSq() < eps2 {
			panic("should never happen")
		}

		// n is the normal at the vertex p
		p := ge.s[ge.simplexSize]
		n := *dir
		debug.GjkLog("new vertex p = %v", p)
		debug.GjkLog("normal n = %v", n)
		debug.GjkLog("ray dir r = %v", rayR)

		// check if a part of the ray can be rejected
		pn := p.Dot(n)
		debug.GjkLog("p dot n = %v", pn)
		if pn < 0 {
			// check if entire the ray can be rejected
			if rayR.Dot(n) >= 0 {
				debug.GjkLog("rejected [0")
				return false
			}
			dLambda := pn / rayR.Dot(n)
			lambda += dLambda
			if lambda >= 1 {
				debug.GjkLog("rejected 1]")
				return false
			}
			debug.GjkLog("advanced: %v", dLambda)
			rayX.AddScaledEq(*rayR, dLambda)

			// translate the simplex
			for i := range ge.simplexSize + 1 {
				ge.s[i].AddScaledEq(*rayR, -dLambda)
			}
		} else {
			debug.GjkLog("ray did not advance")
		}

		// do not add new vertex to the simplex if already exists
		duplicate := false
		for i := range ge.simplexSize {
			dx := ge.s[i].x - ge.s[ge.simplexSize].x
			dy := ge.s[i].y - ge.s[ge.simplexSize].y
			dz := ge.s[i].z - ge.s[ge.simplexSize].z
			if dx*dx+dy*dy+dz*dz < eps2 {
				duplicate = true
				debug.GjkLog("duplicate vertex %v and %v", ge.s[i], ge.s[ge.simplexSize])
				break
			}
		}
		if !duplicate {
			debug.GjkLog("added %v", ge.s[ge.simplexSize])
			ge.simplexSize++
		}

		count++

		debug.GjkLog("iteration: %d", count)
	}

	debug.GjkLog("GJK convex cast failed: did not converge")
	return false
}

func (ge *GjkEpa) interpolateClosestPoints() {
	switch ge.simplexSize {
	case 1:
		ge.ClosestPoint1 = ge.w1[0]
		ge.ClosestPoint2 = ge.w2[0]
	case 2:
		c := ge.closest
		s0 := ge.s[0]
		w10 := ge.w1[0]
		w20 := ge.w2[0]
		s1 := ge.s[1]
		w11 := ge.w1[1]
		w21 := ge.w2[1]
		// s2 := ge.s[2]; w12 := ge.w1[2]; w22 := ge.w2[2] // unused

		s01 := s1.Sub(s0)
		invDet := s01.Dot(s01)
		if invDet != 0 {
			invDet = 1.0 / invDet
		}
		s0c := c.Sub(s0)
		t := s0c.Dot(s01) * invDet

		// compute closest points
		diff := w11.Sub(w10)
		var cp1 Vec3
		MathUtil.Vec3_addRhsScaled(&cp1, &w10, &diff, t)

		diff = w21.Sub(w20)
		var cp2 Vec3
		MathUtil.Vec3_addRhsScaled(&cp2, &w20, &diff, t)

		ge.ClosestPoint1 = cp1
		ge.ClosestPoint2 = cp2
	case 3:
		c := ge.closest
		s0 := ge.s[0]
		w10 := ge.w1[0]
		w20 := ge.w2[0]
		s1 := ge.s[1]
		w11 := ge.w1[1]
		w21 := ge.w2[1]
		s2 := ge.s[2]
		w12 := ge.w1[2]
		w22 := ge.w2[2]

		s01 := s1.Sub(s0)
		s02 := s2.Sub(s0)
		s0c := c.Sub(s0)

		d11 := s01.Dot(s01)
		d12 := s01.Dot(s02)
		d22 := s02.Dot(s02)
		d1c := s01.Dot(s0c)
		d2c := s02.Dot(s0c)
		invDet := d11*d22 - d12*d12
		if invDet != 0 {
			invDet = 1.0 / invDet
		}
		s := (d1c*d22 - d2c*d12) * invDet
		t := (-d1c*d12 + d2c*d11) * invDet

		// compute closest points
		var cp1 Vec3
		var cp2 Vec3

		diff := w11.Sub(w10)
		MathUtil.Vec3_addRhsScaled(&cp1, &w10, &diff, s)
		diff = w12.Sub(w10)
		MathUtil.Vec3_addRhsScaled(&cp1, &cp1, &diff, t)

		diff = w21.Sub(w20)
		MathUtil.Vec3_addRhsScaled(&cp2, &w20, &diff, s)
		diff = w22.Sub(w20)
		MathUtil.Vec3_addRhsScaled(&cp2, &cp2, &diff, t)

		ge.ClosestPoint1 = cp1
		ge.ClosestPoint2 = cp2
	default:
		panic("should never happen")
	}
}

func (ge *GjkEpa) shrinkSimplex(vertexBits int) {
	ge.simplexSize = vertexBits
	ge.simplexSize = (ge.simplexSize & 5) + (ge.simplexSize >> 1 & 5)
	ge.simplexSize = (ge.simplexSize & 3) + (ge.simplexSize >> 2 & 3)

	switch vertexBits {
	// 0, 1, 3, 7, 15: do nothing
	case 2: // 0 <- 1
		ge.s[0] = ge.s[1]
		ge.w1[0] = ge.w1[1]
		ge.w2[0] = ge.w2[1]
	case 4, 6: // 0 <- 2
		ge.s[0] = ge.s[2]
		ge.w1[0] = ge.w1[2]
		ge.w2[0] = ge.w2[2]
	case 5: // 1 <- 2
		ge.s[1] = ge.s[2]
		ge.w1[1] = ge.w1[2]
		ge.w2[1] = ge.w2[2]
	case 8, 10, 14: // 0 <- 3
		ge.s[0] = ge.s[3]
		ge.w1[0] = ge.w1[3]
		ge.w2[0] = ge.w2[3]
	case 9, 13: // 1 <- 3
		ge.s[1] = ge.s[3]
		ge.w1[1] = ge.w1[3]
		ge.w2[1] = ge.w2[3]
	case 11: // 2 <- 3
		ge.s[2] = ge.s[3]
		ge.w1[2] = ge.w1[3]
		ge.w2[2] = ge.w2[3]
	case 12: // 0 <- 2, 1 <- 3
		ge.s[0] = ge.s[2]
		ge.w1[0] = ge.w1[2]
		ge.w2[0] = ge.w2[2]
		ge.s[1] = ge.s[3]
		ge.w1[1] = ge.w1[3]
		ge.w2[1] = ge.w2[3]
	}
}

func (ge *GjkEpa) computeConvexCastSupportingVertex() {
	if ge.c1 != nil {
		ge.computeWitnessPoint1(true)
	} else {
		ge.w1[ge.simplexSize] = ge.tf1.position
	}
	ge.computeWitnessPoint2(true)
	MathUtil.Vec3_sub(&ge.s[ge.simplexSize], &ge.w1[ge.simplexSize], &ge.w2[ge.simplexSize])
}

func (ge *GjkEpa) computeWitnessPoint1(addMargin bool) {
	idir := ge.dir

	// compute local dir
	MathUtil.Vec3_mulMat3Transposed(&ge.dir, &ge.dir, &ge.tf1.rotation)

	// compute local witness point
	ge.c1.ComputeLocalSupportingVertex(ge.dir, &ge.w1[ge.simplexSize])
	if addMargin {
		ge.dir.Normalize()
		ge.w1[ge.simplexSize].AddScaledEq(ge.dir, ge.c1.gjkMargin)
	}

	// compute world witness point
	var iw1 Vec3
	tmp := &ge.w1[ge.simplexSize]
	MathUtil.Vec3_mulMat3(&iw1, tmp, &ge.tf1.rotation)
	MathUtil.Vec3_add(&iw1, &iw1, &ge.tf1.position)
	ge.w1[ge.simplexSize] = iw1

	ge.dir = idir
}

func (ge *GjkEpa) computeWitnessPoint2(addMargin bool) {
	idir := ge.dir

	// compute local dir
	MathUtil.Vec3_mulMat3Transposed(&ge.dir, &ge.dir, &ge.tf2.rotation)
	ge.dir.NegateEq()

	// compute local witness point
	ge.c2.ComputeLocalSupportingVertex(ge.dir, &ge.w2[ge.simplexSize])
	if addMargin {
		ge.dir.Normalize()
		ge.w2[ge.simplexSize].AddScaledEq(ge.dir, ge.c2.gjkMargin)
	}

	// compute world witness point
	var iw2 Vec3
	tmp := &ge.w2[ge.simplexSize]
	MathUtil.Vec3_mulMat3(&iw2, tmp, &ge.tf2.rotation)
	MathUtil.Vec3_add(&iw2, &iw2, &ge.tf2.position)
	ge.w2[ge.simplexSize] = iw2

	ge.dir = idir
}

// Performs ray casting against the convex geometry `c` with transform `tf`. Returns `true` and sets the result information to `hit` if the line segment from `begin` to `end` intersects the convex geometry. Otherwise returns `false`.
// Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
func (ge *GjkEpa) RayCast(c *ConvexGeometry, tf *Transform, begin, end Vec3, hit *RayCastHit) bool {
	ge.tf1.position = begin

	// This temp ref looks like no-op to me, but I'm copying as is
	tl1 := &ge.tl1
	tl2 := &ge.tl2

	MathUtil.Vec3_sub(tl1, &end, &begin)
	tl2.Zero()

	return ge.convexCastImpl(nil, c, &ge.tempTransform, tf, tl1, tl2, hit)
}

// TODO

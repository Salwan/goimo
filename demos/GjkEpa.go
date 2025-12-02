package demos

import "github.com/Salwan/goimo/debug"

//////////////////////////////////////////// GjkEpa
// (oimo/collision/narrowphase/detector/gjkepa/GjkEpa.go)
// GJK algorithm and EPA for narrow-phase collision detection.

var GjkEpaInstance *GjkEpa = _newGjkEpa()

type GjkEpa struct {
	c1  IConvexGeometry
	c2  IConvexGeometry
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
	polyhedron *EpaPolyhedron

	// public vars

	ClosestPoint1 Vec3    // Computed closest point of the first geometry in world coordinate system.
	ClosestPoint2 Vec3    // Computed closest point of the second geometry in world coordinate system.
	Distance      float64 // Computed distance between two geometries. This value may be negative if two geometries are overlapping.
}

// Should not be called directly as GjkEpa is a singleton, access with: GjkEpaInstance
func _newGjkEpa() *GjkEpa {
	g := &GjkEpa{
		s:  make([]Vec3, 4),
		w1: make([]Vec3, 4),
		w2: make([]Vec3, 4),

		baseDirs: []Vec3{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		polyhedron: NewEpaPolyhedron(),
	}
	g.tempTransform.Identity()
	return g
}

// --- private ---

func (self *GjkEpa) _computeClosestPointsImpl(c1, c2 IConvexGeometry, tf1, tf2 *Transform, cache *CachedDetectorData, useEpa bool) GjkEpaResultState {
	debug.GjkLog("----------- GJK begin -----------")

	self.c1 = c1
	self.c2 = c2
	self.tf1 = tf1
	self.tf2 = tf2

	if cache != nil {
		if cache.gjkCache == nil {
			cache.gjkCache = NewGjkCache()
		}
		self._loadCache(cache.gjkCache)
	} else {
		self.dir.Zero()
	}

	if self.dir.LengthSq() == 0 {
		// compute the first vertex of the simplex
		self.dir = tf2.position.Sub(tf1.position)
		if self.dir.LengthSq() < 1e-6 {
			self.dir.Set(1, 0, 0)
		}
	}

	self.simplexSize = 0
	self._computeSupportingVertex()
	self.simplexSize = 1

	// loop count and max iteration of the loop
	count := 0
	max := 40

	eps := 1e-4
	eps2 := eps * eps

	for count < max {
		// project the origin to the simplex and compute index of voronoi region of the origin.
		v := 0
		debug.GjkLog("simplex size: %v", self.simplexSize)
		debug.GjkLog("projecting the origin to the simplex...")
		switch self.simplexSize {
		case 1:
			self.closest.CopyFrom(self.s[0])
			debug.GjkLog("%v", self.s[0])
			v = 1
		case 2:
			v = SimplexUtil.projectOrigin2(self.s[0], self.s[1], &self.closest)
			debug.GjkLog("%v", self.s[0])
			debug.GjkLog("%v", self.s[1])
		case 3:
			v = SimplexUtil.projectOrigin3(self.s[0], self.s[1], self.s[2], &self.closest)
			debug.GjkLog("%v", self.s[0])
			debug.GjkLog("%v", self.s[1])
			debug.GjkLog("%v", self.s[2])
		case 4:
			v = SimplexUtil.projectOrigin4(self.s[0], self.s[1], self.s[2], self.s[3], &self.closest)
			debug.GjkLog("%v", self.s[0])
			debug.GjkLog("%v", self.s[1])
			debug.GjkLog("%v", self.s[2])
			debug.GjkLog("%v", self.s[3])
		}

		// check if the origin is touching or inside the simplex
		if self.closest.LengthSq() < eps2 {
			if !useEpa {
				self.Distance = 0
				return GjkEpaResultState_SUCCEEDED
			}

			// make the simplex to be a tetrahedron for EPA computation
			switch self.simplexSize {
			case 1:
				self._pointToTetrahedron()
				debug.GjkLog("point -> tetrahedron")
			case 2:
				self._lineToTetrahedron()
				debug.GjkLog("line -> tetrahedron")
			case 3:
				self._triangleToTetrahedron()
				debug.GjkLog("triangle -> tetrahedron")
			}
			if self.simplexSize == 4 {
				epaState := self._computeDepth(c1, c2, tf1, tf2, &self.s, &self.w1, &self.w2)
				if epaState != GjkEpaResultState_SUCCEEDED {
					self.Distance = 0
					return epaState
				}
				self.Distance = -self.depth
				return GjkEpaResultState_SUCCEEDED
			}

			// failed to make a tetrahedron
			self.Distance = 0
			return GjkEpaResultState_GJK_FAILED_TO_MAKE_TETRAHEDRON
		}

		debug.GjkLog("projected origin: %v", v)

		// shrink the simplex according to the voronoi index of the origin
		self._shrinkSimplex(v)

		// compute the next vertex
		self.dir.CopyFrom(self.closest).NegateEq()
		self._computeSupportingVertex()

		if self.dir.LengthSq() < eps2 {
			panic("!?") // this should never be happen
		}

		d1 := self.closest.Dot(self.dir)
		d2 := self.s[self.simplexSize].Dot(self.dir)

		debug.GjkLog("n: %v, prev: %v, current: %v, dir: %v, iteration: %v, d2 - d1: %v", self.simplexSize, self.closest, self.s[self.simplexSize], self.dir, count, d2-d1)

		if d2-d1 < eps2 { // terminate GJK; no improvement
			self._interpolateClosestPoints()

			debug.GjkLog("iteration: %d", count)

			self.Distance = self.closest.Length() // no improvement

			if cache != nil && cache.gjkCache != nil {
				self._saveCache(cache.gjkCache)
			}

			return GjkEpaResultState_SUCCEEDED
		}

		self.simplexSize++
		count++
	}

	debug.GjkLog("GJK failed: did not converge")
	return GjkEpaResultState_GJK_DID_NOT_CONVERGE
}

// `c1` can be nil
func (ge *GjkEpa) _convexCastImpl(c1, c2 IConvexGeometry, tf1, tf2 *Transform, tl1, tl2 Vec3, hit *RayCastHit) bool {
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
	ge._computeConvexCastSupportingVertex()
	ge.simplexSize = 1

	// loop count and max iteration of the loop
	count := 0
	max := 40

	lambda := 0.0
	rayX := &ge.rayX // origin
	rayR := &ge.rayR // relative translation

	rayX.Zero()
	MathUtil.Vec3_sub(rayR, &tl2, &tl1)

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
		ge._shrinkSimplex(v)

		// check if the origin is touching or inside the simplex
		if closest.LengthSq() < eps2 {
			if lambda == 0 || ge.simplexSize == 4 {
				debug.GjkLog("overlapping ... closest: %v", closest)
				hit.Fraction = lambda
				return false // overlapping
			}

			ge._interpolateClosestPoints()

			hit.Fraction = lambda
			hit.Normal = dir.Normalized()
			hit.Position = ge.ClosestPoint1
			hit.Position.AddScaledEq(tl1, lambda)
			debug.GjkLog("GJK convex cast succeeded")
			return true
		}

		// compute the next vertex
		*dir = *closest
		dir.NegateEq()
		ge._computeConvexCastSupportingVertex()
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

func (ge *GjkEpa) _interpolateClosestPoints() {
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

func (self *GjkEpa) _loadCache(gjkCache *GjkCache) {
	// copy simplex data from the cache
	self.dir.CopyFrom(gjkCache.prevClosestDir)
}

func (self *GjkEpa) _saveCache(gjkCache *GjkCache) {
	// set GJK cache for the next computation
	gjkCache.prevClosestDir.CopyFrom(self.closest).NegateEq()
}

func (ge *GjkEpa) _shrinkSimplex(vertexBits int) {
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

func (self *GjkEpa) _computeSupportingVertex() {
	self._computeWitnessPoint1(false)
	self._computeWitnessPoint2(false)
	self.s[self.simplexSize].CopyFrom(self.w1[self.simplexSize]).SubEq(self.w2[self.simplexSize])
}

func (ge *GjkEpa) _computeConvexCastSupportingVertex() {
	if ge.c1 != nil {
		ge._computeWitnessPoint1(true)
	} else {
		ge.w1[ge.simplexSize] = ge.tf1.position
	}
	ge._computeWitnessPoint2(true)
	MathUtil.Vec3_sub(&ge.s[ge.simplexSize], &ge.w1[ge.simplexSize], &ge.w2[ge.simplexSize])
}

func (ge *GjkEpa) _computeWitnessPoint1(addMargin bool) {
	idir := ge.dir

	// compute local dir
	ge.dir = ge.dir.MulMat3Transposed(&ge.tf1.rotation)

	// compute local witness point
	ge.c1.ComputeLocalSupportingVertex(ge.dir, &ge.w1[ge.simplexSize])
	if addMargin {
		ge.dir.Normalize()
		ge.w1[ge.simplexSize].AddScaledEq(ge.dir, ge.c1.GetGjkMargin())
	}

	// compute world witness point
	var iw1 Vec3
	tmp := &ge.w1[ge.simplexSize]
	MathUtil.Vec3_mulMat3(&iw1, tmp, &ge.tf1.rotation)
	MathUtil.Vec3_add(&iw1, &iw1, &ge.tf1.position)
	ge.w1[ge.simplexSize] = iw1

	ge.dir = idir
}

func (ge *GjkEpa) _computeWitnessPoint2(addMargin bool) {
	idir := ge.dir

	// compute local dir
	ge.dir = ge.dir.MulMat3Transposed(&ge.tf2.rotation)
	ge.dir.NegateEq()

	// compute local witness point
	ge.c2.ComputeLocalSupportingVertex(ge.dir, &ge.w2[ge.simplexSize])
	if addMargin {
		ge.dir.Normalize()
		ge.w2[ge.simplexSize].AddScaledEq(ge.dir, ge.c2.GetGjkMargin())
	}

	// compute world witness point
	var iw2 Vec3
	tmp := &ge.w2[ge.simplexSize]
	MathUtil.Vec3_mulMat3(&iw2, tmp, &ge.tf2.rotation)
	MathUtil.Vec3_add(&iw2, &iw2, &ge.tf2.position)
	ge.w2[ge.simplexSize] = iw2

	ge.dir = idir
}

func (self *GjkEpa) _pointToTetrahedron() {
	for i := range 3 {
		self.dir.CopyFrom(self.baseDirs[i])

		self._computeSupportingVertex()
		self.simplexSize++
		self._lineToTetrahedron()
		if self.simplexSize == 4 {
			break
		}
		self.simplexSize--

		self.dir.NegateEq()

		self._computeSupportingVertex()
		self.simplexSize++
		self._lineToTetrahedron()
		if self.simplexSize == 4 {
			break
		}
		self.simplexSize--
	}
}

func (self *GjkEpa) _lineToTetrahedron() {
	oldDir := self.dir

	s0 := self.s[0]
	s1 := self.s[1]
	lineDir := s0.Sub(s1)
	for i := range 3 {
		baseDir := self.baseDirs[i]
		newDir := lineDir.Cross(baseDir)
		self.dir = newDir

		self._computeSupportingVertex()
		self.simplexSize++
		self._triangleToTetrahedron()
		if self.simplexSize == 4 {
			break
		}
		self.simplexSize--

		self.dir.NegateEq()

		self._computeSupportingVertex()
		self.simplexSize++
		self._triangleToTetrahedron()
		if self.simplexSize == 4 {
			break
		}
		self.simplexSize--
	}

	self.dir = oldDir
}

func (self *GjkEpa) _triangleToTetrahedron() {
	oldDir := self.dir
	for range 1 {
		s0 := self.s[0]
		s1 := self.s[1]
		s2 := self.s[2]
		s01 := s1.Sub(s0)
		s02 := s2.Sub(s0)
		n := s01.Cross(s02)
		self.dir = n

		self._computeSupportingVertex()
		self.simplexSize++
		if self._isValidTetrahedron() {
			break
		}
		self.simplexSize--

		self.dir.NegateEq()

		self._computeSupportingVertex()
		self.simplexSize++

		if self._isValidTetrahedron() {
			break
		}
		self.simplexSize--
	}
	self.dir = oldDir
}

func (self *GjkEpa) _isValidTetrahedron() bool {
	e00 := self.s[1].x - self.s[0].x
	e01 := self.s[1].y - self.s[0].y
	e02 := self.s[1].z - self.s[0].z
	e10 := self.s[2].x - self.s[0].x
	e11 := self.s[2].y - self.s[0].y
	e12 := self.s[2].z - self.s[0].z
	e20 := self.s[3].x - self.s[0].x
	e21 := self.s[3].y - self.s[0].y
	e22 := self.s[3].z - self.s[0].z
	det := e00*(e11*e22-e12*e21) - e01*(e10*e22-e12*e20) + e02*(e10*e21-e11*e20)
	return det > 1e-12 || det < -1e-12
}

// EPA

func (self *GjkEpa) _computeDepth(convex1, convex2 IConvexGeometry, tf1, tf2 *Transform, initialPolyhedron, initialPolyhedron1, initialPolyhedron2 *[]Vec3) GjkEpaResultState {
	debug.GjkLog("----------- EPA begin ----------- ")

	self.polyhedron.clear()
	if !self.polyhedron.init(
		self.polyhedron.pickVertex().Set((*initialPolyhedron)[0], (*initialPolyhedron1)[0], (*initialPolyhedron2)[0]),
		self.polyhedron.pickVertex().Set((*initialPolyhedron)[1], (*initialPolyhedron1)[1], (*initialPolyhedron2)[1]),
		self.polyhedron.pickVertex().Set((*initialPolyhedron)[2], (*initialPolyhedron1)[2], (*initialPolyhedron2)[2]),
		self.polyhedron.pickVertex().Set((*initialPolyhedron)[3], (*initialPolyhedron1)[3], (*initialPolyhedron2)[3])) {
		debug.GjkLog("EPA failed at initialization: %v", self.polyhedron.status)
		return GjkEpaResultState_EPA_FAILED_TO_INIT
	}

	self.simplexSize = 0
	supportingVertex := &self.s[0]
	witness1 := &self.w1[0]
	witness2 := &self.w2[0]
	count := 0
	maxIterations := 40

	for count < maxIterations {
		face := self.polyhedron.getBestTriangle()

		if debug.Debug {
			debug.GjkLog("nearest face:")
			face.dump()
		}

		self.dir.CopyFrom(face.normal).Normalize()
		self._computeSupportingVertex()

		v0 := face.vertices[0]
		v1 := face.vertices[1]
		v2 := face.vertices[2]

		dot1 := v0.v.Dot(self.dir)
		dot2 := supportingVertex.Dot(self.dir)

		debug.GjkLog("got new vertex: %v", supportingVertex)
		debug.GjkLog("improvement: %v -> %v, normal: %v", dot1, dot2, self.dir)

		if dot2-dot1 < 1e-6 || count == maxIterations-1 { // no improvement
			self.closest.CopyFrom(self.dir).ScaleEq(self.dir.Dot(v0.v) / self.dir.LengthSq())

			c := self.closest
			s0 := v0.v
			s1 := v1.v
			s2 := v2.v
			w10 := v0.w1
			w11 := v1.w1
			w12 := v2.w1
			w20 := v0.w2
			w21 := v1.w2
			w22 := v2.w2

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
			diff := w11.Sub(w10)
			cp1 := w10.AddRhsScaled(diff, s)
			diff = w12.Sub(w10)
			cp1 = cp1.AddRhsScaled(diff, t)

			diff = w21.Sub(w20)
			cp2 := w20.AddRhsScaled(diff, s)
			diff = w22.Sub(w20)
			cp2 = cp2.AddRhsScaled(diff, t)

			self.ClosestPoint1 = cp1
			self.ClosestPoint2 = cp2

			self.depth = self.closest.Length()

			return GjkEpaResultState_SUCCEEDED
		}
		epaVertex := self.polyhedron.pickVertex().Set(*supportingVertex, *witness1, *witness2)
		if !self.polyhedron.addVertex(epaVertex, face) {

			if debug.Debug {
				debug.GjkLog("EPA failed at vertex addition: %v", self.polyhedron.status)
				self.polyhedron.dumpAsObjModel()
			}

			return GjkEpaResultState_EPA_FAILED_TO_ADD_VERTEX
		}
		count++
	}

	if debug.Debug {
		debug.GjkLog("EPA failed: did not converge.")
		self.polyhedron.dumpAsObjModel()
	}

	return GjkEpaResultState_EPA_DID_NOT_CONVERGE
}

// --- public ---

// Computes the closest points of two convex geometries `c1` and `c2` with transforms `tf1` and `tf2` respectively, and returns the status of the result (see `GjkEpaResultState` for details). If cached data `cache` is not `null`, this tries to exploit the previous result in `cache` to improve performance, and stores the new result to `cache`.
// Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
func (self *GjkEpa) ComputeClosestPoints(c1, c2 IConvexGeometry, tf1, tf2 *Transform, cache *CachedDetectorData) GjkEpaResultState {
	return self._computeClosestPointsImpl(c1, c2, tf1, tf2, cache, true)
}

// Computes the distance between two convex geometries `c1` and `c2` with transforms `tf1` and `tf2` respectively, and returns the status of the result (see `GjkEpaResultState` for details). Different from `GjkEpa.computeClosestPoints`, this does not compute negative distances and closest points if two geometries are overlapping. If cached data `cache` is not `null`, this tries to exploit the previous result in `cache` to improve performance, and stores the new result to `cache`.
// Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
func (self *GjkEpa) ComputeDistance(c1, c2 IConvexGeometry, tf1, tf2 *Transform, cache *CachedDetectorData) GjkEpaResultState {
	return self._computeClosestPointsImpl(c1, c2, tf1, tf2, cache, false)
}

// Performs a convex casting between `c1` and `c2`. Returns `true` and sets the result information to `hit` if the convex geometries intersect. Each convex geometries translates by `tl1` and `tl2`, starting from the beginning transforms `tf1` and `tf2` respectively.
// Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
func (self *GjkEpa) ConvexCast(c1, c2 IConvexGeometry, tf1, tf2 *Transform, tl1, tl2 Vec3, hit *RayCastHit) bool {
	return self._convexCastImpl(c1, c2, tf1, tf2, tl1, tl2, hit)
}

// Performs ray casting against the convex geometry `c` with transform `tf`. Returns `true` and sets the result information to `hit` if the line segment from `begin` to `end` intersects the convex geometry. Otherwise returns `false`.
// Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
func (self *GjkEpa) RayCast(c IConvexGeometry, tf *Transform, begin, end Vec3, hit *RayCastHit) bool {
	tf1 := &self.tempTransform
	tf2 := tf

	tf1.position = begin

	tl1 := &self.tl1
	tl2 := &self.tl2

	tl1.CopyFrom(end).SubEq(begin)
	tl2.Zero()

	return self._convexCastImpl(nil, c, tf1, tf2, *tl1, *tl2, hit)
}

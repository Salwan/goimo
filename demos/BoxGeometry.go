package demos

import "math"

//////////////////////////////////////////////// BoxGeometry
// (oimo/path)
// A box collision geometry.

type BoxGeometry struct {
	*ConvexGeometry

	halfExtents Vec3
	halfAxisX   Vec3
	halfAxisY   Vec3
	halfAxisZ   Vec3
}

// Creates a box collision geometry of half-extents
func NewBoxGeometry(halfExtents Vec3) *BoxGeometry {
	b := &BoxGeometry{
		ConvexGeometry: NewConvexGeometry(_BOX),
		halfExtents:    halfExtents,
		halfAxisX:      Vec3{halfExtents.x, 0, 0},
		halfAxisY:      Vec3{0, halfExtents.y, 0},
		halfAxisZ:      Vec3{0, 0, halfExtents.z},
	}
	b.UpdateMass()

	minHalfExtents := math.Min(math.Min(halfExtents.x, halfExtents.y), halfExtents.z)
	if b.gjkMargin > minHalfExtents*0.2 {
		b.gjkMargin = minHalfExtents * 0.2
	}

	return b
}

func (b *BoxGeometry) getHalfExtentsTo(halfExtents *Vec3) {
	*halfExtents = b.halfExtents
}

func (b *BoxGeometry) UpdateMass() {
	b.volume = 8 * b.halfExtents.x * b.halfExtents.y * b.halfExtents.z
	sq := b.halfExtents.CompWiseMul(b.halfExtents)
	MathUtil.Mat3_diagonal(&b.inertiaCoeff,
		1.0/3.0*(sq.y+sq.z),
		1.0/3.0*(sq.z+sq.x),
		1.0/3.0*(sq.x+sq.y))
}

func (b *BoxGeometry) ComputeAabb(aabb *Aabb, tf *Transform) {
	tfx := b.halfAxisX.MulMat3(&tf.rotation)
	tfy := b.halfAxisY.MulMat3(&tf.rotation)
	tfz := b.halfAxisZ.MulMat3(&tf.rotation)

	MathUtil.Vec3_abs(&tfx, &tfx)
	MathUtil.Vec3_abs(&tfy, &tfy)
	MathUtil.Vec3_abs(&tfz, &tfz)

	tfs := tfx.Add(tfy)
	tfs = tfs.Add(tfz)

	aabb.Min = tf.position.Sub(tfs)
	aabb.Max = tf.position.Add(tfs)
}

func (b *BoxGeometry) ComputeLocalSupportingVertex(dir Vec3, out *Vec3) {
	gjkMargins := Vec3{b.gjkMargin, b.gjkMargin, b.gjkMargin}
	// avoid making core extents negative
	MathUtil.Vec3_min(&gjkMargins, &gjkMargins, &b.halfExtents)
	coreExtents := b.halfExtents.Sub(gjkMargins)
	if dir.x > 0 {
		out.x = coreExtents.x
	} else {
		out.x = -coreExtents.x
	}
	if dir.y > 0 {
		out.y = coreExtents.y
	} else {
		out.y = -coreExtents.y
	}
	if dir.z > 0 {
		out.z = coreExtents.z
	} else {
		out.z = -coreExtents.z
	}
}

func (b *BoxGeometry) RayCastLocal(begin, end Vec3, hit *RayCastHit) bool {
	p1x, p1y, p1z := begin.x, begin.y, begin.z
	p2x, p2y, p2z := end.x, end.y, end.z
	halfW, halfH, halfD := b.halfExtents.x, b.halfExtents.y, b.halfExtents.z
	dx, dy, dz := p2x-p1x, p2y-p1y, p2z-p1z
	tminx, tminy, tminz := 0.0, 0.0, 0.0
	tmaxx, tmaxy, tmaxz := 1.0, 1.0, 1.0

	if dx > -1e-6 && dx < 1e-6 {
		if p1x <= -halfW || p1x >= halfW {
			return false
		}
	} else {
		invDx := 1.0 / dx
		t1 := (-halfW - p1x) * invDx
		t2 := (halfW - p1x) * invDx
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > 0 {
			tminx = t1
		}
		if t2 < 1 {
			tmaxx = t2
		}
	}

	if dy > -1e-6 && dy < 1e-6 {
		if p1y <= -halfH || p1y >= halfH {
			return false
		}
	} else {
		invDy := 1.0 / dy
		t1 := (-halfH - p1y) * invDy
		t2 := (halfH - p1y) * invDy
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > 0 {
			tminy = t1
		}
		if t2 < 1 {
			tmaxy = t2
		}
	}

	if dz > -1e-6 && dz < 1e-6 {
		if p1z <= -halfD || p1z >= halfD {
			return false
		}
	} else {
		invDz := 1.0 / dz
		t1 := (-halfD - p1z) * invDz
		t2 := (halfD - p1z) * invDz
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > 0 {
			tminz = t1
		}
		if t2 < 1 {
			tmaxz = t2
		}
	}

	if tminx >= 1 || tminy >= 1 || tminz >= 1 || tmaxx <= 0 || tmaxy <= 0 || tmaxz <= 0 {
		return false
	}

	min := tminx
	max := tmaxx
	hitDirection := 0

	if tminy > min {
		min = tminy
		hitDirection = 1
	}
	if tminz > min {
		min = tminz
		hitDirection = 2
	}

	if tmaxy < max {
		max = tmaxy
	}
	if tmaxz < max {
		max = tmaxz
	}

	if min > max {
		return false
	}
	if min == 0 {
		return false // the ray starts from inside
	}

	switch hitDirection {
	case 0:
		pdx := 1.0
		if dx > 0 {
			pdx = -1.0
		}
		hit.Normal.Set(pdx, 0, 0)
	case 1:
		pdy := 1.0
		if dy > 0 {
			pdy = -1.0
		}
		hit.Normal.Set(0, pdy, 0)
	case 2:
		pdz := 1.0
		if dz > 0 {
			pdz = -1.0
		}
		hit.Normal.Set(0, 0, pdz)
	}

	hit.Position.Set(p1x+min*dx, p1y+min*dy, p1z+min*dz)
	hit.Fraction = min

	return true
}

func (b *BoxGeometry) GetType() GeometryType {
	return _BOX
}

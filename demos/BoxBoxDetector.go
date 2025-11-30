package demos

import "fmt"

//////////////////////////////////////////////// BoxBoxDetector
// (oimo/collision/narrowphase/detector/BoxBoxDetector.go)
// Box vs Box detector.

const EDGE_BIAS_MULT = 1.0

type BoxBoxDetector struct {
	*Detector

	clipper *FaceClipper
}

func NewBoxBoxDetector() *BoxBoxDetector {
	return &BoxBoxDetector{
		Detector: NewDetector(false),
		clipper:  NewFaceClipper(),
	}
}

// override
func (bbd *BoxBoxDetector) detectImpl(result *DetectorResult, geom1, geom2 IGeometry, tf1, tf2 *Transform, cachedData *CachedDetectorData) {
	b1 := geom1.(*BoxGeometry)
	b2 := geom2.(*BoxGeometry)

	result.incremental = false

	// basis of box1 := {x1, y1, z1}
	// basis of box2 := {x2, y2, z2}
	// half-extents of box1 := {w1, h1, d1}
	// half-extents of box2 := {w2, h2, d2}
	//
	// candidates of the separating axis:
	//     x1,
	//     y1,
	//     z1,
	//     x2,
	//     y2,
	//     z2,
	//     cross(x1, x2),
	//     cross(x1, y2),
	//     cross(x1, z2),
	//     cross(y1, x2),
	//     cross(y1, y2),
	//     cross(y1, z2),
	//     cross(z1, x2),
	//     cross(z1, y2),
	//     cross(z1, z2).
	//
	// projected length of box1:
	//   project to       | length
	//   -------------------------------
	//   x1               | w1
	//   y1               | h1
	//   z1               | d1
	//   a = cross(x1, _) | h1|y1.a| + d1|z1.a|
	//   a = cross(y1, _) | w1|x1.a| + d1|z1.a|
	//   a = cross(z1, _) | w1|x1.a| + h1|y1.a|
	//   a = _            | w1|x1.a| + h1|y1.a| + d1|z1.a|
	//
	// projected length of box2:
	//   project to       | length
	//   -------------------------------
	//   x2               | w2
	//   y2               | h2
	//   z2               | d2
	//   a = cross(x2, _) | h2|y2.a| + d2|z2.a|
	//   a = cross(y2, _) | w2|x2.a| + d2|z2.a|
	//   a = cross(z2, _) | w2|x2.a| + h2|y2.a|
	//   a = _            | w2|x2.a| + h2|y2.a| + d2|z2.a|

	c1 := tf1.position
	c2 := tf2.position
	c12 := c2.Sub(c1) // from center1 to center2

	// bases
	x1 := tf1.rotation.GetCol(0)
	y1 := tf1.rotation.GetCol(1)
	z1 := tf1.rotation.GetCol(2)
	x2 := tf2.rotation.GetCol(0)
	y2 := tf2.rotation.GetCol(1)
	z2 := tf2.rotation.GetCol(2)

	// half extents
	w1 := b1.halfExtents.x
	h1 := b1.halfExtents.y
	d1 := b1.halfExtents.z
	w2 := b2.halfExtents.x
	h2 := b2.halfExtents.y
	d2 := b2.halfExtents.z

	// scaled bases by half extents
	sx1 := x1.Scale(w1)
	sy1 := y1.Scale(h1)
	sz1 := z1.Scale(d1)
	sx2 := x2.Scale(w2)
	sy2 := y2.Scale(h2)
	sz2 := z2.Scale(d2)

	// --------------------- SAT check start ---------------------
	mDepth := MathUtil.POSITIVE_INFINITY
	mId := -1
	mSign := 0
	var mAxis Vec3

	// --------------------- 6 faces ---------------------

	// try axis = x1
	proj1 := w1
	proj2 := bbd.project(x1, sx2, sy2, sz2)
	projC12 := x1.Dot(c12)
	// _satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, x1, 0, 1.0)
	_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, x1, 0, 1.0)

	// try axis = y1
	proj1 = h1
	proj2 = bbd.project(y1, sx2, sy2, sz2)
	projC12 = y1.Dot(c12)
	_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, y1, 1, 1.0)

	// try axis = z1
	proj1 = d1
	proj2 = bbd.project(z1, sx2, sy2, sz2)
	projC12 = z1.Dot(c12)
	_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, z1, 2, 1.0)

	// apply bias to avoid jitting
	if mDepth > Settings.LinearSlop {
		mDepth -= Settings.LinearSlop
	} else {
		mDepth = 0
	}

	// try axis = x2
	proj1 = bbd.project(x2, sx1, sy1, sz1)
	proj2 = w2
	projC12 = x2.Dot(c12)
	_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, x2, 3, 1.0)

	// try axis = y2
	proj1 = bbd.project(y2, sx1, sy1, sz1)
	proj2 = h2
	projC12 = y2.Dot(c12)
	_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, y2, 4, 1.0)

	// try axis = z2
	proj1 = bbd.project(z2, sx1, sy1, sz1)
	proj2 = d2
	projC12 = z2.Dot(c12)
	_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, z2, 5, 1.0)

	// --------------------- 9 edges ---------------------

	// apply bias again to avoid jitting
	if mDepth > Settings.LinearSlop {
		mDepth -= Settings.LinearSlop
	} else {
		mDepth = 0
	}

	// try cross(x1, x2)
	edgeAxis := x1.Cross(x2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sy1, sz1)
		proj2 = bbd.project2(edgeAxis, sy2, sz2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 6, EDGE_BIAS_MULT)
	}

	// try cross(x1, y2)
	edgeAxis = x1.Cross(y2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sy1, sz1)
		proj2 = bbd.project2(edgeAxis, sx2, sz2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 7, EDGE_BIAS_MULT)
	}

	// try cross(x1, z2)
	edgeAxis = x1.Cross(z2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sy1, sz1)
		proj2 = bbd.project2(edgeAxis, sx2, sy2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 8, EDGE_BIAS_MULT)
	}

	// try cross(y1, x2)
	edgeAxis = y1.Cross(x2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sx1, sz1)
		proj2 = bbd.project2(edgeAxis, sy2, sz2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 9, EDGE_BIAS_MULT)
	}

	// try cross(y1, y2)
	edgeAxis = y1.Cross(y2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sx1, sz1)
		proj2 = bbd.project2(edgeAxis, sx2, sz2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 10, EDGE_BIAS_MULT)
	}

	// try cross(y1, z2)
	edgeAxis = y1.Cross(z2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sx1, sz1)
		proj2 = bbd.project2(edgeAxis, sx2, sy2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 11, EDGE_BIAS_MULT)
	}

	// try cross(z1, x2)
	edgeAxis = z1.Cross(x2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sx1, sy1)
		proj2 = bbd.project2(edgeAxis, sy2, sz2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 12, EDGE_BIAS_MULT)
	}

	// try cross(z1, y2)
	edgeAxis = z1.Cross(y2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sx1, sy1)
		proj2 = bbd.project2(edgeAxis, sx2, sz2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 13, EDGE_BIAS_MULT)
	}

	// try cross(z1, z2)
	edgeAxis = z1.Cross(z2)
	if !edgeAxis.IsZero() {
		edgeAxis.Normalize()
		proj1 = bbd.project2(edgeAxis, sx1, sy1)
		proj2 = bbd.project2(edgeAxis, sx2, sy2)
		projC12 = edgeAxis.Dot(c12)
		_satCheck(&mDepth, &mId, &mSign, &mAxis, proj1, proj2, projC12, edgeAxis, 14, EDGE_BIAS_MULT)
	}

	// --------------------- edge-edge collision check ---------------------

	if mId >= 6 {
		// flip axis so that it directs from box1 to box2
		mAxis.ScaleEq(float64(mSign))

		// direction of edges: 0 = x, 1 = y, 2 = z
		id1 := (mId - 6) / 3
		id2 := (mId - 6) - id1*3

		// points on the edges
		var p1, p2 Vec3

		// directions
		var d1, d2 Vec3

		switch id1 {
		case 0: // use y and z
			d1 = x1
			_supportingVertexRect(&p1, sy1, sz1, mAxis)
		case 1: // use x and z
			d1 = y1
			_supportingVertexRect(&p1, sx1, sz1, mAxis)
		default: // use x and y
			d1 = z1
			_supportingVertexRect(&p1, sx1, sy1, mAxis)
		}
		p1.AddEq(c1)

		switch id2 {
		case 0: // user y and z
			d2 = x2
			_supportingVertexRect(&p2, sy2, sz2, mAxis)
		case 1: // use x and z
			d2 = y2
			_supportingVertexRect(&p2, sx1, sz2, mAxis)
		default: // use x and y
			d2 = z2
			_supportingVertexRect(&p2, sx2, sy2, mAxis)
		}
		p2.AddEq(c2)

		// compute params
		r := p1.Sub(p2)

		dot12 := d1.Dot(d2)
		dot1r := d1.Dot(r)
		dot2r := d2.Dot(r)

		invDet := 1.0 / (1.0 - dot12*dot12)
		t1 := (dot12*dot2r - dot1r) * invDet
		t2 := (dot2r - dot12*dot1r) * invDet

		// compute closest points and normal
		var cp1, cp2 Vec3
		MathUtil.Vec3_addRhsScaled(&cp1, &p1, &p2, t1)
		MathUtil.Vec3_addRhsScaled(&cp2, &p2, &d2, t2)

		normal := mAxis.Negate()

		// add contact point
		bbd.setNormal(result, normal)
		bbd.addPoint(result, cp1, cp2, mDepth, 4)
		return
	}

	// --------------------- face-face collision check ---------------------

	if mId >= 3 { // swap box1 and box2
		mSign = -mSign
		c12.NegateEq()
		b1, b2 = b2, b1
		w1, w2 = w2, w1
		h1, h2 = h2, h1
		d1, d2 = d2, d1
		c1, c2 = c2, c1
		x1, x2 = x2, x1
		y1, y2 = y2, y1
		z1, z2 = z2, z1
		sx1, sx2 = sx2, sx1
		sy1, sy2 = sy2, sy1
		sz1, sz2 = sz2, sz1

		mId -= 3
		bbd.swapped = true
	} else {
		bbd.swapped = false
	}

	// --------------------- find reference face ---------------------

	var refCenter, refNormal, refX, refY Vec3
	var refW, refH float64

	switch mId {
	case 0: // x+ or x-
		refCenter = sx1
		refNormal = x1
		refX = y1
		refY = z1
		refW = h1
		refH = d1
	case 1: // y+ or y-
		refCenter = sy1
		refNormal = y1
		refX = z1
		refY = x1
		refW = d1
		refH = w1
	default: // z+ or z-
		refCenter = sz1
		refNormal = z1
		refX = x1
		refY = y1
		refW = w1
		refH = h1
	}

	if mSign < 0 { // x- or y- or z-
		refCenter.NegateEq()
		refNormal.NegateEq()
		refX, refY = refY, refX
		refW, refH = refH, refW
	}

	// translate reference center
	refCenter.AddEq(c1)

	// --------------------- find incident face ---------------------

	minIncDot := 1.0
	incId := 0

	incDot := refNormal.Dot(x2)
	if incDot < minIncDot { // x+
		minIncDot = incDot
	}
	if -incDot < minIncDot { // x-
		minIncDot = -incDot
		incId = 1
	}
	incDot = refNormal.Dot(y2)
	if incDot < minIncDot { // y+
		minIncDot = incDot
		incId = 2
	}
	if -incDot < minIncDot { // y-
		minIncDot = -incDot
		incId = 3
	}

	incDot = refNormal.Dot(z2)
	if incDot < minIncDot { // y+
		minIncDot = incDot
		incId = 4
	}
	if -incDot < minIncDot { // y-
		minIncDot = -incDot
		incId = 5
	}

	var incV1, incV2, incV3, incV4 Vec3

	switch incId {
	case 0:
		_getBoxFace(&incV1, &incV2, &incV3, &incV4, sx2, sy2, sz2, "x+")
	case 1:
		_getBoxFace(&incV1, &incV2, &incV3, &incV4, sx2, sy2, sz2, "x-")
	case 2:
		_getBoxFace(&incV1, &incV2, &incV3, &incV4, sx2, sy2, sz2, "y+")
	case 3:
		_getBoxFace(&incV1, &incV2, &incV3, &incV4, sx2, sy2, sz2, "y-")
	case 4:
		_getBoxFace(&incV1, &incV2, &incV3, &incV4, sx2, sy2, sz2, "z+")
	default:
		_getBoxFace(&incV1, &incV2, &incV3, &incV4, sx2, sy2, sz2, "z-")
	}

	incV1.AddEq(c12)
	incV2.AddEq(c12)
	incV3.AddEq(c12)
	incV4.AddEq(c12)

	// --------------------- clip incident face ---------------------

	bbd.clipper.set(refW, refH)
	bbd.clipper.addIncidentVertex(incV1.Dot(refX), incV1.Dot(refY), incV1.x, incV1.y, incV1.z)
	bbd.clipper.addIncidentVertex(incV2.Dot(refX), incV2.Dot(refY), incV2.x, incV2.y, incV2.z)
	bbd.clipper.addIncidentVertex(incV3.Dot(refX), incV3.Dot(refY), incV3.x, incV3.y, incV3.z)
	bbd.clipper.addIncidentVertex(incV4.Dot(refX), incV4.Dot(refY), incV4.x, incV4.y, incV4.z)

	// --------------------- reduce vertices ---------------------

	bbd.clipper.reduce()

	// --------------------- add contact points ---------------------

	// set normal
	var normal Vec3
	if bbd.swapped {
		normal = refNormal
	} else {
		normal = refNormal.Negate()
	}
	bbd.setNormal(result, normal)

	// add contact points
	for i := range bbd.clipper.numVertices {
		v := bbd.clipper.vertices[i]

		clippedVertex := Vec3{v.wx, v.wy, v.wz}
		clippedVertex.AddEq(c1)

		clippedVertexToRefCenter := refCenter.Sub(clippedVertex)
		depth := clippedVertexToRefCenter.Dot(refNormal)

		var clippedVertexOnRefFace Vec3
		MathUtil.Vec3_addRhsScaled(&clippedVertexOnRefFace, &clippedVertex, &refNormal, depth)

		if depth > -Settings.ContactPersistenceThreshold {
			if bbd.swapped {
				bbd.addPoint(result, clippedVertex, clippedVertexOnRefFace, depth, i)
			} else {
				bbd.addPoint(result, clippedVertexOnRefFace, clippedVertex, depth, i)
			}
		}
	}
}

// Returns half of the projected length of the box with scaled bases (`sx`, `sy`, `sz`) onto the normalized axis `axis`.
func (bbd *BoxBoxDetector) project(axis, sx, sy, sz Vec3) float64 {
	dx := axis.Dot(sx)
	dy := axis.Dot(sy)
	dz := axis.Dot(sz)

	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	if dz < 0 {
		dz = -dz
	}
	return dx + dy + dz
}

// 2D version of `project`.
func (bbd *BoxBoxDetector) project2(axis, sx, sy Vec3) float64 {
	dx := axis.Dot(sx)
	dy := axis.Dot(sy)
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	return dx + dy
}

// --- Macros ---

func _satCheck(minDepth *float64, minDepthId, minDepthSign *int, minDepthAxis *Vec3, proj1, proj2, projC12 float64, axis Vec3, id int, biasMult float64) {
	sum := proj1 + proj2
	neg := projC12 < 0
	abs := projC12
	if neg {
		abs = -projC12
	}
	if abs < sum {
		depth := sum - abs
		if depth*biasMult < *minDepth {
			// giving some bias to edge-edge separating axes
			*minDepth = depth * biasMult
			*minDepthId = id
			*minDepthAxis = axis
			if neg {
				*minDepthSign = -1
			} else {
				*minDepthSign = 1
			}
		}
	} else {
		return
	}
}

func _supportingVertexRect(out *Vec3, halfExtX, halfExtY, axis Vec3) {
	signX := MathUtil.SignBiased(halfExtX.Dot(axis))
	signY := MathUtil.SignBiased(halfExtY.Dot(axis))
	_mix2(out, halfExtX, halfExtY, signX, signY)
}

func _mix2(dst *Vec3, v1, v2 Vec3, sign1, sign2 int) {
	switch {
	case sign1 == 1 && sign2 == 1:
		*dst = v1.Add(v2)
	case sign1 == 1 && sign2 == -1:
		*dst = v1.Sub(v2)
	case sign1 == -1 && sign2 == 1:
		*dst = v2.Sub(v1)
	case sign1 == -1 && sign2 == -1:
		*dst = v1.Add(v2)
		dst.NegateEq()
	default:
		panic(fmt.Sprintf("Mix2: invalid signs (%d, %d)", sign1, sign2))
	}
}

// No need for these in Go
// func _swapV(v1, v2 *Vec3) {
// 	*v1, *v2 = *v2, *v1
// }

// func _swap(f1, f2 *float64) {
// 	*f1, *f2 = *f2, *f1
// }

func _getBoxFace(v1, v2, v3, v4 *Vec3, basisX, basisY, basisZ Vec3, face string) {
	switch face {
	case "x+":
		_mix3(v1, basisX, basisY, basisZ, 1, 1, 1)
		_mix3(v2, basisX, basisY, basisZ, 1, -1, 1)
		_mix3(v3, basisX, basisY, basisZ, 1, -1, -1)
		_mix3(v4, basisX, basisY, basisZ, 1, 1, -1)
	case "x-":
		_mix3(v1, basisX, basisY, basisZ, -1, 1, 1)
		_mix3(v2, basisX, basisY, basisZ, -1, 1, -1)
		_mix3(v3, basisX, basisY, basisZ, -1, -1, -1)
		_mix3(v4, basisX, basisY, basisZ, -1, -1, 1)
	case "y+":
		_mix3(v1, basisX, basisY, basisZ, 1, 1, 1)
		_mix3(v2, basisX, basisY, basisZ, 1, 1, -1)
		_mix3(v3, basisX, basisY, basisZ, -1, 1, -1)
		_mix3(v4, basisX, basisY, basisZ, -1, 1, 1)
	case "y-":
		_mix3(v1, basisX, basisY, basisZ, 1, -1, 1)
		_mix3(v2, basisX, basisY, basisZ, -1, -1, 1)
		_mix3(v3, basisX, basisY, basisZ, -1, -1, -1)
		_mix3(v4, basisX, basisY, basisZ, 1, -1, -1)
	case "z+":
		_mix3(v1, basisX, basisY, basisZ, 1, 1, 1)
		_mix3(v2, basisX, basisY, basisZ, -1, 1, 1)
		_mix3(v3, basisX, basisY, basisZ, -1, -1, 1)
		_mix3(v4, basisX, basisY, basisZ, 1, -1, 1)
	case "z-":
		_mix3(v1, basisX, basisY, basisZ, 1, 1, -1)
		_mix3(v2, basisX, basisY, basisZ, 1, -1, -1)
		_mix3(v3, basisX, basisY, basisZ, -1, -1, -1)
		_mix3(v4, basisX, basisY, basisZ, -1, 1, -1)
	default:
		panic(fmt.Errorf("invalid face: %s", face))
	}
}

func _mix3(dst *Vec3, v1, v2, v3 Vec3, sign1, sign2, sign3 int) {
	switch sign3 {
	case 1:
		_mix2(dst, v1, v2, sign1, sign2)
		dst.AddEq(v3)
	case -1:
		_mix2(dst, v1, v2, sign1, sign2)
		dst.SubEq(v3)
	default:
		panic(fmt.Errorf("invalid sign: (%d %d %d)", sign1, sign2, sign3))
	}
}

// ////////////////////////////////////// IncidentVertex
type IncidentVertex struct {
	// projected coord
	x float64
	y float64

	// world coord
	wx float64
	wy float64
	wz float64
}

func NewIncidentVertex() *IncidentVertex {
	return &IncidentVertex{}
}

func (iv *IncidentVertex) set(x, y, wx, wy, wz float64) {
	iv.x, iv.y, iv.wx, iv.wy, iv.wz = x, y, wx, wy, wz
}

func (iv *IncidentVertex) interp(v1, v2 *IncidentVertex, t float64) {
	iv.x = v1.x + (v2.x-v1.x)*t
	iv.y = v1.y + (v2.y-v1.y)*t
	iv.wx = v1.wx + (v2.wx-v1.wx)*t
	iv.wy = v1.wy + (v2.wy-v1.wy)*t
	iv.wz = v1.wz + (v2.wz-v1.wz)*t
}

// ////////////////////////////////////// FaceClipper
type FaceClipper struct {
	w           float64
	h           float64
	numVertices int
	vertices    []*IncidentVertex

	numTmpVertices int
	tmpVertices    []*IncidentVertex
}

func NewFaceClipper() *FaceClipper {
	f := &FaceClipper{
		vertices:    make([]*IncidentVertex, 8),
		tmpVertices: make([]*IncidentVertex, 8),
	}
	for i := range 8 {
		f.vertices[i] = NewIncidentVertex()
		f.tmpVertices[i] = NewIncidentVertex()
	}
	return f
}

func (fc *FaceClipper) set(w, h float64) {
	fc.w = w
	fc.h = h
	fc.numVertices = 0
	fc.numTmpVertices = 0
}

func (fc *FaceClipper) addIncidentVertex(x, y, wx, wy, wz float64) {
	fc.vertices[fc.numVertices].set(x, y, wx, wy, wz)
	fc.numVertices++
}

// Clips the incident face by the reference face, generates up to eight vertices.
func (fc *FaceClipper) clip() {
	fc.clipL()
	fc.flip()
	fc.clipR()
	fc.flip()
	fc.clipT()
	fc.flip()
	fc.clipB()
	fc.flip()
}

// Reduces vertices up to four.
func (fc *FaceClipper) reduce() {
	if fc.numVertices < 4 {
		return
	}

	// TODO(oimo): maximize area
	max1 := MathUtil.NEGATIVE_INFINITY
	min1 := MathUtil.POSITIVE_INFINITY
	max2 := MathUtil.NEGATIVE_INFINITY
	min2 := MathUtil.POSITIVE_INFINITY

	var max1V *IncidentVertex
	var min1V *IncidentVertex
	var max2V *IncidentVertex
	var min2V *IncidentVertex

	e1x := 1.0
	e1y := 1.0
	e2x := -1.0
	e2y := 1.0

	for i := range fc.numVertices {
		v := fc.vertices[i]
		dot1 := v.x*e1x + v.y*e1y
		dot2 := v.x*e2x + v.y*e2y

		if i == 0 { // issue #32
			max1 = dot1
			max1V = v
			min1 = dot1
			min1V = v
			max2 = dot2
			max2V = v
			min2 = dot2
			min2V = v
		} else {
			if dot1 > max1 {
				max1 = dot1
				max1V = v
			}
			if dot1 < min1 {
				min1 = dot1
				min1V = v
			}
			if dot1 > max2 {
				max2 = dot2
				max2V = v
			}
			if dot2 < min2 {
				min2 = dot2
				min2V = v
			}
		}
	}

	fc.add(max1V)
	fc.add(max2V)
	fc.add(min1V)
	fc.add(min2V)
	fc.flip()
}

func (fc *FaceClipper) clipL() {
	for i := range fc.numVertices {
		v1 := fc.vertices[i]
		v2 := fc.vertices[(i+1)%fc.numVertices]
		s1 := fc.w + v1.x
		s2 := fc.w + v2.x
		fc.clipWithParam(v1, v2, s1, s2)
	}
}

func (fc *FaceClipper) clipR() {
	for i := range fc.numVertices {
		v1 := fc.vertices[i]
		v2 := fc.vertices[(i+1)%fc.numVertices]
		s1 := fc.w - v1.x
		s2 := fc.w - v2.x
		fc.clipWithParam(v1, v2, s1, s2)
	}
}

func (fc *FaceClipper) clipT() {
	for i := range fc.numVertices {
		v1 := fc.vertices[i]
		v2 := fc.vertices[(i+1)%fc.numVertices]
		s1 := fc.h + v1.y
		s2 := fc.h + v2.y
		fc.clipWithParam(v1, v2, s1, s2)
	}
}

func (fc *FaceClipper) clipB() {
	for i := range fc.numVertices {
		v1 := fc.vertices[i]
		v2 := fc.vertices[(i+1)%fc.numVertices]
		s1 := fc.h - v1.y
		s2 := fc.h - v2.y
		fc.clipWithParam(v1, v2, s1, s2)
	}
}

func (fc *FaceClipper) flip() {
	fc.vertices, fc.tmpVertices = fc.tmpVertices, fc.vertices
	fc.numVertices = fc.numTmpVertices
	fc.numTmpVertices = 0
}

func (fc *FaceClipper) clipWithParam(v1, v2 *IncidentVertex, s1, s2 float64) {
	if s1 > 0 && s2 > 0 {
		fc.add(v1)
	} else if s1 > 0 && s2 <= 0 {
		// v2 is clipped
		fc.add(v1)
		fc.interp(v1, v2, s1/(s1-s2))
	} else if s1 <= 0 && s2 > 0 {
		// v1 is clipped
		fc.interp(v1, v2, s1/(s1-s2))
	}
}

func (fc *FaceClipper) add(v *IncidentVertex) {
	*fc.tmpVertices[fc.numTmpVertices] = *v
	fc.numTmpVertices++
}

func (fc *FaceClipper) interp(v1, v2 *IncidentVertex, t float64) {
	fc.tmpVertices[fc.numTmpVertices].interp(v1, v2, t)
	fc.numTmpVertices++
}

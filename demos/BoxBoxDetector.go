package demos

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
func (bbd *BoxBoxDetector) detectImpl(result *DetectorResult, geom1, geom2 *Geometry, tf1, tf2 *Transform, cachedData *CachedDetectorData) {
	panic("not impl")
}

// Returns half of the projected length of the box with scaled bases (`sx`, `sy`, `sz`) onto the normalized axis `axis`.
func (bbd *BoxBoxDetector) project(axis, sx, sy, sz Vec3) float64 {
	panic("not impl")
}

// 2D version of `project`.
func (bbd *BoxBoxDetector) project2(axis, sx, sy Vec3) float64 {
	panic("not impl")
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

	// TODO: maximize area
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
	//BoxBoxDetectorMacro.swap(vertices, tmpVertices)
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

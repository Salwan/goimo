package demos

import "github.com/Salwan/goimo/debug"

/////////////////////////////////////// EpaTriangle
// (oimo/collision/narrowphase/detector/gjkepa/EpaTriangle.go)
// Internal class.

type EpaTriangle struct {
	next *EpaTriangle
	prev *EpaTriangle

	vertices          []*EpaVertex
	adjacentTriangles []*EpaTriangle
	adjacentPairIndex []int
	normal            Vec3
	distanceSq        float64

	nextIndex []int // (0, 1, 2) -> (1, 2, 0)

	tmpDfsId      int
	tmpDfsVisible bool

	tmp Vec3
	id  int
}

var _epaTriangle_nextIndex int = 0

func NewEpaTriangle() *EpaTriangle {
	et := EpaTriangle{
		vertices:          make([]*EpaVertex, 3),
		adjacentTriangles: make([]*EpaTriangle, 3),
		adjacentPairIndex: make([]int, 3),
		nextIndex:         []int{1, 2, 0},
		id:                _epaTriangle_nextIndex,
	}
	_epaTriangle_nextIndex++

	for i := range 3 {
		et.vertices[i] = NewEpaVertex()
		et.adjacentTriangles[i] = NewEpaTriangle()
	}
	return &et
}

// --- double linked list interface ---

func (c *EpaTriangle) GetNext() *EpaTriangle {
	return c.next
}

func (c *EpaTriangle) SetNext(x *EpaTriangle) {
	c.next = x
}

func (c *EpaTriangle) GetPrev() *EpaTriangle {
	return c.prev
}

func (c *EpaTriangle) SetPrev(x *EpaTriangle) {
	c.prev = x
}

func (self *EpaTriangle) checkVisible(id int, from Vec3) bool {
	// (oimo) if (id == _tmpDfsId) return _tmpDfsVisible;
	_ = id
	self.tmp = from.Sub(self.vertices[0].v)
	self.tmpDfsVisible = self.tmp.Dot(self.normal) > 0
	return self.tmpDfsVisible
}

func (self *EpaTriangle) init(vertex1, vertex2, vertex3 *EpaVertex, center Vec3, autoCheck bool) bool {
	v1 := vertex1.v
	v2 := vertex2.v
	v3 := vertex3.v
	vc := center

	v12 := v2.Sub(v1) // 1 to 2
	v13 := v3.Sub(v1) // 1 to 3
	vc1 := v1.Sub(vc) // c to 1

	inor := v12.Cross(v13)
	inverted := false
	d := vc1.Dot(inor)

	if d < 0 {
		if autoCheck {
			debug.GjkLog("found the triangle inverted, but it does not matter.")
			// vertices must be CCW
			vertex2, vertex3 = vertex3, vertex2
			inor.ScaleEq(-1.0)
		} else {
			debug.GjkLog("the triangle is inverted!")
			inverted = true
			// (oimo) return false
		}
	}

	*self.vertices[0] = *vertex1
	*self.vertices[1] = *vertex2
	*self.vertices[2] = *vertex3
	self.normal = inor
	SimplexUtil.projectOrigin3(vertex1.v, vertex2.v, vertex3.v, &self.tmp)
	self.distanceSq = self.tmp.LengthSq()

	self.adjacentTriangles[0] = nil
	self.adjacentTriangles[1] = nil
	self.adjacentTriangles[2] = nil
	self.adjacentPairIndex[0] = -1
	self.adjacentPairIndex[1] = -1
	self.adjacentPairIndex[2] = -1

	return !inverted
}

func (self *EpaTriangle) setAdjacentTriangle(triangle *EpaTriangle) bool {
	count := 0
	for i := range 3 {
		for j := range 3 {
			i2 := self.nextIndex[i]
			j2 := self.nextIndex[j]
			if self.vertices[i] == triangle.vertices[j2] && self.vertices[i2] == triangle.vertices[j] {
				*self.adjacentTriangles[i] = *triangle
				triangle.adjacentTriangles[j] = self

				self.adjacentPairIndex[i] = j
				triangle.adjacentPairIndex[j] = i

				count++
			}
		}
	}
	if count != 1 {
		debug.GjkLog("invalid polyhedron")
		debug.GjkLog("%d %d %d", self.vertices[0].randId, self.vertices[1].randId, self.vertices[2].randId)
		debug.GjkLog("%d %d %d", triangle.vertices[0].randId, triangle.vertices[1].randId, triangle.vertices[2])
		return false // invalid polyhedron
	}
	return true
}

func (self *EpaTriangle) removeAdjacentTriangles() {
	for i := range 3 {
		triangle := self.adjacentTriangles[i]
		if triangle != nil {
			pairIndex := self.adjacentPairIndex[i]
			triangle.adjacentTriangles[pairIndex] = nil
			triangle.adjacentPairIndex[pairIndex] = -1
			self.adjacentTriangles[i] = nil
			self.adjacentPairIndex[i] = -1
		}
	}
}

func (self *EpaTriangle) removeReferences() {
	self.next = nil
	self.prev = nil
	self.tmpDfsId = 0
	self.tmpDfsVisible = false
	self.distanceSq = 0
	self.vertices[0] = nil
	self.vertices[1] = nil
	self.vertices[2] = nil
	self.adjacentTriangles[0] = nil
	self.adjacentTriangles[1] = nil
	self.adjacentTriangles[2] = nil
	self.adjacentPairIndex[0] = 0
	self.adjacentPairIndex[1] = 0
	self.adjacentPairIndex[2] = 0
}

func (self *EpaTriangle) dump() {
	debug.GjkLog(`Face data:
  id:%d
  v1:%v
  v2:%v
  v3:%v
  n:%v
  distSq:%f
`, self.id, self.vertices[0].v, self.vertices[1].v, self.vertices[2].v, self.normal, self.distanceSq)
}

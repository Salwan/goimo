package demos

import (
	"fmt"

	"github.com/Salwan/goimo/debug"
)

//////////////////////////////////////////////// EpaPolyhedron
// (oimo/collision/narrowphase/detector/gjkepa/EpaPolyhedron.go)
// Internal class.

type EpaPolyhedron struct {
	vertices    []*EpaVertex
	numVertices int

	triangleList     *EpaTriangle
	triangleListLast *EpaTriangle
	numTriangles     int

	trianglePool *EpaTriangle
	vertexPool   *EpaVertex

	center Vec3
	status EpaPolyhedronState
}

func NewEpaPolyhedron() *EpaPolyhedron {
	e := &EpaPolyhedron{
		vertices: make([]*EpaVertex, Settings.MaxEPAVertices),
	}
	return e
}

// --- private ---

func (self *EpaPolyhedron) _pickTriangle() *EpaTriangle {
	var t *EpaTriangle
	self.trianglePool, t = SingleList_pick(self.trianglePool, NewEpaTriangle)
	return t
}

func (self *EpaPolyhedron) _poolTriangle(t *EpaTriangle) {
	t.removeReferences()
	self.trianglePool = SingleList_pool(self.trianglePool, t)
}

func (self *EpaPolyhedron) _setAdjacentTriangle(t1, t2 *EpaTriangle) {
	if !t1.setAdjacentTriangle(t2) {
		self.status = EpaPolyhedronState_INVALID_TRIANGLE
	}
}

func (self *EpaPolyhedron) _initTriangle(t *EpaTriangle, vertex1, vertex2, vertex3 *EpaVertex, center Vec3, autoCheck bool) {
	if !t.init(vertex1, vertex2, vertex3, center, autoCheck) {
		self.status = EpaPolyhedronState_INVALID_TRIANGLE
	}
}

func (self *EpaPolyhedron) _dumpHoleEdge(first *EpaVertex) {
	if debug.Debug {
		v := first
		vs := ""
		fs := ""
		cnt := 0
		for {
			cnt += 2
			vs += fmt.Sprintf("v %f %f %f\n", v.v.x, v.v.y, v.v.z)
			vs += fmt.Sprintf("v %f %f %f\n", v.v.x, v.v.y, v.v.z)
			fs += fmt.Sprintf("f %d %d %d\n", cnt-1, cnt, cnt+1)
			v = v.tmpEdgeLoopNext
			if v == first {
				break
			}
		}
		vs += fmt.Sprintf("v %f %f %f\n", v.v.x, v.v.y, v.v.z)
		debug.GjkLog("edge loop data:\n%v\n%v", vs, fs)
	}
}

func (self *EpaPolyhedron) _validate() bool {
	for t := self.triangleList; t != nil; {
		next := t.next

		for i := range 3 {
			t.vertices[i].tmpEdgeLoopOuterTriangle = nil
			t.vertices[i].tmpEdgeLoopNext = nil
			if t.adjacentPairIndex[i] == -1 {
				self.status = EpaPolyhedronState_NO_ADJACENT_PAIR_INDEX
				return false
				//(oimo) throw M.error("!?")
			}
			if t.adjacentTriangles[i] == nil {
				self.status = EpaPolyhedronState_NO_ADJACENT_TRIANGLE
				return false
				//throw M.error("!?")
			}
		}

		t = next
	}
	return true
}

func (self *EpaPolyhedron) _findEdgeLoop(id int, base *EpaTriangle, from Vec3) {
	if base.tmpDfsId == id {
		return
	}
	base.tmpDfsId = id
	debug.GjkLog("DFS: %d", base.id)
	if !base.checkVisible(id, from) {
		self.status = EpaPolyhedronState_TRIANGLE_INVISIBLE
		debug.GjkLog("tri %d is invisible", base.id)
		return
	}

	// find edges of the hole
	for i := range 3 {
		t := base.adjacentTriangles[i]
		if t == nil {
			continue
		}
		if t.checkVisible(id, from) {
			debug.GjkLog("tri %d is visible.", t.id)
			self._findEdgeLoop(id, t, from)
		} else {
			// triangle `base` can be seen from `from`, but triangle `t` cannot.
			debug.GjkLog("tri  %d is invisible.", t.id)
			debug.GjkLog("added edge: %d %d", base.id, t.id)
			i2 := base.nextIndex[i]
			v1 := base.vertices[i]
			v2 := base.vertices[i2]
			v1.tmpEdgeLoopNext = v2
			v1.tmpEdgeLoopOuterTriangle = t
		}
	}

	// expand the whole
	base.removeAdjacentTriangles()
	self._removeTriangle(base)
}

func (self *EpaPolyhedron) _addTriangle(t *EpaTriangle) {
	self.numTriangles++
	if debug.Debug {
		debug.GjkLog("triangle added %d %d", self.numTriangles, t.id)
		t.dump()
	}
	self.triangleList, self.triangleListLast = DoubleList_push(self.triangleList, self.triangleListLast, t)
}

func (self *EpaPolyhedron) _removeTriangle(t *EpaTriangle) {
	self.numTriangles--
	debug.GjkLog("triangle removed %d, id: %d", self.numTriangles, t.id)
	self.triangleList, self.triangleListLast = DoubleList_remove(self.triangleList, self.triangleListLast, t)
	self._poolTriangle(t)
}

// --- internal ---

func (self *EpaPolyhedron) pickVertex() *EpaVertex {
	var v *EpaVertex
	self.vertexPool, v = SingleList_pick(self.vertexPool, NewEpaVertex)
	return v
}

func (self *EpaPolyhedron) poolVertex(v *EpaVertex) {
	v.removeReferences()
	self.vertexPool = SingleList_pool(self.vertexPool, v)
}

func (self *EpaPolyhedron) clear() {
	for self.numTriangles > 0 {
		self._removeTriangle(self.triangleList)
	}
	if debug.Debug {
		if self.triangleList != nil {
			panic("Oimo assert here")
		}
		if self.triangleListLast != nil {
			panic("Oimo assert here")
		}
	}
	for self.numVertices > 0 {
		self.numVertices--
		self.poolVertex(self.vertices[self.numVertices])
	}
}

func (self *EpaPolyhedron) init(v1, v2, v3, v4 *EpaVertex) bool {
	self.status = EpaPolyhedronState_OK
	self.numVertices = 4

	self.vertices[0] = v1
	self.vertices[1] = v2
	self.vertices[2] = v3
	self.vertices[3] = v4
	self.center.CopyFrom(v1.v).AddEq(v2.v).AddEq(v3.v).AddEq(v4.v).ScaleEq(0.25)

	t1 := self._pickTriangle()
	t2 := self._pickTriangle()
	t3 := self._pickTriangle()
	t4 := self._pickTriangle()

	self._initTriangle(t1, v1, v2, v3, self.center, true)
	self._initTriangle(t2, v1, v2, v4, self.center, true)
	self._initTriangle(t3, v1, v3, v4, self.center, true)
	self._initTriangle(t4, v2, v3, v4, self.center, true)

	self._setAdjacentTriangle(t1, t2)
	self._setAdjacentTriangle(t1, t3)
	self._setAdjacentTriangle(t1, t4)
	self._setAdjacentTriangle(t2, t3)
	self._setAdjacentTriangle(t2, t4)
	self._setAdjacentTriangle(t3, t4)

	self._addTriangle(t1)
	self._addTriangle(t2)
	self._addTriangle(t3)
	self._addTriangle(t4)

	return self.status == EpaPolyhedronState_OK
}

func (self *EpaPolyhedron) getBestTriangle() *EpaTriangle {
	mind := MathUtil.POSITIVE_INFINITY
	var minf *EpaTriangle
	for f := self.triangleList; f != nil; f = f.next {
		if f.distanceSq < mind {
			mind = f.distanceSq
			minf = f
		}
	}
	return minf
}

func (self *EpaPolyhedron) addVertex(vertex *EpaVertex, base *EpaTriangle) bool {
	self.vertices[self.numVertices] = vertex
	self.numVertices++

	debug.GjkLog("vertex added %d %v", self.numVertices, vertex.v)
	debug.GjkLog("begin polyhedron modifying...")

	v1 := base.vertices[0]

	debug.GjkLog("trying to find a edge loop... v=%v", vertex.v)

	// make a hole on the polyhedron finding its edge loop
	self._findEdgeLoop(self.numVertices, base, vertex.v)
	if self.status != EpaPolyhedronState_OK {
		return false
	}

	if debug.Debug {
		self._dumpHoleEdge(v1)
	}

	// ... and "patch" the hole
	v := v1
	firstV := v1
	var prevT *EpaTriangle
	var firstT *EpaTriangle
	for {
		if v.tmpEdgeLoopNext == nil {
			debug.GjkLog("edge loop is broken:")
			self.dumpAsObjModel()
			self.status = EpaPolyhedronState_EDGE_LOOP_BROKEN
			return false
		}
		if v.tmpEdgeLoopOuterTriangle == nil {
			self.status = EpaPolyhedronState_NO_OUTER_TRIANGLE
			return false
		}

		t := self._pickTriangle()
		if firstT == nil {
			firstT = t
		}
		debug.GjkLog("patching...")

		self._initTriangle(t, v, v.tmpEdgeLoopNext, vertex, self.center, false)
		if self.status != EpaPolyhedronState_OK {
			return false
		}
		self._addTriangle(t)

		self._setAdjacentTriangle(t, v.tmpEdgeLoopOuterTriangle)
		if prevT != nil {
			self._setAdjacentTriangle(t, prevT)
		}

		prevT = t

		v = v.tmpEdgeLoopNext

		if v == firstV {
			break
		}
	}
	self._setAdjacentTriangle(prevT, firstT)

	return self.status == EpaPolyhedronState_OK && self._validate()
}

func (self *EpaPolyhedron) dumpAsObjModel() {
	if debug.Debug {
		debug.GjkLog("dumping .obj model of the polyhedron...")
		var vs string
		var fs string
		c := 0
		for f := self.triangleList; f != nil; {
			next := f.next

			vs += fmt.Sprintf("v %v %v %v\n", f.vertices[0].v.x, f.vertices[0].v.y, f.vertices[0].v.z)
			vs += fmt.Sprintf("v %v %v %v\n", f.vertices[1].v.x, f.vertices[1].v.y, f.vertices[1].v.z)
			vs += fmt.Sprintf("v %v %v %v\n", f.vertices[2].v.x, f.vertices[2].v.y, f.vertices[2].v.z)
			fs += fmt.Sprintf("f %d %d %d\n", c+1, c+2, c+3)
			c += 3

			f = next
		}
		debug.GjkLog("\n\n#EPAPolyhedron\n%v\n%v\n\n", vs, fs)
	}
}

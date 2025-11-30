package demos

import "github.com/Salwan/goimo/debug"

/////////////////////////////////////////////// ManifoldUpdater
// (oimo/dynamics/constraint/contact/ManifoldUpdater.go)
// Internal class.

type ManifoldUpdater struct {
	manifold *Manifold

	numOldPoints int
	oldPoints    []*ManifoldPoint
}

func NewManifoldUpdater(manifold *Manifold) *ManifoldUpdater {
	mu := &ManifoldUpdater{
		manifold:  manifold,
		oldPoints: make([]*ManifoldPoint, Settings.MaxManifoldPoints),
	}
	for i := range len(mu.oldPoints) {
		mu.oldPoints[i] = NewManifoldPoint()
	}
	return mu
}

// --- private ---

func (self *ManifoldUpdater) _removeOutdatedPoints() {
	num := self.manifold.numPoints
	for index := num - 1; index >= 0; index-- {
		p := self.manifold.points[index]

		diff := p.pos1.Sub(p.pos2)
		dotN := self.manifold.normal.Dot(diff)

		if dotN > Settings.ContactPersistenceThreshold {
			self._removeManifoldPoint(index)
			continue
		}

		// compute projection of diff
		diff = diff.AddRhsScaled(self.manifold.normal, -dotN)
		if diff.Dot(diff) > Settings.ContactPersistenceThreshold*Settings.ContactPersistenceThreshold {
			// the amount of horizontal sliding exceeds threshold
			self._removeManifoldPoint(index)
			continue
		}
	}
}

func (self *ManifoldUpdater) _removeManifoldPoint(index int) {
	self.manifold.numPoints--
	lastIndex := self.manifold.numPoints
	if index != lastIndex {
		self.manifold.points[index], self.manifold.points[lastIndex] = self.manifold.points[lastIndex], self.manifold.points[index]
	}
	self.manifold.points[lastIndex].clear()
}

func (self *ManifoldUpdater) _addManifoldPoint(point *DetectorResultPoint, tf1, tf2 *Transform) {
	// check if the number of points will exceed the limit
	num := self.manifold.numPoints
	if num == Settings.MaxManifoldPoints {
		targetIndex := self._computeTargetIndex(point, tf1, tf2)
		self.manifold.points[targetIndex].initialize(point, tf1, tf2)
		return
	}

	// just add the point
	self.manifold.points[num].initialize(point, tf1, tf2)
	self.manifold.numPoints++
}

func (self *ManifoldUpdater) _computeTargetIndex(newPoint *DetectorResultPoint, tf1, tf2 *Transform) int {
	p1 := self.manifold.points[0]
	p2 := self.manifold.points[1]
	p3 := self.manifold.points[2]
	p4 := self.manifold.points[3]

	maxDepth := p1.depth
	maxDepthIndex := 0

	if p2.depth > maxDepth {
		maxDepth = p2.depth
		maxDepthIndex = 1
	}
	if p3.depth > maxDepth {
		maxDepth = p3.depth
		maxDepthIndex = 2
	}
	if p4.depth > maxDepth {
		maxDepth = p4.depth
		maxDepthIndex = 3
	}

	rp1 := newPoint.position1
	rp1.SubEq(tf1.position)

	a1 := self._quadAreaFast(p2.relPos1, p3.relPos1, p4.relPos1, rp1)
	a2 := self._quadAreaFast(p1.relPos1, p3.relPos1, p4.relPos1, rp1)
	a3 := self._quadAreaFast(p1.relPos1, p2.relPos1, p4.relPos1, rp1)
	a4 := self._quadAreaFast(p1.relPos1, p2.relPos1, p3.relPos1, rp1)

	max := a1
	target := 0

	if a2 > max && maxDepthIndex != 1 || maxDepthIndex == 0 {
		max = a2
		target = 1
	}
	if a3 > max && maxDepthIndex != 2 {
		max = a3
		target = 2
	}
	if a4 > max && maxDepthIndex != 3 {
		max = a4
		target = 3
	}

	return target
}

func (self *ManifoldUpdater) _quadAreaFast(p1, p2, p3, p4 Vec3) float64 {
	// possible diagonals (12-34, 13-24, 14-23)

	v12 := p2.Sub(p1)
	v34 := p4.Sub(p3)
	v13 := p3.Sub(p1)
	v24 := p4.Sub(p2)
	v14 := p4.Sub(p1)
	v23 := p3.Sub(p2)

	cross1 := v12.Cross(v34)
	cross2 := v13.Cross(v24)
	cross3 := v14.Cross(v23)

	a1 := cross1.Dot(cross1)
	a2 := cross2.Dot(cross2)
	a3 := cross3.Dot(cross3)

	if a1 > a2 {
		if a1 > a3 {
			return a1
		} else {
			return a3
		}
	} else {
		if a2 > a3 {
			return a2
		} else {
			return a3
		}
	}
}

func (self *ManifoldUpdater) _computeRelativePositions(tf1, tf2 *Transform) {
	num := self.manifold.numPoints
	for i := range num {
		p := self.manifold.points[i]
		p.relPos1 = p.localPos1.MulMat3(&tf1.rotation)
		p.relPos2 = p.localPos2.MulMat3(&tf1.rotation)
		p.warmStarted = true // set warm starting flag
	}
}

func (self *ManifoldUpdater) _findNearestContactPointIndex(target *DetectorResultPoint, tf1, tf2 *Transform) int {
	nearestSq := Settings.ContactPersistenceThreshold * Settings.ContactPersistenceThreshold
	idx := -1

	for i := range self.manifold.numPoints {
		d := self._distSq(self.manifold.points[i], target, tf1, tf2)
		//trace("d is " + d);
		if d < nearestSq {
			nearestSq = d
			idx = i
		}
	}
	//trace("idx is " + idx);
	return idx
}

func (self *ManifoldUpdater) _distSq(mp *ManifoldPoint, result *DetectorResultPoint, tf1, tf2 *Transform) float64 {
	rp1 := result.position1
	rp2 := result.position2
	rp1.Sub(tf1.position)
	rp2.Sub(tf2.position)

	diff1 := mp.relPos1.Sub(rp1)
	diff2 := mp.relPos2.Sub(rp2)

	sq1 := diff1.Dot(diff1)
	sq2 := diff2.Dot(diff2)

	if sq1 < sq2 {
		return sq1
	} else {
		return sq2
	}
}

func (self *ManifoldUpdater) _saveOldData() {
	self.numOldPoints = self.manifold.numPoints
	for i := range self.numOldPoints {
		self.oldPoints[i].copyFrom(self.manifold.points[i])
	}
}

func (self *ManifoldUpdater) _updateContactPointById(cp *ManifoldPoint) {
	for i := range self.numOldPoints {
		ocp := self.oldPoints[i]
		if cp.id == ocp.id {
			cp.impulse.copyFrom(&ocp.impulse)
			cp.warmStarted = true
			break
		}
	}
}

// --- internal ---

func (self *ManifoldUpdater) totalUpdate(result *DetectorResult, tf1, tf2 *Transform) {
	self._saveOldData()

	num := result.numPoints
	self.manifold.numPoints = num

	for i := range num {
		p := self.manifold.points[i]
		ref := result.points[i]
		p.initialize(ref, tf1, tf2)
		self._updateContactPointById(p)
	}
}

func (self *ManifoldUpdater) incrementalUpdate(result *DetectorResult, tf1, tf2 *Transform) {
	// update old data
	self.manifold.updateDepthsAndPositions(tf1, tf2)

	// set warm started flag
	for i := range self.manifold.numPoints {
		self.manifold.points[i].warmStarted = true
	}

	if debug.Debug && result.numPoints != 1 {
		panic("OimoPhysics asserts here")
	}
	newPoint := result.points[0]

	// add or update point
	index := self._findNearestContactPointIndex(newPoint, tf1, tf2)
	if index == -1 {
		self._addManifoldPoint(newPoint, tf1, tf2)
	} else {
		cp := self.manifold.points[index]
		cp.updateDepthAndPositions(newPoint, tf1, tf2)
	}

	// remove some points
	self._removeOutdatedPoints()
}

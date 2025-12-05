package demos

//////////////////////////////////////////////// BruteForceBroadPhase
// (oimo/collision/broadphase/bruteforce/BruteForceBroadPhase.go)
// Brute force implementation of broad-phase collision detection. Time complexity is O(n^2).

type BruteForceBroadPhase struct {
	*BroadPhase
}

func NewBruteForceBroadPhase() *BruteForceBroadPhase {
	b := &BruteForceBroadPhase{
		BroadPhase: NewBroadPhase(BroadPhaseType_BRUTE_FORCE),
	}
	b.incremental = false
	return b
}

// --- private ---

func (self *BruteForceBroadPhase) _overlap(p1 IProxy, p2 IProxy) bool {
	return MathUtil.Aabb_overlap(p1.GetAabbMin(), p1.GetAabbMax(), p2.GetAabbMin(), p2.GetAabbMax())
}

// --- public

func (self *BruteForceBroadPhase) CreateProxy(userData any, aabb *Aabb) IProxy { // override
	proxy := NewProxy(userData, self.idCount)
	self.idCount++
	self._addProxy(proxy)

	proxy.SetAabb(aabb)
	return proxy
}

func (self *BruteForceBroadPhase) DestroyProxy(proxy IProxy) { // override
	self._removeProxy(proxy)

	proxy.SetUserData(nil)
}

func (self *BruteForceBroadPhase) MoveProxy(proxy IProxy, aabb *Aabb, displacement Vec3) { // override
	proxy.SetAabb(aabb)
}

func (self *BruteForceBroadPhase) CollectPairs() { // override
	self._poolProxyPairs()
	self.testCount = 0
	for p1 := self.proxyList; p1 != nil; {
		next := p1.GetNext()
		for p2 := next; p2 != nil; {
			next2 := p2.GetNext()
			self.testCount++
			if self._overlap(p1, p2) {
				self._pickAndPushProxyPair(p1, p2)
			}
			p2 = next2
		}
		p1 = next
	}
}

func (self *BruteForceBroadPhase) RayCast(begin Vec3, end Vec3, callback IBroadPhaseProxyCallback) { // override
	p1 := begin
	p2 := end

	// Unused
	// dir := p2.Sub(p1)
	// var min, max Vec3
	// MathUtil.Vec3_min(&min, &p1, &p2)
	// MathUtil.Vec3_max(&max, &p1, &p2)

	for p := self.proxyList; p != nil; {
		next := p.GetNext()
		if self._aabbSegmentTest(*p.GetAabbMin(), *p.GetAabbMax(), p1, p2) {
			callback.Process(p)
		}
		p = next
	}
}

func (self *BruteForceBroadPhase) ConvexCast(convex IConvexGeometry, begin *Transform, translation Vec3, callback IBroadPhaseProxyCallback) { // override
	for p := self.proxyList; p != nil; {
		next := p.GetNext()
		if self._aabbConvexSweepTest(*p.GetAabbMin(), *p.GetAabbMax(), convex, begin, translation) {
			callback.Process(p)
		}
		p = next
	}

}

func (self *BruteForceBroadPhase) AabbTest(aabb *Aabb, callback IBroadPhaseProxyCallback) { // override
	for p := self.proxyList; p != nil; {
		next := p.GetNext()
		if MathUtil.Aabb_overlap(&aabb.Min, &aabb.Max, p.GetAabbMin(), p.GetAabbMax()) {
			callback.Process(p)
		}
		p = next
	}
}

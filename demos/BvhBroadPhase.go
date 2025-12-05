package demos

//////////////////////////////////////////////// BvhBroadPhase
// (oimo/collision/broadphase/bvh/BvhBroadPhase.go)
// The broad-phase collision detection algorithm based on bounding volume hierarchy (BVH). Average time complexity is O(NlogN) or lower.

type BvhBroadPhase struct {
	*BroadPhase

	Tree *BvhTree

	movedProxies    []*BvhProxy
	numMovedProxies int
}

func NewBvhBroadPhase() *BvhBroadPhase {
	b := &BvhBroadPhase{
		BroadPhase:   NewBroadPhase(BroadPhaseType_BVH),
		Tree:         NewBvhTree(),
		movedProxies: make([]*BvhProxy, 1024),
	}
	b.incremental = true
	return b
}

// --- private ---

func (self *BvhBroadPhase) _addToMovedProxy(bvhProxy *BvhProxy) {
	// add to the buffer
	if bvhProxy.Moved {
		return
	}
	bvhProxy.Moved = true

	// expand the buffer
	if len(self.movedProxies) == self.numMovedProxies {
		self.movedProxies = Array_expand(self.movedProxies)
	}

	self.movedProxies[self.numMovedProxies] = bvhProxy
	self.numMovedProxies++
}

// displacement can be nil
func (self *BvhBroadPhase) _updateProxy(p *BvhProxy, aabb *Aabb, displacement *Vec3) {
	// set tight AABB
	p.SetAabb(aabb)

	// fatten the AABB
	padding := Settings.BvhProxyPadding
	paddingVec := Vec3{padding, padding, padding}
	p.aabbMin.SubEq(paddingVec)
	p.aabbMax.AddEq(paddingVec)

	if displacement != nil {
		// predict movement
		var zero Vec3
		var addToMin, addToMax Vec3
		MathUtil.Vec3_min(&addToMin, &zero, displacement)
		MathUtil.Vec3_max(&addToMax, &zero, displacement)
		p.aabbMin.AddEq(addToMin)
		p.aabbMax.AddEq(addToMax)
	}
}

func (self *BvhBroadPhase) _collide(n1, n2 *BvhNode) {
	self.testCount++
	l1 := n1.height == 0
	l2 := n2.height == 0
	if n1 == n2 {
		if l1 {
			return
		}
		self._collide(n1.children[0], n2)
		self._collide(n1.children[1], n2)
		return
	}

	if !MathUtil.Aabb_overlap(&n1.aabbMin, &n1.aabbMax, &n2.aabbMin, &n2.aabbMax) {
		return
	}
	if l1 && l2 {
		self._pickAndPushProxyPair(n1.proxy, n2.proxy)
		return
	}
	if l2 || n1.height > n2.height {
		// descend node 1
		self._collide(n1.children[0], n2)
		self._collide(n1.children[1], n2)
	} else {
		// descend node 2
		self._collide(n2.children[0], n1)
		self._collide(n2.children[1], n1)
	}
}

func (self *BvhBroadPhase) _rayCastRecursive(node *BvhNode, _p1, _p2 Vec3, callback IBroadPhaseProxyCallback) {
	if !self._aabbSegmentTest(node.aabbMin, node.aabbMax, _p1, _p2) {
		return
	}

	if node.height == 0 { // leaf
		callback.Process(node.proxy)
		return
	}

	self._rayCastRecursive(node.children[0], _p1, _p2, callback)
	self._rayCastRecursive(node.children[1], _p1, _p2, callback)
}

func (self *BvhBroadPhase) _convexCastRecursive(node *BvhNode, convex IConvexGeometry, begin *Transform, translation Vec3, callback IBroadPhaseProxyCallback) {
	if !self._aabbConvexSweepTest(node.aabbMin, node.aabbMax, convex, begin, translation) {
		return
	}

	if node.height == 0 { // leaf
		callback.Process(node.proxy)
		return
	}

	self._convexCastRecursive(node.children[0], convex, begin, translation, callback)
	self._convexCastRecursive(node.children[1], convex, begin, translation, callback)
}

func (self *BvhBroadPhase) _aabbTestRecursive(node *BvhNode, aabb *Aabb, callback IBroadPhaseProxyCallback) {
	if !MathUtil.Aabb_overlap(&node.aabbMin, &node.aabbMax, &aabb.Min, &aabb.Max) {
		return
	}

	if node.height == 0 { // leaf
		callback.Process(node.proxy)
		return
	}

	self._aabbTestRecursive(node.children[0], aabb, callback)
	self._aabbTestRecursive(node.children[1], aabb, callback)
}

// --- public ---

func (self *BvhBroadPhase) CreateProxy(userData any, aabb *Aabb) IProxy {
	p := NewBvhProxy(userData, self.idCount)
	self.idCount++
	self._addProxy(p)

	self._updateProxy(p, aabb, nil)
	self.Tree.insertProxy(p)
	self._addToMovedProxy(p)

	return p
}

func (self *BvhBroadPhase) DestroyProxy(proxy IProxy) {
	self._removeProxy(proxy)

	bvhProxy := proxy.(*BvhProxy)
	self.Tree.deleteProxy(bvhProxy)
	bvhProxy.userData = nil
	bvhProxy.next = nil
	bvhProxy.prev = nil

	if bvhProxy.Moved {
		bvhProxy.Moved = false
	}
}

func (self *BvhBroadPhase) MoveProxy(proxy IProxy, aabb *Aabb, displacement Vec3) {
	p := proxy.(*BvhProxy)
	if MathUtil.Aabb_contains(&p.aabbMin, &p.aabbMax, &aabb.Min, &aabb.Max) {
		// need not move proxy
		return
	}

	self._updateProxy(p, aabb, &displacement)
	self._addToMovedProxy(p)
}

func (self *BvhBroadPhase) CollectPairs() {
	self._poolProxyPairs()
	self.testCount = 0
	if self.numProxies < 2 {
		return
	}

	topDown := false

	// This never runs?
	if topDown {
		for self.numMovedProxies > 0 {
			self.numMovedProxies--
			self.movedProxies[self.numMovedProxies] = nil
		}
		self.Tree.buildTopDown()
		self._collide(self.Tree.root, self.Tree.root)
		return
	}

	incrementalCollision := float64(self.numMovedProxies)/float64(self.numProxies) < Settings.BvhIncrementalCollisionThreshold

	// incremental modification
	for i := range self.numMovedProxies {
		p := self.movedProxies[i]
		if p.Moved {
			self.Tree.deleteProxy(p)
			self.Tree.insertProxy(p)
			if incrementalCollision {
				self._collide(self.Tree.root, p.Leaf)
			}
			p.Moved = false
		}
		self.movedProxies[i] = nil
	}
	if !incrementalCollision {
		self._collide(self.Tree.root, self.Tree.root)
	}

	self.numMovedProxies = 0
}

func (self *BvhBroadPhase) RayCast(begin Vec3, end Vec3, callback IBroadPhaseProxyCallback) {
	if self.Tree.root == nil {
		return // no AABBs in the broadphase
	}

	self._rayCastRecursive(self.Tree.root, begin, end, callback)
}

func (self *BvhBroadPhase) ConvexCast(convex IConvexGeometry, begin *Transform, translation Vec3, callback IBroadPhaseProxyCallback) {
	if self.Tree.root == nil {
		return // no AABBs in the broadphase
	}

	self._convexCastRecursive(self.Tree.root, convex, begin, translation, callback)
}

func (self *BvhBroadPhase) AabbTest(aabb *Aabb, callback IBroadPhaseProxyCallback) {
	if self.Tree.root == nil {
		return // no AABBs in the broadphase
	}
	self._aabbTestRecursive(self.Tree.root, aabb, callback)
}

// Returns the balance of the bounding volume tree.
func (self *BvhBroadPhase) getTreeBalance() int {
	return self.Tree.getBalance()
}

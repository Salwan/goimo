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
		BroadPhase:   NewBroadPhase(_BVH),
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
func (self *BvhBroadPhase) _updateProxy(p *BvhProxy, aabb Aabb, displacement *Vec3) {
	// set tight AABB
	p.setAabb(aabb)

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
		p.aabbMin.Add(addToMin)
		p.aabbMax.Add(addToMax)
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
		self._collide(n2.children[1], n2)
		return
	}

	if !MathUtil.Aabb_overlap(&n1.aabbMin, &n1.aabbMax, &n2.aabbMin, &n2.aabbMax) {
		return
	}
	if l1 && l2 {
		// HERE: switch BroadPhase *Proxy to use IProxy for this to work
		self._pickAndPushProxyPair(n1.proxy, n2.proxy)
		return
	}
}

func (self *BvhBroadPhase) _rayCastRecursive(node *BvhNode, _p1, _p2 Vec3, callback *BroadPhaseProxyCallback) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) _convexCastRecursive(node *BvhNode, convex IConvexGeometry, begin *Transform, translation Vec3, callback *BroadPhaseProxyCallback) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) _aabbTestRecursive(node *BvhNode, aabb Aabb, callback *BroadPhaseProxyCallback) {
	// TODO
	panic("not impl")
}

// --- public ---

func (self *BvhBroadPhase) CreateProxy(userData any, aabb *Aabb) *Proxy {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) DestroyProxy(proxy *Proxy) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) MoveProxy(proxy *Proxy, aabb *Aabb, displacement Vec3) {
	// TODO
	panic("not impl")
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

func (self *BvhBroadPhase) RayCast(begin Vec3, end Vec3, callback *BroadPhaseProxyCallback) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) ConvexCast(convex IConvexGeometry, begin *Transform, translation Vec3, callback *BroadPhaseProxyCallback) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) AabbTest(aabb *Aabb, callback *BroadPhaseProxyCallback) {
	// TODO
	panic("not impl")
}

// Returns the balance of the bounding volume tree.
func (self *BvhBroadPhase) getTreeBalance() int {
	// TODO
	panic("not impl")
}

// TODO

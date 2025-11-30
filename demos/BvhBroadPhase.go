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
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) _updateProxy(p *BvhProxy, aabb Aabb, displacement Vec3) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) _collide(n1 *BvhNode, n2 *BvhNode) {
	// TODO
	panic("not impl")
}

func (self *BvhBroadPhase) _rayCastRecursive(node *BvhNode, _p1 Vec3, _p2 Vec3, callback *BroadPhaseProxyCallback) {
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

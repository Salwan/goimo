package demos

import "math"

// /////////////////////////////////////// BroadPhase
// (oimo/dynamics/rigidbody/Shape.go)
// The abstract class of a broad-phase collision detection algorithm.
type IBroadPhase interface {
	GetProxyPairList() *ProxyPair

	// Moves the proxy `proxy` to the axis-aligned bounding box `aabb`. `displacement` is the difference between current and previous center of the AABB. This is used for predicting movement of the proxy.
	MoveProxy(proxy IProxy, aabb *Aabb, displacement Vec3)

	// Collects overlapping pairs of the proxies and put them into a linked list. The linked list can be get through `BroadPhase.getProxyPairList` method.
	// Note that in order to collect pairs, the broad-phase algorithm requires to be informed of movements of proxies through `BroadPhase.moveProxy` method.
	CollectPairs()

	// Returns whether the pair of `proxy1` and `proxy2` is overlapping. As proxies can be larger than the containing AABBs, two proxies may overlap even though their inner AABBs are separate.
	IsOverlapping(proxy1, proxy2 IProxy) bool

	IsIncremental() bool

	CreateProxy(userData any, aabb *Aabb) IProxy
	DestroyProxy(proxy IProxy)
	RayCast(begin Vec3, end Vec3, callback IBroadPhaseProxyCallback)
	ConvexCast(convex IConvexGeometry, begin *Transform, translation Vec3, callback IBroadPhaseProxyCallback)
	AabbTest(aabb *Aabb, callback IBroadPhaseProxyCallback)
}

type BroadPhase struct {
	_type         BroadPhaseType
	numProxies    int
	proxyList     IProxy
	proxyListLast IProxy

	proxyPairList *ProxyPair
	incremental   bool

	testCount int

	proxyPairPool *ProxyPair
	idCount       int

	convexSweep *ConvexSweepGeometry
	aabb        *AabbGeometry

	identity   Transform
	zero       Vec3
	rayCastHit RayCastHit
}

func NewBroadPhase(_type_ BroadPhaseType) *BroadPhase {
	b := &BroadPhase{
		_type: _type_,

		convexSweep: NewConvexSweepGeometry(),
		aabb:        NewAabbGeometry(),
	}
	return b
}

// --- private ---

func (bp *BroadPhase) _pickAndPushProxyPair(p1, p2 IProxy) {
	var pp *ProxyPair
	bp.proxyPairPool, pp = SingleList_pick(bp.proxyPairPool, NewProxyPair)
	bp.proxyPairPool = SingleList_addFirst(bp.proxyPairList, pp)
	pp.p1 = p1
	pp.p2 = p2
}

func (self *BroadPhase) _poolProxyPairs() {
	p := self.proxyPairList
	if p != nil {
		for {
			p.p1 = nil
			p.p2 = nil
			p = p.next
			if p == nil {
				break
			}
		}
		self.proxyPairList.next = self.proxyPairPool
		self.proxyPairPool = self.proxyPairList
		self.proxyPairList = nil
	}
}

func (self *BroadPhase) _addProxy(p IProxy) {
	self.numProxies++
	self.proxyList, self.proxyListLast = DoubleList_push(self.proxyList, self.proxyListLast, p)
}

func (self *BroadPhase) _removeProxy(p IProxy) {
	self.numProxies--
	self.proxyList, self.proxyListLast = DoubleList_remove(self.proxyList, self.proxyListLast, p)
}

func (self *BroadPhase) _aabbSegmentTest(aabbMin, aabbMax, begin, end Vec3) bool {
	x1 := begin.x
	y1 := begin.y
	z1 := begin.z
	x2 := end.x
	y2 := end.y
	z2 := end.z

	sminx := math.Min(x1, x2)
	sminy := math.Min(y1, y2)
	sminz := math.Min(z1, z2)
	smaxx := math.Max(x1, x2)
	smaxy := math.Max(y1, y2)
	smaxz := math.Max(z1, z2)

	pminx := aabbMin.x
	pminy := aabbMin.y
	pminz := aabbMin.z
	pmaxx := aabbMax.x
	pmaxy := aabbMax.y
	pmaxz := aabbMax.z

	// axis1: (1, 0, 0)
	// axis2: (0, 1, 0)
	// axis3: (0, 0, 1)
	if pminx > smaxx || pmaxx < sminx ||
		pminy > smaxy || pmaxy < sminy ||
		pminz > smaxz || pmaxz < sminz {
		return false
	}

	dx := x2 - x1
	dy := y2 - y1
	dz := z2 - z1
	adx := math.Abs(dx)
	ady := math.Abs(dy)
	adz := math.Abs(dz)

	pextx := (pmaxx - pminx) * 0.5
	pexty := (pmaxy - pminy) * 0.5
	pextz := (pmaxz - pminz) * 0.5
	pcntx := (pmaxx + pminx) * 0.5
	pcnty := (pmaxy + pminy) * 0.5
	pcntz := (pmaxz + pminz) * 0.5

	cpx := x1 - pcntx
	cpy := y1 - pcnty
	cpz := z1 - pcntz

	// axis4: (dx, dy, dz) x (1, 0, 0) = (0, dz, -dy)
	// axis5: (dx, dy, dz) x (0, 1, 0) = (-dz, 0, dx)
	// axis6: (dx, dy, dz) x (0, 0, 1) = (dy, -dx, 0)
	if math.Abs(cpy*dz-cpz*dy)-(pexty*adz+pextz*ady) > 0 ||
		math.Abs(cpz*dx-cpx*dz)-(pextz*adx+pextx*adz) > 0 ||
		math.Abs(cpx*dy-cpy*dx)-(pextx*ady+pexty*adx) > 0 {
		return false
	}

	return true
}

func (self *BroadPhase) aabbConvexSweepTest(aabbMin, aabbMax Vec3, convex IConvexGeometry, begin *Transform, translation Vec3) bool {
	self.aabb.min = aabbMin
	self.aabb.max = aabbMax
	self.convexSweep.Set(convex, begin, translation)

	if GjkEpaInstance.ComputeDistance(self.convexSweep, self.aabb, begin, &self.identity, nil) == _SUCCEEDED {
		return GjkEpaInstance.Distance <= 0
	}
	return false
}

// --- public ---

func (bp *BroadPhase) MoveProxy(proxy IProxy, aabb *Aabb, displacement Vec3) {}

func (bp *BroadPhase) IsOverlapping(proxy1, proxy2 IProxy) bool {
	// TODO
	panic("not impl")
}

func (bp *BroadPhase) CollectPairs() {}

func (bp *BroadPhase) GetProxyPairList() *ProxyPair {
	return bp.proxyPairList
}

func (self *BroadPhase) IsIncremental() bool {
	return self.incremental
}

func (self *BroadPhase) CreateProxy(userData any, aabb *Aabb) IProxy {
	panic("abstract call")
}
func (self *BroadPhase) DestroyProxy(proxy IProxy) {
	panic("abstract call")
}
func (self *BroadPhase) RayCast(begin Vec3, end Vec3, callback IBroadPhaseProxyCallback) {
	panic("abstract call")
}
func (self *BroadPhase) ConvexCast(convex IConvexGeometry, begin *Transform, translation Vec3, callback IBroadPhaseProxyCallback) {
	panic("abstract call")
}
func (self *BroadPhase) AabbTest(aabb *Aabb, callback IBroadPhaseProxyCallback) {
	panic("abstract call")
}

// /////////////////////////////////////////// ConvexSweepGeometry
type ConvexSweepGeometry struct {
	*ConvexGeometry

	c                IConvexGeometry
	localTranslation Vec3
}

func NewConvexSweepGeometry() *ConvexSweepGeometry {
	return &ConvexSweepGeometry{
		ConvexGeometry: NewConvexGeometry(-1),
	}
}

func (csg *ConvexSweepGeometry) Set(c IConvexGeometry, transform *Transform, translation Vec3) {
	csg.c = c
	MathUtil.Vec3_mulMat3Transposed(&csg.localTranslation, &translation, &transform.rotation)
	csg.gjkMargin = c.GetGjkMargin()
}

func (csg *ConvexSweepGeometry) ComputeLocalSupportingVertex(dir Vec3, out *Vec3) {
	csg.c.ComputeLocalSupportingVertex(dir, out)
	if dir.Dot(csg.localTranslation) > 0 {
		out.AddEq(csg.localTranslation)
	}
}

// /////////////////////////////////////////// AabbGeometry
type AabbGeometry struct {
	*ConvexGeometry

	min Vec3
	max Vec3
}

func NewAabbGeometry() *AabbGeometry {
	return &AabbGeometry{
		ConvexGeometry: NewConvexGeometry(-1),
	}
}

func (ag *AabbGeometry) ComputeLocalSupportingVertex(dir Vec3, out *Vec3) {
	if dir.x > 0 {
		out.x = ag.max.x
	} else {
		out.x = ag.min.x
	}
	if dir.y > 0 {
		out.y = ag.max.y
	} else {
		out.y = ag.min.y
	}
	if dir.z > 0 {
		out.z = ag.max.z
	} else {
		out.z = ag.min.z
	}
}

// TODO

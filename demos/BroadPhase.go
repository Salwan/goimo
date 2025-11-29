package demos

// /////////////////////////////////////// BroadPhase
// (oimo/dynamics/rigidbody/Shape.go)
// The abstract class of a broad-phase collision detection algorithm.
type IBroadPhase interface {
	// Moves the proxy `proxy` to the axis-aligned bounding box `aabb`. `displacement` is the difference between current and previous center of the AABB. This is used for predicting movement of the proxy.
	moveProxy(proxy *Proxy, aabb *Aabb, displacement Vec3)

	// Collects overlapping pairs of the proxies and put them into a linked list. The linked list can be get through `BroadPhase.getProxyPairList` method.
	// Note that in order to collect pairs, the broad-phase algorithm requires to be informed of movements of proxies through `BroadPhase.moveProxy` method.
	collectPairs()
}

type BroadPhase struct {
	_type         BroadPhaseType
	numProxies    int
	proxyList     *Proxy
	proxyListLast *Proxy

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

// Private

func (bp *BroadPhase) pickAndPushProxyPair(p1 *Proxy, p2 *Proxy) {
	pp := SingleList_pick(&bp.proxyPairPool, NewProxyPair)
	SingleList_addFirst(&bp.proxyPairList, pp)
	pp.p1 = p1
	pp.p2 = p2
}

func (bp *BroadPhase) moveProxy(proxy *Proxy, aabb *Aabb, displacement Vec3) {}

func (bp *BroadPhase) collectPairs() {}

// TODO

// /////////////////////////////////////////// ConvexSweepGeometry
type ConvexSweepGeometry struct {
	*ConvexGeometry

	c                *ConvexGeometry
	localTranslation Vec3
}

func NewConvexSweepGeometry() *ConvexSweepGeometry {
	return &ConvexSweepGeometry{
		ConvexGeometry: NewConvexGeometry(-1),
	}
}

func (csg *ConvexSweepGeometry) Set(c *ConvexGeometry, transform *Transform, translation Vec3) {
	csg.c = c
	MathUtil.Vec3_mulMat3Transposed(&csg.localTranslation, &translation, &transform.rotation)
	csg.gjkMargin = c.gjkMargin
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

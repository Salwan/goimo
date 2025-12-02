package demos

//////////////////////////////////////////////// BvhNode
// (oimo/collision/broadphase/bvh/BvhNode.go)
// Internal class.

type BvhNode struct {
	// for object pool
	next *BvhNode

	// for BvhTree.leafList
	prevLeaf *BvhNode
	nextLeaf *BvhNode

	children   []*BvhNode
	childIndex int // must be 0 or 1 regardless of having parent
	parent     *BvhNode
	height     int
	proxy      IProxy

	// node's aabb. if the node is a leaf, the aabb is equal to the proxy's one.
	aabbMin Vec3
	aabbMax Vec3

	// used by other classes
	tmp Vec3
}

func NewBvhNode() *BvhNode {
	b := &BvhNode{
		children: make([]*BvhNode, 2),
	}
	return b
}

// --- single linked list interface ---

func (c *BvhNode) GetNext() *BvhNode {
	return c.next
}

func (c *BvhNode) SetNext(x *BvhNode) {
	c.next = x
}

// --- internal ---

func (self *BvhNode) setChild(index int, child *BvhNode) {
	self.children[index] = child
	child.parent = self
	child.childIndex = index
}

func (self *BvhNode) removeReferences() {
	self.next = nil
	self.childIndex = 0
	self.children[0] = nil
	self.children[1] = nil
	self.childIndex = 0
	self.parent = nil
	self.height = 0
	self.proxy = nil
}

func (self *BvhNode) computeAabb() {
	c1 := self.children[0]
	c2 := self.children[1]
	MathUtil.Vec3_min(&self.aabbMin, &c1.aabbMin, &c2.aabbMin)
	MathUtil.Vec3_max(&self.aabbMax, &c1.aabbMax, &c2.aabbMax)
}

func (self *BvhNode) computeHeight() {
	h1 := self.children[0].height
	h2 := self.children[1].height
	if h1 > h2 {
		self.height = h1 + 1
	} else {
		self.height = h2 + 1
	}
}

func (self *BvhNode) perimeter() float64 {
	size := self.aabbMax.Sub(self.aabbMin)
	x := size.x
	y := size.y
	z := size.z
	return x*(y+z) + y*z
}

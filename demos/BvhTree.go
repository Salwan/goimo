package demos

import (
	"math/rand"

	"github.com/Salwan/goimo/debug"
)

//////////////////////////////////////////////// BvhTree
// (oimo/collision/broadphase/bvh/BvhTree.go)
// Internal class.

type BvhTree struct {
	root      *BvhNode
	numLeaves int
	strategy  *BvhStrategy

	nodePool *BvhNode

	leafList     *BvhNode
	leafListLast *BvhNode

	tmp []*BvhNode
}

func NewBvhTree() *BvhTree {
	return &BvhTree{
		strategy: NewBvhStrategy(),
		tmp:      make([]*BvhNode, 1024),
	}
}

// --- internal ---

func (self *BvhTree) print(root *BvhNode, indent string) {
	if root == nil {
		return
	}
	if root.height == 0 {
		debug.GjkLog("%v%v", indent, root.proxy.GetID())
	} else {
		self.print(root.children[0], indent+"  ")
		debug.GjkLog("%v#%v, %v", indent, root.height, MathUtil.ToFixed4(root.perimeter()))
		self.print(root.children[1], indent+"  ")
	}
}

// BvhNode uses nextLeaf and prevLeaf AS WELL as next, so this is treated special
func BvhNode_push(head, tail **BvhNode, node *BvhNode) {
	if *head == nil {
		*head = node
		*tail = *head
	} else {
		(*tail).nextLeaf = node
		node.prevLeaf = *tail
		*tail = node
	}
}

func BvhNode_remove(head, tail **BvhNode, node *BvhNode) {
	prev := node.prevLeaf
	next := node.nextLeaf

	if prev != nil {
		prev.nextLeaf = next
	}
	if next != nil {
		next.prevLeaf = prev
	}
	if node == *head {
		*head = (*head).nextLeaf
	}
	if node == *tail {
		*tail = (*tail).prevLeaf
	}
	node.nextLeaf = nil
	node.prevLeaf = nil
}

// Inserts the proxy. This creates a leaf connected to the proxy and inserts it to the tree and `leafList`.
func (self *BvhTree) insertProxy(proxy *BvhProxy) {
	leaf := self._pick()
	leaf.proxy = proxy
	proxy.Leaf = leaf

	leaf.aabbMin = proxy.aabbMin
	leaf.aabbMax = proxy.aabbMax

	self.numLeaves++
	BvhNode_push(&self.leafList, &self.leafListLast, leaf)

	self._insertLeaf(leaf)
}

// Deletes the proxy. This also deletes the leaf connected to the proxy from the tree and `leafList`.
func (self *BvhTree) deleteProxy(proxy *BvhProxy) {
	leaf := proxy.Leaf

	self.numLeaves--
	BvhNode_remove(&self.leafList, &self.leafListLast, leaf)

	self._deleteLeaf(leaf)

	proxy.Leaf = nil
	self._pool(leaf)
}

// Clears whole the tree.
// All leaves are disposed and deleted from `leafList`.
func (self *BvhTree) clear() {
	if self.root == nil {
		return
	}
	self._deleteRecursive(self.root)
	self.root = nil
	self.numLeaves = 0
}

func (self *BvhTree) optimize(count int) {
	if self.root == nil {
		return
	}
	for range count {
		leaf := self.root
		for leaf.height > 0 {
			h1 := float64(leaf.children[0].height)
			h2 := float64(leaf.children[1].height)
			// TODO(oimo): better strategy
			ix := 0
			if rand.Float64() > (h1 / (h1 + h2)) {
				ix = 1
			}
			leaf = leaf.children[ix]
		}
		self._deleteLeaf(leaf)
		self._insertLeaf(leaf)
	}
}

func (self *BvhTree) buildTopDown() {
	if self.root == nil {
		return
	}
	self._decompose()

	for len(self.tmp) < self.numLeaves {
		self.tmp = Array_expand(self.tmp)
	}

	// collect leaves
	idx := 0
	for leaf := self.leafList; leaf != nil; leaf = leaf.nextLeaf {
		self.tmp[idx] = leaf
		idx++
	}

	self.root = self._buildTopDownRecursive(self.tmp, 0, self.numLeaves)
}

func (self *BvhTree) getBalance() int {
	return self._getBalanceRecursive(self.root)
}

// --- private ---

// Makes the tree empty, but leaf nodes are not disposed and are reusable.
// The tree must be reconstructed using `leafList` after the call of this method.
func (self *BvhTree) _decompose() {
	if self.root == nil {
		return
	}
	self._decomposeRecursive(self.root)
	self.root = nil
}

func (self *BvhTree) _deleteRecursive(root *BvhNode) {
	if root.height == 0 {
		BvhNode_remove(&self.leafList, &self.leafListLast, root)
		root.proxy.(*BvhProxy).Leaf = nil
		self._pool(root)
		return
	}
	self._deleteRecursive(root.children[0])
	self._deleteRecursive(root.children[1])
	self._pool(root)
}

func (self *BvhTree) _decomposeRecursive(root *BvhNode) {
	if root.height == 0 {
		root.childIndex = 0
		root.parent = nil
		return
	}
	self._decomposeRecursive(root.children[0])
	self._decomposeRecursive(root.children[1])
	self._pool(root)
}

func (self *BvhTree) _buildTopDownRecursive(leaves []*BvhNode, from int, until int) *BvhNode {
	num := until - from
	if debug.Debug && num < 0 {
		panic("Oimo asserts here")
	}
	if num == 1 {
		leaf := leaves[from]
		proxy := leaf.proxy
		leaf.aabbMin = *proxy.GetAabbMin()
		leaf.aabbMax = *proxy.GetAabbMax()
		return leaf
	}

	// sort and split
	splitAt := self.strategy.splitLeaves(leaves, from, until)
	child1 := self._buildTopDownRecursive(leaves, from, splitAt)
	child2 := self._buildTopDownRecursive(leaves, splitAt, until)
	parent := self._pick()
	parent.setChild(0, child1)
	parent.setChild(1, child2)
	parent.computeAabb()
	parent.computeHeight()
	return parent
}

func (self *BvhTree) _getBalanceRecursive(root *BvhNode) int {
	if root == nil || root.height == 0 {
		return 0
	}
	balance := root.children[0].height - root.children[1].height
	if balance < 0 {
		balance = -balance
	}
	return balance + self._getBalanceRecursive(root.children[0]) + self._getBalanceRecursive(root.children[1])
}

func (self *BvhTree) _insertLeaf(leaf *BvhNode) {
	self._assertBeLeaf(leaf)
	if self.root == nil { // tree is empty
		self.root = leaf
		return
	}

	// search for the position to insert
	sibling := self.root

	for sibling.height > 0 {
		nextStep := self.strategy.decideInsertion(sibling, leaf)

		if nextStep == -1 {
			// insert to current position
			break
		} else {
			sibling = sibling.children[nextStep]
		}
	}

	parent := sibling.parent

	// new common parent with the sibling
	node := self._pick()

	if parent == nil {
		// replace root node
		self.root = node
	} else {
		// connect to the old parent
		parent.setChild(sibling.childIndex, node)
	}
	node.setChild(sibling.childIndex, sibling)
	node.setChild(sibling.childIndex^1, leaf)

	// fix data
	for node != nil {
		if self.strategy.balancingEnabled {
			node = self._balance(node)
		}

		node.computeHeight()
		node.computeAabb()
		node = node.parent
	}
}

func (self *BvhTree) _deleteLeaf(leaf *BvhNode) {
	self._assertBeLeaf(leaf)
	if self.root == leaf { // the tree has only the leaf
		self.root = nil
		return
	}
	parent := leaf.parent
	sibling := parent.children[leaf.childIndex^1]
	grandParent := parent.parent
	if grandParent == nil {
		sibling.parent = nil
		sibling.childIndex = 0
		self.root = sibling
		self._pool(parent)
		return
	}
	sibling.parent = grandParent
	grandParent.setChild(parent.childIndex, sibling)
	self._pool(parent)

	// fix data
	node := grandParent
	for node != nil {
		if self.strategy.balancingEnabled {
			node = self._balance(node)
		}
		node.computeHeight()
		node.computeAabb()
		node = node.parent
	}
}

// Balances and returns the node at the same position of `node`.
func (self *BvhTree) _balance(node *BvhNode) *BvhNode {
	nh := node.height
	if nh < 2 {
		return node
	}

	p := node.parent
	l := node.children[0]
	r := node.children[1]
	lh := l.height
	rh := r.height
	balance := lh - rh
	nodeIndex := node.childIndex

	//          [ N ]
	//         /     \
	//    [ L ]       [ R ]
	//     / \         / \
	// [L-L] [L-R] [R-L] [R-R]

	// is the tree balanced?
	if balance > 1 {
		ll := l.children[0]
		lr := l.children[1]
		llh := ll.height
		lrh := lr.height

		// is L-L highter tha L-R?
		if llh > lrh {
			// set N to L-R
			l.setChild(1, node)

			//          [ L ]
			//         /     \
			//    [L-L]       [ N ]
			//     / \         / \
			// [...] [...] [ L ] [ R ]

			// set L-R
			node.setChild(0, lr)

			//          [ L ]
			//         /     \
			//    [L-L]       [ N ]
			//     / \         / \
			// [...] [...] [L-R] [ R ]

			// fix bounds and heights
			l.computeAabb()
			l.computeHeight()
			node.computeAabb()
			node.computeHeight()
		} else {
			// set N to L-L
			l.setChild(0, node)

			//          [ L ]
			//         /     \
			//    [ N ]       [L-R]
			//     / \         / \
			// [ L ] [ R ] [...] [...]

			// set L-L
			node.setChild(0, ll)

			//          [ L ]
			//         /     \
			//    [ N ]       [L-R]
			//     / \         / \
			// [L-L] [ R ] [...] [...]

			// fix bounds and heights
			l.computeAabb()
			l.computeHeight()
			node.computeAabb()
			node.computeHeight()
		}
		// set new parent of L
		if p != nil {
			p.setChild(nodeIndex, l)
		} else {
			self.root = l
			l.parent = nil
		}
		return l
	}
	if balance < -1 {
		rl := r.children[0]
		rr := r.children[1]
		rlh := rl.height
		rrh := rr.height

		// is R-L higher than R-R?
		if rlh > rrh {
			// set N to R-R
			r.setChild(1, node)

			//          [ R ]
			//         /     \
			//    [R-L]       [ N ]
			//     / \         / \
			// [...] [...] [ L ] [ R ]

			// set R-R
			node.setChild(1, rr)

			//          [ R ]
			//         /     \
			//    [R-L]       [ N ]
			//     / \         / \
			// [...] [...] [ L ] [R-R]

			// fix bounds and heights
			r.computeAabb()
			r.computeHeight()
			node.computeAabb()
			node.computeHeight()
		} else {
			// set N to R-L
			r.setChild(0, node)

			//          [ R ]
			//         /     \
			//    [ N ]       [R-R]
			//     / \         / \
			// [ L ] [ R ] [...] [...]

			// set R-L
			node.setChild(1, rl)

			//          [ R ]
			//         /     \
			//    [ N ]       [R-R]
			//     / \         / \
			// [ L ] [R-L] [...] [...]

			// fix bounds and heights
			r.computeAabb()
			r.computeHeight()
			node.computeAabb()
			node.computeHeight()
		}
		// set new parent of R
		if p != nil {
			p.setChild(nodeIndex, r)
		} else {
			self.root = r
			r.parent = nil
		}
		return r
	}
	return node
}

func (self *BvhTree) _assertBeLeaf(leaf *BvhNode) {
	if debug.Debug {
		if leaf.proxy != nil && leaf.proxy.(*BvhProxy).Leaf == leaf && leaf.children[0] == nil && leaf.children[1] == nil && leaf.height == 0 {
			// all good
		} else {
			panic("Oimo asserts here")
		}
	}
}

func (self *BvhTree) _pool(node *BvhNode) {
	if debug.Debug {
		if node.proxy == nil || node.proxy.(*BvhProxy).Leaf == nil {
			// no problem
		} else {
			panic("Oimo asserts here")
		}
	}
	node.removeReferences()
	self.nodePool = SingleList_pool(self.nodePool, node)
}

func (self *BvhTree) _pick() *BvhNode {
	var bn *BvhNode
	self.nodePool, bn = SingleList_pick(self.nodePool, NewBvhNode)
	return bn
}

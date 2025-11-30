package demos

import "fmt"

//////////////////////////////////////////////// BvhStrategy
// (oimo/collision/broadphase/bvh/BvhStrategy.go)
// BVH strategy for BVH tree

type BvhStrategy struct {
	insertionStrategy BvhInsertionStrategy
	balancingEnabled  bool
}

func NewBvhStrategy() *BvhStrategy {
	return &BvhStrategy{
		insertionStrategy: _SIMPLE,
	}
}

// --- internal ---

// Returns the next step of leaf insertion. `0` or `1` to descend to corresponding child of current node. `-1` to stop descending and make common parent with current node.
func (self *BvhStrategy) decideInsertion(currentNode *BvhNode, leaf *BvhNode) int {
	switch self.insertionStrategy {
	case _SIMPLE:
		return self._decideInsertionSimple(currentNode, leaf)
	case _MINIMIZE_SURFACE_AREA:
		return self._decideInsertionMinimumSurfaceArea(currentNode, leaf)
	default:
		panic(fmt.Errorf("Invalid BVH insertion strategy: %v", self.insertionStrategy))
	}
}

// Sorts `leaves` and returns the split index `k` of the half-open interval [`from`, `until`). Leaves are separated into [`from`, `k`) and [`k`, `until`).
func (self *BvhStrategy) splitLeaves(leaves []*BvhNode, from int, until int) int {
	return self._splitLeavesMean(leaves, from, until)
}

// --- private ---

func (self *BvhStrategy) _decideInsertionSimple(currentNode *BvhNode, leaf *BvhNode) int {
	center := leaf.aabbMin.Add(leaf.aabbMax)

	c1 := currentNode.children[0]
	c2 := currentNode.children[1]

	diff1 := c1.aabbMin.Add(c1.aabbMax)
	diff2 := c2.aabbMin.Add(c2.aabbMax)
	diff1.SubEq(center)
	diff2.SubEq(center)

	dist1 := diff1.Dot(diff1)
	dist2 := diff2.Dot(diff2)

	if dist1 < dist2 {
		return 0
	} else {
		return 1
	}
}

func (self *BvhStrategy) _decideInsertionMinimumSurfaceArea(currentNode *BvhNode, leaf *BvhNode) int {
	c1 := currentNode.children[0]
	c2 := currentNode.children[1]

	oldArea := MathUtil.Aabb_surfaceArea(&currentNode.aabbMin, &currentNode.aabbMax)

	var combinedMin, combinedMax Vec3
	MathUtil.Aabb_combine(&combinedMin, &combinedMax, &currentNode.aabbMin, &currentNode.aabbMax, &leaf.aabbMin, &leaf.aabbMax)

	newArea := MathUtil.Aabb_surfaceArea(&combinedMin, &combinedMax)

	// cost of creating new pair with the node
	creatingCost := newArea * 2
	incrementalCost := (newArea - oldArea) * 2

	descendingCost1 := incrementalCost
	MathUtil.Aabb_combine(&combinedMin, &combinedMax, &c1.aabbMin, &c1.aabbMax, &leaf.aabbMin, &leaf.aabbMax)

	if c1.height == 0 {
		// leaf cost = area(combined aabb)
		descendingCost1 += MathUtil.Aabb_surfaceArea(&combinedMin, &combinedMax)
	} else {
		// node cost = area(combined aabb) - area(old aabb)
		descendingCost1 += MathUtil.Aabb_surfaceArea(&combinedMin, &combinedMax) - MathUtil.Aabb_surfaceArea(&c1.aabbMin, &c1.aabbMax)
	}

	descendingCost2 := incrementalCost
	MathUtil.Aabb_combine(&combinedMin, &combinedMax, &c2.aabbMin, &c2.aabbMax, &leaf.aabbMin, &leaf.aabbMax)
	if c2.height == 0 {
		// leaf cost = area(combined aabb)
		descendingCost2 += MathUtil.Aabb_surfaceArea(&combinedMin, &combinedMax)
	} else {
		// node cost = area(combined aabb) - area(old aabb)
		descendingCost2 += MathUtil.Aabb_surfaceArea(&combinedMin, &combinedMax) - MathUtil.Aabb_surfaceArea(&c2.aabbMin, &c2.aabbMax)
	}

	if creatingCost < descendingCost1 && creatingCost < descendingCost2 {
		return -1
	} else if descendingCost1 < creatingCost && descendingCost1 < descendingCost2 {
		return 0
	} else {
		return 1
	}
}

func (self *BvhStrategy) _splitLeavesMean(leaves []*BvhNode, from int, until int) int {
	invN := 1.0 / float64(until-from)

	// mean := sum(min + max) / n
	var centerMean Vec3
	for i := range until {
		leaf := leaves[i]
		leaf.tmp = leaf.aabbMax.Add(leaf.aabbMin)
		centerMean.AddEq(leaf.tmp)
	}
	centerMean.ScaleEq(invN)

	var variance Vec3
	for i := range until {
		leaf := leaves[i]
		diff := leaf.tmp.Sub(centerMean)
		diff.CompWiseMulEq(diff)
		variance.AddEq(diff)
	}

	// sort and split
	varx := variance.x
	vary := variance.y
	varz := variance.z

	l := from
	r := until - 1

	if varx > vary && varx > varz {
		mean := centerMean.x
		for {
			for {
				leaf := leaves[l]
				if leaf.tmp.x <= mean {
					break
				}
				l++
			}
			for {
				leaf := leaves[r]
				if leaf.tmp.x >= mean {
					break
				}
				r--
			}
			if l >= r {
				break
			}
			leaves[l], leaves[r] = leaves[r], leaves[l]
			l++
			r--
		}
	} else if vary > varx && vary > varz {
		mean := centerMean.y
		for {
			for {
				leaf := leaves[l]
				if leaf.tmp.y <= mean {
					break
				}
				l++
			}
			for {
				leaf := leaves[r]
				if leaf.tmp.y >= mean {
					break
				}
				r--
			}
			if l >= r {
				break
			}
			leaves[l], leaves[r] = leaves[r], leaves[l]
			l++
			r--
		}
	} else {
		mean := centerMean.z
		for {
			for {
				leaf := leaves[l]
				if leaf.tmp.z <= mean {
					break
				}
				l++
			}
			for {
				leaf := leaves[r]
				if leaf.tmp.z >= mean {
					break
				}
				r--
			}
			if l >= r {
				break
			}
			leaves[l], leaves[r] = leaves[r], leaves[l]
			l++
			r--
		}
	}

	return 1
}

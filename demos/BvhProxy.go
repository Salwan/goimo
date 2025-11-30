package demos

//////////////////////////////////////////////// BvhProxy
// (oimo/collision/broadphase/bvh/BvhProxy.go)
// Internal class.

type BvhProxy struct {
	*Proxy

	Leaf  *BvhNode
	Moved bool
}

func NewBvhProxy(userData any, id int) *BvhProxy {
	return &BvhProxy{
		Proxy: NewProxy(userData, id),
	}
}

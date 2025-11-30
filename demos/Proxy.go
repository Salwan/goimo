package demos

//////////////////////////////////////// Proxy
// (oimo/collision/broadphase/Proxy.go)
// A proxy is an object that can be added to a broad-phase collision detection algorithm. Users of the collision part of the library can move an axis-aligned bounding box of a proxy through `BroadPhase` class.

type Proxy struct {
	prev *Proxy
	next *Proxy

	// flattened aabb
	aabbMin Vec3
	aabbMax Vec3

	id int

	// Extra field that users can use for their own purposes. **Do not modify this property if you use the physics part of the library**, as the physics part of the library uses this property for connecting proxies and shapes of rigid bodies.
	userData any
}

func NewProxy(userData any, id int) *Proxy {
	np := &Proxy{
		id:       id,
		userData: userData,
	}
	return np
}

// --- double linked list interface ---

func (self *Proxy) GetNext() *Proxy {
	return self.next
}

func (self *Proxy) SetNext(x *Proxy) {
	self.next = x
}

func (self *Proxy) GetPrev() *Proxy {
	return self.prev
}

func (self *Proxy) SetPrev(x *Proxy) {
	self.prev = x
}

// --- internal ---

func (self *Proxy) setAabb(aabb Aabb) {
	self.aabbMin = aabb.Min
	self.aabbMax = aabb.Max
}

// --- public ---

// Returns the unique if of the proxy
func (self *Proxy) GetId() int {
	return self.id
}

// Returns the fat AABB of the proxy.
func (self *Proxy) GetFatAabb() Aabb {
	return Aabb{
		Min: self.aabbMin,
		Max: self.aabbMax,
	}
}

// Sets `aabb` to the fat AABB of the proxy.
// This does not create a new instance of `Aabb`.
func (self *Proxy) GetFatAabbTo(aabb *Aabb) {
	aabb.Min = self.aabbMin
	aabb.Max = self.aabbMax
}

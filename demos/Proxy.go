package demos

//////////////////////////////////////// Proxy
// (oimo/collision/broadphase/Proxy.go)
// A proxy is an object that can be added to a broad-phase collision detection algorithm. Users of the collision part of the library can move an axis-aligned bounding box of a proxy through `BroadPhase` class.

type IProxy interface {
	IDoubleLinkNode[IProxy]
	GetID() int
	GetAabbMin() *Vec3
	GetAabbMax() *Vec3
	GetUserData() any
}

type Proxy struct {
	prev IProxy
	next IProxy

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

func (self *Proxy) GetNext() IProxy {
	return self.next
}

func (self *Proxy) SetNext(x IProxy) {
	self.next = x
}

func (self *Proxy) GetPrev() IProxy {
	return self.prev
}

func (self *Proxy) SetPrev(x IProxy) {
	self.prev = x
}

// --- internal ---

func (self *Proxy) setAabb(aabb Aabb) {
	self.aabbMin = aabb.Min
	self.aabbMax = aabb.Max
}

// --- public ---

// Returns the unique if of the proxy
func (self *Proxy) GetID() int {
	return self.id
}

func (self *Proxy) GetAabbMin() *Vec3 {
	return &self.aabbMin
}

func (self *Proxy) GetAabbMax() *Vec3 {
	return &self.aabbMax
}

func (self *Proxy) GetUserData() any {
	return self.userData
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

package demos

//////////////////////////////////////// Proxy
// (oimo/collision/broadphase/Proxy.go)
// A proxy is an object that can be added to a broad-phase collision detection algorithm. Users of the collision part of the library can move an axis-aligned bounding box of a proxy through `BroadPhase` class.

type Proxy struct {
	prev *Proxy
	next *Proxy

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

// // --- internal ---

// extern public inline function _setAabb(aabb:Aabb):Void {
// 	M.vec3_assign(_aabbMin, aabb._min);
// 	M.vec3_assign(_aabbMax, aabb._max);
// }

// // --- public ---

// /**
// 	* Returns the unique id of the proxy.
// 	*/
// public function getId():Int {
// 	return _id;
// }

// /**
// 	* Returns the fat AABB of the proxy.
// 	*/
// public function getFatAabb():Aabb {
// 	var aabb:Aabb = new Aabb();
// 	M.vec3_assign(aabb._min, _aabbMin);
// 	M.vec3_assign(aabb._max, _aabbMax);
// 	return aabb;
// }

// /**
// 	* Sets `aabb` to the fat AABB of the proxy.
// 	*
// 	* This does not create a new instance of `Aabb`.
// 	*/
// public function getFatAabbTo(aabb:Aabb):Void {
// 	M.vec3_assign(aabb._min, _aabbMin);
// 	M.vec3_assign(aabb._max, _aabbMax);
// }

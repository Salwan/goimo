package demos

//////////////////////////////////////////////// Pool
// (oimo/common/Pool.hx)
// The object pool system of Vec3, Mat3, Mat4 and Quat

type Pool struct {
	stackVec3 []*Vec3
	sizeVec3  int
	stackMat3 []*Mat3
	sizeMat3  int
	stackMat4 []*Mat4
	sizeMat4  int
	stackQuat []*Quat
	sizeQuat  int
}

func NewPool() *Pool {
	return &Pool{
		stackVec3: make([]*Vec3, 256),
		stackMat3: make([]*Mat3, 256),
		stackMat4: make([]*Mat4, 256),
		stackQuat: make([]*Quat, 256),
	}
}

// --- private ---

func _getFromPool[T any](stack []*T, size *int) *T {
	if *size == 0 {
		return new(T)
	} else {
		*size--
		return stack[*size]
	}
}

func _disposeToPool[T any](stack *[]*T, size *int, obj *T) {
	if *size == len(*stack) {
		*stack = Array_expand(*stack)
	}
	*size++
	(*stack)[*size] = obj
}

// --- public ---

// Returns a `Vec3` object. If an unused object of `Vec3` is pooled, this does not create a new instance.
func (self *Pool) PickVec3() *Vec3 {
	return _getFromPool(self.stackVec3, &self.sizeVec3)
}

// Returns a `Mat3` object. If an unused object of `Mat3` is pooled, this does not create a new instance.
func (self *Pool) PickMat3() *Mat3 {
	return _getFromPool(self.stackMat3, &self.sizeMat3)
}

// Returns a `Mat4` object. If an unused object of `Mat4` is pooled, this does not create a new instance.
func (self *Pool) PickMat4() *Mat4 {
	return _getFromPool(self.stackMat4, &self.sizeMat4)
}

// Returns a `Quat` object. If an unused object of `Quat` is pooled, this does not create a new instance.
func (self *Pool) PickQuat() *Quat {
	return _getFromPool(self.stackQuat, &self.sizeQuat)
}

// Disposes an object got from `Pool.vec3`, `Pool.mat3`, `Pool.mat4`, or `Pool.quat`.
func (self *Pool) Dispose(vec3 *Vec3, mat3 *Mat3, mat4 *Mat4, quat *Quat) {
	if vec3 != nil {
		self.DisposeVec3(vec3)
	}
	if mat3 != nil {
		self.DisposeMat3(mat3)
	}
	if mat4 != nil {
		self.DisposeMat4(mat4)
	}
	if quat != nil {
		self.DisposeQuat(quat)
	}
}

// Disposes an `Vec3` object got from `Pool.vec3`.
func (self *Pool) DisposeVec3(v *Vec3) {
	v.Zero()
	_disposeToPool(&self.stackVec3, &self.sizeVec3, v)
}

// Disposes an `Mat3` object got from `Pool.mat3`.
func (self *Pool) DisposeMat3(m *Mat3) {
	m.Identity()
	_disposeToPool(&self.stackMat3, &self.sizeMat3, m)
}

// Disposes an `Mat4` object got from `Pool.mat4`.
func (self *Pool) DisposeMat4(m *Mat4) {
	m.Identity()
	_disposeToPool(&self.stackMat4, &self.sizeMat4, m)
}

// Disposes an `Quat` object got from `Pool.quat`.
func (self *Pool) DisposeQuat(q *Quat) {
	q.Identity()
	_disposeToPool(&self.stackQuat, &self.sizeQuat, q)
}

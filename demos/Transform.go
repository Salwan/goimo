package demos

////////////////////////////////// Transform
// (oimo/common/Transform.go)
// Transform class provides a set of translation and rotation.

type Transform struct {
	position Vec3
	rotation Mat3
}

func NewTransform() *Transform {
	t := Transform{}
	t.position.Zero()
	t.rotation.Identity()
	return &t
}

// Sets the transformation to identity and returns `this`.
func (t *Transform) Identity() *Transform {
	t.position.Zero()
	t.rotation.Identity()
	return t
}

// Returns the position of the transformation. (copy)
func (t *Transform) GetPosition() Vec3 {
	return t.position
}

// Sets `posOut` to the position of the transformation.
// This does not create a new instance of `Vec3`.
func (t *Transform) GetPositionTo(posOut *Vec3) {
	*posOut = t.position
}

// Sets the position of the transformation to `position` and returns `this`.
func (t *Transform) SetPosition(p Vec3) *Transform {
	t.position.x = p.x
	t.position.y = p.y
	t.position.z = p.z
	return t
}

// Translates the position by `translation`.
func (t *Transform) Translate(translation Vec3) {
	t.position.AddEq(translation)
}

// Returns the rotation matrix. (copy)
func (t *Transform) GetRotation() Mat3 {
	return t.rotation
}

// Sets `rotOut` to the rotation matrix.
// This does not create a new instance of `Mat3`.
func (t *Transform) GetRotationTo(rotOut *Mat3) {
	*rotOut = t.rotation
}

// Sets the rotation matrix to `rotation` and returns `this`.
func (t *Transform) SetRotation(rot Mat3) *Transform {
	t.rotation = rot
	return t
}

// TODO
// /**
// 	* Sets the rotation by Euler angles `eulerAngles` in radians.
// 	*/
// public inline function setRotationXyz(eulerAngles:Vec3):Void {
// 	var xyz:IVec3;
// 	M.vec3_fromVec3(xyz, eulerAngles);
// 	M.mat3_fromEulerXyz(_rotation, xyz);
// }

// /**
// 	* Applies rotation by the rotation matrix `rotation`.
// 	*/
// public inline function rotate(rotation:Mat3):Void {
// 	var rot:IMat3;
// 	M.mat3_fromMat3(rot, rotation);
// 	M.mat3_mul(_rotation, rot, _rotation);
// }

// /**
// 	* Applies the rotation by Euler angles `eulerAngles` in radians.
// 	*/
// public inline function rotateXyz(eulerAngles:Vec3):Void {
// 	var xyz:IVec3;
// 	var rot:IMat3;
// 	M.vec3_fromVec3(xyz, eulerAngles);
// 	M.mat3_fromEulerXyz(rot, xyz);
// 	M.mat3_mul(_rotation, rot, _rotation);
// }

// /**
// 	* Returns the rotation as a quaternion.
// 	*/
// public inline function getOrientation():Quat {
// 	var q:Quat = new Quat();
// 	var iq:IQuat;
// 	M.quat_fromMat3(iq, _rotation);
// 	M.quat_toQuat(q, iq);
// 	return q;
// }

// /**
// 	* Sets `orientation` to the quaternion representing the rotation.
// 	*
// 	* This does not create a new instance of `Quat`.
// 	*/
// public inline function getOrientationTo(orientation:Quat):Void {
// 	var iq:IQuat;
// 	M.quat_fromMat3(iq, _rotation);
// 	M.quat_toQuat(orientation, iq);
// }

// /**
// 	* Sets the rotation from a quaternion `quaternion` and returns `this`.
// 	*/
// public inline function setOrientation(quaternion:Quat):Transform {
// 	var q:IQuat;
// 	M.quat_fromQuat(q, quaternion);
// 	M.mat3_fromQuat(_rotation, q);
// 	return this;
// }

// /**
// 	* Returns a clone of the transformation.
// 	*/
// public inline function clone():Transform {
// 	var tf = new Transform();
// 	M.vec3_assign(tf._position, _position);
// 	M.mat3_assign(tf._rotation, _rotation);
// 	return tf;
// }

// /**
// 	* Sets the transformation to `transform` and returns `this`.
// 	*/
// public inline function copyFrom(transform:Transform):Transform {
// 	M.vec3_assign(_position, transform._position);
// 	M.mat3_assign(_rotation, transform._rotation);
// 	return this;
// }

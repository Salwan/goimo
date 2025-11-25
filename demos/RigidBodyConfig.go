package demos

// ////////////////////// RigidBodyConfig
// (oimo/dynamics/rigidbody/RigidBodyConfig.go)
// A rigid body configuration is used for constructions of rigid bodies. An instance of this class can safely be reused, as a rigid body will not have any references to a field of this class.

type RigidBodyConfig struct {
	Position                         Vec3          // The world position of the rigid body's center of gravity.
	Rotation                         Mat3          // The rotation matrix of the rigid body.
	LinearVelocity                   Vec3          // The initial value of the rigid body's linear velocity.
	AngularVelocity                  Vec3          // The initial value of the rigid body's angular velocity.
	Type                             RigidBodyType // The rigid body's motion type. See `RigidBodyType` for details.
	LinearDamping                    float64       // The damping coefficient of the linear velocity. Set positive values to gradually reduce the linear velocity.
	AngularDamping                   float64       // The damping coefficient of the angular velocity. Set positive values to gradually reduce the angular velocity.
	AutoSleep                        bool          // Whether to automatically sleep the rigid body when it stops moving for a certain period of time, namely `sleepingTimeThreshold`.
	SleepingVelocityThreshold        float64       // The linear velocity threshold to sleep the rigid body.
	SleepingAngularVelocityThreshold float64       // The angular velocity threshold to sleep the rigid body.
	SleepingTimeThreshold            float64       // The time threshold to sleep the rigid body.
}

func NewRigidBodyConfig() *RigidBodyConfig {
	rbc := &RigidBodyConfig{
		Type:                             _DYNAMIC,
		AutoSleep:                        true,
		SleepingVelocityThreshold:        Settings.SleepingVelocityThreshold,
		SleepingAngularVelocityThreshold: Settings.SleepingAngularVelocityThreshold,
		SleepingTimeThreshold:            Settings.SleepingTimeThreshold,
	}
	return rbc
}

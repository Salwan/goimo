package demos

/////////////////////// Settings (oimo/common/Settings.go)
// Settings provides advenced parameters used by the physics simulation.

type SettingsStruct struct {
	// default shape parameters
	DefaultFriction       float64
	DefaultRestitution    float64
	DefaultDensity        float64
	DefaultCollisionGroup int
	DefaultCollisionMask  int

	// velocity limitations
	MaxTranslationPerStep float64
	MaxRotationPerStep    float64

	// dynamic BVH
	BvhProxyPadding                  float64
	BvhIncrementalCollisionThreshold float64

	// GJK/EPA
	DefaultGJKMargin      float64
	EnableGJKCaching      bool
	MaxEPAVertices        int
	MaxEPAPolyhedronFaces int

	// general constraints
	ContactEnableBounceThreshold  float64
	VelocityBaumgarte             float64
	PositionSplitImpulseBaumgarte float64
	PositionNgsBaumgarte          float64

	// contacts
	ContactUseAlternativePositionCorrectionAlgorithmDepthThreshold float64
	DefaultContactPositionCorrectionAlgorithm                      PositionCorrectionAlgorithm
	AlternativeContactPositionCorrectionAlgorithm                  PositionCorrectionAlgorithm
	ContactPersistenceThreshold                                    float64
	MaxManifoldPoints                                              int

	// joints
	DefaultJointConstraintSolverType        ConstraintSolverType
	DefaultJointPositionCorrectionAlgorithm PositionCorrectionAlgorithm
	JointWarmStartingFactorForBaungarte     float64
	JointWarmStartingFactor                 float64
	MinSpringDamperDampingRatio             float64
	MinRagdollMaxSwingAngle                 float64
	MaxJacobianRows                         int

	// direct MLCP solver
	DirectMlcpSolverEps float64

	// islands
	IslandInitialRigidBodyArraySize  int
	IslandInitialConstraintArraySize int

	// sleeping
	SleepingVelocityThreshold        float64
	SleepingAngularVelocityThreshold float64
	SleepingTimeThreshold            float64
	DisableSleeping                  bool

	// slops
	LinearSlop  float64
	AngularSlop float64
}

var Settings = SettingsStruct{
	// default shape parameters
	DefaultFriction:       0.2,
	DefaultRestitution:    0.2,
	DefaultDensity:        1.0,
	DefaultCollisionGroup: 1,
	DefaultCollisionMask:  1,

	// velocity limitations
	MaxTranslationPerStep: 20.0,
	MaxRotationPerStep:    MathUtil.PI,

	// dynamic BVH
	BvhProxyPadding:                  0.1,
	BvhIncrementalCollisionThreshold: 0.45,

	// GJK/EPA
	DefaultGJKMargin:      0.05,
	EnableGJKCaching:      true,
	MaxEPAVertices:        128,
	MaxEPAPolyhedronFaces: 128,

	// general constraints
	ContactEnableBounceThreshold:  0.5,
	VelocityBaumgarte:             0.2,
	PositionSplitImpulseBaumgarte: 0.4,
	PositionNgsBaumgarte:          1.0,

	// contacts
	ContactUseAlternativePositionCorrectionAlgorithmDepthThreshold: 0.05,
	DefaultContactPositionCorrectionAlgorithm:                      _BAUMGARTE,
	AlternativeContactPositionCorrectionAlgorithm:                  _SPLIT_IMPULSE,
	ContactPersistenceThreshold:                                    0.05,
	MaxManifoldPoints:                                              4,

	// joints
	DefaultJointConstraintSolverType:        _ITERATIVE,
	DefaultJointPositionCorrectionAlgorithm: _BAUMGARTE,
	JointWarmStartingFactorForBaungarte:     0.8,
	JointWarmStartingFactor:                 0.95,
	MinSpringDamperDampingRatio:             1e-6,
	MinRagdollMaxSwingAngle:                 1e-6,
	MaxJacobianRows:                         6,

	// direct MLCP solver
	DirectMlcpSolverEps: 1e-9,

	// islands
	IslandInitialRigidBodyArraySize:  128,
	IslandInitialConstraintArraySize: 128,

	// sleeping, some are just default values and can be changed through RigidBodyConfig
	SleepingVelocityThreshold:        0.2,
	SleepingAngularVelocityThreshold: 0.5,
	SleepingTimeThreshold:            1.0,
	DisableSleeping:                  false,

	// slops
	LinearSlop:  0.005,
	AngularSlop: 1 * MathUtil.TO_RADIANS,
}

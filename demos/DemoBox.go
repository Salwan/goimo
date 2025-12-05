package demos

import (
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
)

// A box that follows a physical rigidbody box

// OimoPhysics colors
var (
	shapeColor1          = &math32.Color{R: 0.7, G: 0.2, B: 0.4}
	shapeColor2          = &math32.Color{R: 1.0, G: 0.8, B: 0.1}
	sleepyShapeColor1    = &math32.Color{R: 0.5, G: 0.25, B: 0.6}
	sleepyShapeColor2    = &math32.Color{R: 0.6, G: 0.8, B: 0.3}
	sleepingShapeColor1  = &math32.Color{R: 0.3, G: 0.3, B: 0.8}
	sleepingShapeColor2  = &math32.Color{R: 0.2, G: 0.8, B: 0.5}
	staticShapeColor     = &math32.Color{R: 0.7, G: 0.7, B: 0.7}
	kinematicShapeColor  = &math32.Color{R: 1.0, G: 0.5, B: 0.1}
	aabbColor            = &math32.Color{R: 1.0, G: 0.1, B: 0.1}
	bvhNodeColor         = &math32.Color{R: 0.4, G: 0.4, B: 0.4}
	pairColor            = &math32.Color{R: 1.0, G: 1.0, B: 0.1}
	contactColor         = &math32.Color{R: 1.0, G: 0.1, B: 0.1}
	contactColor2        = &math32.Color{R: 1.0, G: 0.6, B: 0.1}
	contactColor3        = &math32.Color{R: 0.1, G: 0.8, B: 0.6}
	contactColor4        = &math32.Color{R: 0.8, G: 0.1, B: 1.0}
	newContactColor      = &math32.Color{R: 1.0, G: 1.0, B: 0.1}
	disabledContactColor = &math32.Color{R: 0.5, G: 0.1, B: 0.1}
	contactNormalColor   = &math32.Color{R: 1.0, G: 0.1, B: 0.1}
	contactTangentColor  = &math32.Color{R: 0.1, G: 0.8, B: 0.1}
	contactBinormalColor = &math32.Color{R: 0.2, G: 0.2, B: 1.0}
	jointLineColor       = &math32.Color{R: 0.8, G: 0.8, B: 0.8}
	jointErrorColor      = &math32.Color{R: 1.0, G: 0.1, B: 0.1}
	basisColorX          = &math32.Color{R: 1.0, G: 0.0, B: 0.0}
	basisColorY          = &math32.Color{R: 0.0, G: 1.0, B: 0.0}
	basisColorZ          = &math32.Color{R: 0.0, G: 0.0, B: 1.0}

	contactNormalLength             = float32(0.5)
	contactTangentLength            = float32(0.5)
	contactBinormalLength           = float32(0.5)
	jointRotationalConstraintRadius = float32(0.3)
	basisLength                     = float32(0.5)
)

var nextDynamicColor int = 0

type DemoBox struct {
	*core.Node
	rb *RigidBody
}

func NewDemoBox(rb *RigidBody, pos, hsize math32.Vector3) *DemoBox {
	var color *math32.Color
	switch rb._type {
	case RigidBodyType_DYNAMIC:
		// OimoPhysics uses shape id number to alternate colors
		nextDynamicColor++
		if nextDynamicColor%2 == 0 {
			color = shapeColor1
		} else {
			color = shapeColor2
		}
	case RigidBodyType_KINEMATIC:
		color = kinematicShapeColor
	case RigidBodyType_STATIC:
		color = staticShapeColor
	}

	d := &DemoBox{
		Node: core.NewNode(),
		rb:   rb,
	}
	box_geom := geometry.NewBox(hsize.X*2, hsize.Y*2, hsize.Z*2)
	box_mat := material.NewStandard(color)
	box := graphic.NewMesh(box_geom, box_mat)
	d.Add(box)

	return d
}

func convVec3ToVector3(v *Vec3) math32.Vector3 {
	return math32.Vector3{X: float32(v.x), Y: float32(v.y), Z: float32(v.z)}
}

// Synchronizes box with physical rigidbody
func (db *DemoBox) sync() {
	p := convVec3ToVector3(&db.rb.transform.position)
	var r Quat
	r.fromMat3(&db.rb.transform.rotation)
	db.SetPositionVec(&p)
	db.SetRotationQuat(&math32.Quaternion{X: float32(r.x), Y: float32(r.y), Z: float32(r.z), W: float32(r.w)})
}

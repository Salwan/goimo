package demos

import (
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
)

// A box that follows a physical rigidbody box

type DemoBox struct {
	*core.Node
	rb *RigidBody
}

func NewDemoBox(rb *RigidBody, pos, hsize math32.Vector3) *DemoBox {
	d := &DemoBox{
		Node: core.NewNode(),
		rb:   rb,
	}
	box_geom := geometry.NewBox(hsize.X*2, hsize.Y*2, hsize.Z*2)
	box_mat := material.NewStandard(&math32.Color{R: 0.5, G: 0.5, B: 0.6})
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
	//r := db.rb.transform.rotation
	db.SetPositionVec(&p)
}

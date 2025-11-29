package demos

import (
	"time"

	"github.com/g3n/engine/app"
	"github.com/g3n/engine/camera"
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/gls"
	"github.com/g3n/engine/gui"
	"github.com/g3n/engine/light"
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/renderer"
	"github.com/g3n/engine/util/helper"
	"github.com/g3n/engine/window"
)

// OimoPhysics demo box.

// Next TODO:
// - Create and init World instance before demo construction
// - Implement DemoBase.init() at start of initBasicDemo()

type DemoMain struct {
	application *app.Application
	gs          *gls.GLS
	root        *core.Node
	width       int
	height      int
	keyState    *window.KeyState
	cam         *camera.Camera
	world       *World
	dt          float64
	demoBoxes   []*DemoBox
}

func NewDemoMain() *DemoMain {
	dm := DemoMain{}
	dm.application = app.App(1280, 720, "Goimo Demos: Base Demo")
	dm.gs = dm.application.Gls()
	dm.width, dm.height = dm.application.GetSize()
	dm.root = core.NewNode()
	gui.Manager().Set(dm.root)
	dm.keyState = dm.application.KeyState()

	dm.cam = camera.New(float32(dm.width) / float32(dm.height))
	dm.cam.SetPosition(0, 5, 10)
	dm.cam.LookAt(math32.NewVector3(0, 0, 0), math32.NewVector3(0, 1, 0))
	dm.cam.SetNear(0.1)
	dm.cam.SetFar(1000.0)
	dm.root.Add(dm.cam)

	dm.gs.ClearColor(0.1, 0.1, 0.1, 1.0)
	dm.root.Add(helper.NewAxes(0.5))
	dm.root.Add(helper.NewGrid(10.0, 1.0, &math32.Color{R: 0.2, G: 0.2, B: 0.2}))

	dirLight := light.NewDirectional(&math32.Color{R: 1, G: 1, B: 1}, 1.0)
	dirLight.SetPosition(0.2, 0.8, 0.7)
	dm.root.Add(dirLight)

	// App events
	onResize := func(evname string, ev interface{}) {
		dm.width, dm.height = dm.application.GetSize()
		aspect_ratio := float32(dm.width) / float32(dm.height)
		dm.gs.Viewport(0, 0, int32(dm.width), int32(dm.height))
		dm.cam.SetAspect(aspect_ratio)
		dm.cam.SetFov(60)
	}
	dm.application.Subscribe(window.OnWindowSize, onResize)
	onResize("", nil)

	dm.world = NewWorld(_BVH, nil)

	// Init demos
	dm.initBaseDemo()
	dm.initBasicDemo()

	// TEMP: orbit camera to see what's going on
	camera.NewOrbitControl(dm.cam)

	return &dm
}

func (dm *DemoMain) initBaseDemo() {
	dm.cam.SetPosition(0, 5, 10)
	dm.cam.LookAt(math32.NewVector3(0, 0, 0), math32.NewVector3(0, 1, 0))
	dm.dt = 1 / 60.0
}

func (dm *DemoMain) initBasicDemo() {
	dm.cam.SetPosition(0, 7, 9)
	dm.cam.LookAt(math32.NewVector3(0, 2, 0), math32.NewVector3(0, 1, 0))

	thickness := 0.5
	_, gground := OimoUtil.AddBox(dm.world, &Vec3{0, -thickness, 0},
		&Vec3{7, thickness, 7}, true)
	dm.root.Add(gground)
	dm.demoBoxes = append(dm.demoBoxes, gground)

	w, h, n := 2, 2, 5
	sp, size := 0.61, 0.3
	for i := range n {
		for j := -w; j <= w+1; j++ {
			for k := -h; k <= h+1; k++ {
				pos := Vec3{float64(j) * sp, size + float64(i)*size*3, float64(k) * sp}
				box, gbox := OimoUtil.AddBox(dm.world, &pos, &Vec3{size, size, size}, false)
				box.SetAngularVelocity(MathUtil.RandVec3In(-0.05, 0.05))
				dm.root.Add(gbox)
				dm.demoBoxes = append(dm.demoBoxes, gbox)
			}
		}
	}
}

func (dm *DemoMain) Run() {
	dm.application.Run(func(render *renderer.Renderer, deltaTime time.Duration) {
		dt := float32(deltaTime.Seconds())
		dm.Update(dt)
		dm.Render(render)
	})
}

func (dm *DemoMain) Update(dt float32) {
	if dm.keyState.Pressed(window.KeyQ) {
		dm.application.Exit()
	}

	// synchronize boxes
	for _, b := range dm.demoBoxes {
		b.sync()
	}

	// DemoBase: update() doesnt run when in demo
	// DemoMain: loop() runs at 60fps (called from javascript)
	// BasicDemo (currentDemo): nothing relevant
	// currentDemo.update(): nothing relevant
	dm.world.step(dm.dt)
}

func (dm *DemoMain) Render(render *renderer.Renderer) {
	dm.gs.Clear(gls.DEPTH_BUFFER_BIT | gls.STENCIL_BUFFER_BIT | gls.COLOR_BUFFER_BIT)
	render.Render(dm.root, dm.cam)
}

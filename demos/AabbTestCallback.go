package demos

//////////////////////////////////////////////// AabbTestCallback
// (oimo/dynamics/callback/AabbTestCallback.go)
// A callback interface for aabb tests in a world.

type IAabbTestCallback interface {
	Process(shape *Shape)
}

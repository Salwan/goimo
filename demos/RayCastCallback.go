package demos

//////////////////////////////////////////////// RayCastCallback
// (oimo/dynamics/callback/RayCastCallback.go)
// A callback class for ray casts in a world.

type IRayCastCallback interface {
	// This is called every time the world detects a shape `shape` that the ray intersects with the hit data `hit`.
	Process(shape *Shape, hit *RayCastHit)
}

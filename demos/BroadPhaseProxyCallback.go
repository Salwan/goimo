package demos

//////////////////////////////////////////////// BroadPhaseProxyCallback
// (oimo/collision/broadphase/BroadPhaseProxyCallback.go)
// A callback class for queries in a broad phase.

type IBroadPhaseProxyCallback interface {
	// This is called every time a broad phase algorithm reports a proxy `proxy`.
	Process(proxy IProxy)
}

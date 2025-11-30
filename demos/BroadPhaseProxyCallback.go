package demos

//////////////////////////////////////////////// BroadPhaseProxyCallback
// (oimo/collision/broadphase/BroadPhaseProxyCallback.go)
// A callback class for queries in a broad phase.

type BroadPhaseProxyCallback struct{}

func NewBroadPhaseProxyCallback() *BroadPhaseProxyCallback {
	return &BroadPhaseProxyCallback{}
}

// This is called every time a broad phase algorithm reports a proxy `proxy`.
func (self *BroadPhaseProxyCallback) process(proxy *Proxy) {}

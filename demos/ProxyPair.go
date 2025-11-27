package demos

///////////////////////////////////// ProxyPair
// (oimo/collision/broadphase/ProxyPair.go)
// A pair between two proxies. Broad-phase collision algorithms collect pairs of proxies as linked list of ProxyPair.

type ProxyPair struct {
	next *ProxyPair

	p1 *Proxy
	p2 *Proxy
}

func NewProxyPair() *ProxyPair {
	return &ProxyPair{}
}

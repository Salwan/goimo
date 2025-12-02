package demos

///////////////////////////////////// ProxyPair
// (oimo/collision/broadphase/ProxyPair.go)
// A pair between two proxies. Broad-phase collision algorithms collect pairs of proxies as linked list of ProxyPair.

type ProxyPair struct {
	next *ProxyPair

	p1 IProxy
	p2 IProxy
}

func NewProxyPair() *ProxyPair {
	return &ProxyPair{}
}

func (pp *ProxyPair) GetNext() *ProxyPair {
	return pp.next
}

func (pp *ProxyPair) SetNext(next *ProxyPair) {
	pp.next = next
}

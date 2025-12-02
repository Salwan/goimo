package demos

////////////////////////////////// ListUtils
// (?)
// Utilities to do single and double linked list on Oimo types via next and prev pointers.

// Current Go has hard limits on generic constraints:
// - generics cannot describe properties in any way, only methods via interface, (proposal: https://github.com/golang/go/issues/51259)
// - generics cannot describe a nullable constraint.
// - generics cannot describe a pointer constraint. (proposal: https://github.com/golang/go/issues/70960)

type ISingleLinkNode[N any] interface {
	GetNext() N
	SetNext(N)
}

//////////////////////////////////////// Single List

// add to front: head is *N (so if N is *MyNode, head is **MyNode)
func SingleList_addFirst[N interface {
	ISingleLinkNode[N]
	comparable
}](head *N, n N) {
	var zero N
	if *head == zero {
		*head = n
	} else {
		n.SetNext(*head)
		*head = n
	}
}

// Pop next available element from pool, or return nil if no elements
func SingleList_pick[N interface {
	ISingleLinkNode[N]
	comparable
}](head *N, creator func() N) N {
	var zero N
	if *head == zero {
		*head = creator()
	}
	n := *head
	*head = n.GetNext()
	n.SetNext(zero)
	return n
}

// Push the node 'n' onto the front of list
func SingleList_pool[N interface {
	ISingleLinkNode[N]
	comparable
}](head *N, n N) {
	n.SetNext(*head)
	*head = n
}

///////////////////////////////////////////// Double List Interface
// Special case of DoubleList_* that works for interfaces that are also adhoc double lists. It's critical to remember that these methods unlike normal ones return new (head, tail) that must be set otherwise the call won't work!
// TODO: unify all DoubleList and SingleList calls to return new_head/new_tail to eliminate this special case implementation

type IDoubleLinkINode[IN any] interface {
	GetINext() IN
	SetINext(x IN)
	GetIPrev() IN
	SetIPrev(x IN)
}

// Push interface node to end of list. Remember that it returns new (head, tail) that must be set
func DoubleListInterface_push[IN interface {
	IDoubleLinkINode[IN]
	comparable
}](head IN, tail IN, n IN) (out_head, out_tail IN) {
	var zero IN
	out_head = head
	out_tail = tail
	if head == zero {
		out_head = n
		out_tail = n
	} else {
		tail.SetINext(n)
		n.SetIPrev(tail)
		out_tail = n
	}
	return
}

// Remove given interface node from list. Remember that it returns new (head, tail) that must be set
func DoubleListInterface_remove[IN interface {
	IDoubleLinkINode[IN]
	comparable
}](head IN, tail IN, n IN) (out_head, out_tail IN) {
	var zero IN
	out_head = head
	out_tail = tail
	prev := n.GetIPrev() // must return an interface node
	next := n.GetINext() // =
	if prev != zero {
		prev.SetINext(next)
	}
	if next != zero {
		next.SetIPrev(prev)
	}
	if n == head {
		out_head = head.GetINext()
	}
	if n == tail {
		out_tail = tail.GetIPrev()
	}
	n.SetINext(zero)
	n.SetIPrev(zero)
	return
}

///////////////////////////////////////////// Double List

type IDoubleLinkNode[N any] interface {
	ISingleLinkNode[N]
	GetPrev() N
	SetPrev(N)
}

// Visit all nodes in the list calling callback(n) on each
func DoubleList_foreach[N interface {
	IDoubleLinkNode[N]
	comparable
}](head *N, callback func(n N)) {
	list := *head
	var zero N
	for n := list; n != zero; n = n.GetNext() {
		callback(n)
	}
}

// Push node to end of list
func DoubleList_push[N interface {
	IDoubleLinkNode[N]
	comparable
}](head *N, tail *N, n N) {
	var zero N
	if *head == zero {
		*head = n
		*tail = *head
	} else {
		(*tail).SetNext(n)
		n.SetPrev(*tail)
		*tail = n
	}
}

// Push node to begining of list
func DoubleList_addFirst[N interface {
	IDoubleLinkNode[N]
	comparable
}](head *N, tail *N, n N) {
	var zero N
	if *head == zero {
		*head = n
		*tail = n
	} else {
		(*head).SetPrev(n)
		n.SetNext(*head)
		*head = n
	}
}

// Remove given node from list
func DoubleList_remove[N interface {
	IDoubleLinkNode[N]
	comparable
}](head *N, tail *N, n N) {
	var zero N
	prev := n.GetPrev()
	next := n.GetNext()
	if prev != zero {
		prev.SetNext(next)
	}
	if next != zero {
		next.SetPrev(prev)
	}
	if n == *head {
		*head = (*head).GetNext()
	}
	if n == *tail {
		*tail = (*tail).GetPrev()
	}
	n.SetNext(zero)
	n.SetPrev(zero)
}

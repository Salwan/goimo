package demos

////////////////////////////////// ListUtils
// (?)
// Utilities to do single and double linked list on Oimo types via next and prev pointers.

// Current Go has hard limits on generic constraints:
// - generics cannot describe properties in any way, only methods via interface, Theres a proposal but looks far in the future: https://github.com/golang/go/issues/51259
// - generics cannot describe a nullable constraint.
// - generics cannot describe a pointer constraint.

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
	n.SetNext(*head)
	*head = n
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

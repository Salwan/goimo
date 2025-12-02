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

// add to front: head is *N (so if N is *MyNode, head is **MyNode). Returns new head to be set.
func SingleList_addFirst[N interface {
	ISingleLinkNode[N]
	comparable
}](head N, n N) (new_head N) {
	var zero N
	new_head = head
	if head == zero {
		new_head = n
	} else {
		n.SetNext(head)
		new_head = n
	}
	return
}

// Pop next available element from pool, or return new head and new element/nil if no elements
func SingleList_pick[N interface {
	ISingleLinkNode[N]
	comparable
}](head N, creator func() N) (new_head, new_elem N) {
	var zero N
	if head == zero {
		head = creator()
	}
	new_head = head
	n := head
	new_head = n.GetNext()
	n.SetNext(zero)
	new_elem = n
	return
}

// Push the node 'n' onto the front of list. Returns new head
func SingleList_pool[N interface {
	ISingleLinkNode[N]
	comparable
}](head N, n N) (new_head N) {
	n.SetNext(head)
	new_head = n
	return
}

///////////////////////////////////////////// Double List

type IDoubleLinkNode[N any] interface {
	ISingleLinkNode[N]
	GetPrev() N
	SetPrev(N)
}

// Visit all nodes in the list calling callback(n) on each. (unused)
func DoubleList_foreach[N interface {
	IDoubleLinkNode[N]
	comparable
}](head N, callback func(n N)) {
	list := head
	var zero N
	for n := list; n != zero; n = n.GetNext() {
		callback(n)
	}
}

// Push node to end of list. Returns new (head, tail) that must be set
func DoubleList_push[N interface {
	IDoubleLinkNode[N]
	comparable
}](head N, tail N, n N) (new_head, new_tail N) {
	var zero N
	new_head = head
	new_tail = tail
	if head == zero {
		new_head = n
		new_tail = n
	} else {
		tail.SetNext(n)
		n.SetPrev(tail)
		new_tail = n
	}
	return
}

// Push node to begining of list. Returns new (head, tail) that must be set
func DoubleList_addFirst[N interface {
	IDoubleLinkNode[N]
	comparable
}](head N, tail N, n N) (new_head, new_tail N) {
	var zero N
	new_head = head
	new_tail = tail
	if head == zero {
		new_head = n
		new_tail = n
	} else {
		head.SetPrev(n)
		n.SetNext(head)
		new_head = n
	}
	return
}

// Remove given node from list. Returns new (head, tail) that must be set
func DoubleList_remove[N interface {
	IDoubleLinkNode[N]
	comparable
}](head N, tail N, n N) (new_head, new_tail N) {
	var zero N
	new_head = head
	new_tail = tail
	prev := n.GetPrev()
	next := n.GetNext()
	if prev != zero {
		prev.SetNext(next)
	}
	if next != zero {
		next.SetPrev(prev)
	}
	if n == head {
		new_head = head.GetNext()
	}
	if n == tail {
		new_tail = tail.GetPrev()
	}
	n.SetNext(zero)
	n.SetPrev(zero)
	return
}

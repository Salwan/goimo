package demos

////////////////////////////////// ListUtils
// (?)
// Utilities to do single and double linked list on Oimo types via next and prev pointers.

// Current Go has hard limits on generic constraints:
// - generics cannot describe properties in any way, only methods via interface, There's a proposal but looks far in the future: https://github.com/golang/go/issues/51259
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

// TODO

// list_foreach(base:Expr, next:Expr, loop:Expr)

// list_push(first:Expr, last:Expr, prev:Expr, next:Expr, e:Expr)

// list_addFirst(first:Expr, last:Expr, prev:Expr, next:Expr, e:Expr)

// list_remove(first:Expr, last:Expr, prev:Expr, next:Expr, e:Expr)

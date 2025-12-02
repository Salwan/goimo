package demos

// Test Single and Double linked lists
// LLM

import (
	"testing"
)

// --- Single List Tests ---

type MyNode struct {
	Next *MyNode
	ID   int
}

func (n *MyNode) GetNext() *MyNode  { return n.Next }
func (n *MyNode) SetNext(x *MyNode) { n.Next = x }

func TestSingleList_addFirst(t *testing.T) {
	var head *MyNode

	a := &MyNode{ID: 1}
	b := &MyNode{ID: 2}
	c := &MyNode{ID: 3}

	// Push in sequence: a, b, c --> list becomes: c -> b -> a
	head = SingleList_addFirst(head, a)
	if head != a {
		t.Fatalf("expected head=a; got %v", head)
	}
	if a.Next != nil {
		t.Fatalf("expected a.Next=nil; got %v", a.Next)
	}

	head = SingleList_addFirst(head, b)
	if head != b {
		t.Fatalf("expected head=b; got %v", head)
	}
	if b.Next != a {
		t.Fatalf("expected b.Next=a; got %v", b.Next)
	}

	head = SingleList_addFirst(head, c)
	if head != c {
		t.Fatalf("expected head=c; got %v", head)
	}
	if c.Next != b {
		t.Fatalf("expected c.Next=b; got %v", c.Next)
	}
}

func TestSingleList_pick_FromExisting(t *testing.T) {
	var head *MyNode

	a := &MyNode{ID: 10}
	b := &MyNode{ID: 20}
	c := &MyNode{ID: 30}

	// Build list c -> b -> a
	head = SingleList_addFirst(head, a)
	head = SingleList_addFirst(head, b)
	head = SingleList_addFirst(head, c)

	// Pick should remove c first
	head, p := SingleList_pick(head, func() *MyNode {
		t.Fatal("creator should not be called here")
		return nil
	})
	if p != c {
		t.Fatalf("expected pick=c; got %v", p)
	}
	if head != b {
		t.Fatalf("expected new head=b; got %v", head)
	}
	if p.Next != nil {
		t.Fatalf("expected p.Next=nil after clearing; got %v", p.Next)
	}

	// Next pick → b
	head, p = SingleList_pick(head, func() *MyNode { t.Fatal("creator called"); return nil })
	if p != b {
		t.Fatalf("expected pick=b; got %v", p)
	}
	if head != a {
		t.Fatalf("expected new head=a; got %v", head)
	}

	// Next pick → a
	head, p = SingleList_pick(head, func() *MyNode { t.Fatal("creator called"); return nil })
	if p != a {
		t.Fatalf("expected pick=a; got %v", p)
	}
	if head != nil {
		t.Fatalf("expected head=nil after last element; got %v", head)
	}
}

func TestSingleList_pick_UsesCreator(t *testing.T) {
	var head *MyNode

	created := 0
	creator := func() *MyNode {
		created++
		return &MyNode{ID: 999}
	}

	// First pick should create because list is empty
	head, p := SingleList_pick(head, creator)
	if created != 1 {
		t.Fatalf("expected creator called once; got %d", created)
	}
	if p.ID != 999 {
		t.Fatalf("expected ID=999; got %d", p.ID)
	}

	// After picking, list should be empty again
	if head != nil {
		t.Fatalf("expected head=nil; got %v", head)
	}
}

func TestSingleList_pool(t *testing.T) {
	var head *MyNode

	a := &MyNode{ID: 1}
	b := &MyNode{ID: 2}
	c := &MyNode{ID: 3}

	// Push a → list: a
	head = SingleList_pool(head, a)
	if head != a {
		t.Fatalf("expected head=a; got %v", head)
	}
	if a.Next != nil {
		t.Fatalf("expected a.Next=nil; got %v", a.Next)
	}

	// Push b → list: b -> a
	head = SingleList_pool(head, b)
	if head != b {
		t.Fatalf("expected head=b; got %v", head)
	}
	if b.Next != a {
		t.Fatalf("expected b.Next=a; got %v", b.Next)
	}

	// Push c → list: c -> b -> a
	head = SingleList_pool(head, c)
	if head != c {
		t.Fatalf("expected head=c; got %v", head)
	}
	if c.Next != b {
		t.Fatalf("expected c.Next=b; got %v", c.Next)
	}
}

// --- Double List Tests ---

type MyDoubleNode struct {
	Next *MyDoubleNode
	Prev *MyDoubleNode
	ID   int
}

func (n *MyDoubleNode) GetNext() *MyDoubleNode  { return n.Next }
func (n *MyDoubleNode) SetNext(x *MyDoubleNode) { n.Next = x }
func (n *MyDoubleNode) GetPrev() *MyDoubleNode  { return n.Prev }
func (n *MyDoubleNode) SetPrev(x *MyDoubleNode) { n.Prev = x }

func TestDoubleList(t *testing.T) {
	var first *MyDoubleNode
	var last *MyDoubleNode

	a := &MyDoubleNode{ID: 1}
	b := &MyDoubleNode{ID: 2}
	c := &MyDoubleNode{ID: 3}

	t.Run("push", func(t *testing.T) {
		// Push a, b, c -> a <-> b <-> c
		first, last = DoubleList_push(first, last, a)
		if first != a || last != a {
			t.Fatalf("push [a] failed, first=%v, last=%v", first, last)
		}
		if a.Prev != nil || a.Next != nil {
			t.Fatal("[a] links wrong")
		}

		first, last = DoubleList_push(first, last, b)
		if first != a || last != b {
			t.Fatal("push b failed")
		}
		if a.Next != b || b.Prev != a || b.Next != nil {
			t.Fatal("b links wrong")
		}

		first, last = DoubleList_push(first, last, c)
		if first != a || last != c {
			t.Fatal("push c failed")
		}
		if b.Next != c || c.Prev != b || c.Next != nil {
			t.Fatal("c links wrong")
		}
	})

	t.Run("foreach", func(t *testing.T) {
		ids := []int{}
		DoubleList_foreach(first, func(n *MyDoubleNode) {
			ids = append(ids, n.ID)
		})
		if ids[0] != 1 || ids[1] != 2 || ids[2] != 3 {
			t.Fatalf("foreach failed, got %v", ids)
		}
	})

	t.Run("remove", func(t *testing.T) {
		// list is a <-> b <-> c
		// remove b -> a <-> c
		first, last = DoubleList_remove(first, last, b)
		if first != a || last != c {
			t.Fatal("remove b failed")
		}
		if a.Next != c || c.Prev != a {
			t.Fatal("remove b links wrong")
		}
		if b.Prev != nil || b.Next != nil {
			t.Fatal("b should be unlinked")
		}

		// remove a -> c
		first, last = DoubleList_remove(first, last, a)
		if first != c || last != c {
			t.Fatal("remove a failed")
		}
		if c.Prev != nil || c.Next != nil {
			t.Fatal("c links wrong after removing a")
		}

		// remove c -> empty
		first, last = DoubleList_remove(first, last, c)
		if first != nil || last != nil {
			t.Fatal("remove c failed")
		}
	})

	t.Run("addFirst", func(t *testing.T) {
		// list is empty
		d := &MyDoubleNode{ID: 4}
		e := &MyDoubleNode{ID: 5}

		first, last = DoubleList_addFirst(first, last, d)
		if first != d || last != d {
			t.Fatal("addFirst d failed")
		}

		first, last = DoubleList_addFirst(first, last, e)
		if first != e || last != d {
			t.Fatal("addFirst e failed")
		}
		if e.Next != d || d.Prev != e {
			t.Fatal("addFirst e links wrong")
		}
	})
}

/////////////////////////////////// Double List Interface

type IMyINode interface {
	IDoubleLinkINode[IMyINode]
	GetID() int
}

type MyINode struct {
	Next IMyINode
	Prev IMyINode
	ID   int
}

func (n *MyINode) GetINext() IMyINode  { return n.Next }
func (n *MyINode) SetINext(x IMyINode) { n.Next = x }
func (n *MyINode) GetIPrev() IMyINode  { return n.Prev }
func (n *MyINode) SetIPrev(x IMyINode) { n.Prev = x }
func (n *MyINode) GetID() int          { return n.ID }

// TestDoubleList_Interface tests the interface-based double linked list functions.
func TestDoubleList_Interface(t *testing.T) {
	t.Run("push", func(t *testing.T) {
		var zero IMyINode
		var first IMyINode
		var last IMyINode

		a := &MyINode{ID: 11}
		b := &MyINode{ID: 22}
		c := &MyINode{ID: 33}

		// Reset list for this test
		first = zero
		last = zero

		// Go test issue? passing `*MyINode` directly errors out with not IMyINode type which is incorrect. Fixed by passing a temp: `var ia IMyINode = a`

		// Push a -> list is now: a
		var ia IMyINode = a
		first, last = DoubleListInterface_push(first, last, ia)
		if first != a || last != a {
			t.Fatalf("push [a] failed, expected first and last to be 'a'; got first=%v, last=%v", first, last)
		}
		if a.GetIPrev() != zero || a.GetINext() != zero {
			t.Fatal("[a] links are incorrect after push")
		}

		// Push b -> list is now: a <-> b
		var ib IMyINode = b
		first, last = DoubleListInterface_push(first, last, ib)
		if first != a || last != b {
			t.Fatalf("push b failed, expected first=a and last=b; got first=%v, last=%v", first, last)
		}
		if a.GetINext() != b || b.GetIPrev() != a || b.GetINext() != zero {
			t.Fatal("list links are incorrect after pushing b")
		}

		// Push c -> list is now: a <-> b <-> c
		var ic IMyINode = c
		first, last = DoubleListInterface_push(first, last, ic)
		if first != a || last != c {
			t.Fatalf("push c failed, expected first=a and last=c; got first=%v, last=%v", first, last)
		}
		if b.GetINext() != c || c.GetIPrev() != b || c.GetINext() != zero {
			t.Fatal("list links are incorrect after pushing c")
		}
	})

	t.Run("remove", func(t *testing.T) {
		// Manually set up the list to be a <-> b <-> c for this test
		var zero IMyINode
		var first IMyINode
		var last IMyINode

		a := &MyINode{ID: 11}
		b := &MyINode{ID: 22}
		c := &MyINode{ID: 33}

		first = a
		last = c
		a.SetINext(b)
		a.SetIPrev(zero)
		b.SetINext(c)
		b.SetIPrev(a)
		c.SetINext(zero)
		c.SetIPrev(b)

		// Remove b -> list should be: a <-> c
		var ib IMyINode = b
		first, last = DoubleListInterface_remove(first, last, ib)
		if first != a || last != c {
			t.Fatalf("remove b failed, expected first=a and last=c; got first=%v, last=%v", first, last)
		}
		if a.GetINext() != c || c.GetIPrev() != a {
			t.Fatal("links incorrect after removing b")
		}
		if b.GetINext() != zero || b.GetIPrev() != zero {
			t.Fatal("removed node b was not correctly isolated")
		}

		// Remove a (the head) -> list should be: c
		var ia IMyINode = a
		first, last = DoubleListInterface_remove(first, last, ia)
		if first != c || last != c {
			t.Fatalf("remove a failed, expected first and last to be c; got first=%v, last=%v", first, last)
		}
		if c.GetIPrev() != zero || c.GetINext() != zero {
			t.Fatal("links incorrect after removing a")
		}
		if a.GetINext() != zero || a.GetIPrev() != zero {
			t.Fatal("removed node a was not correctly isolated")
		}

		// Remove c (the last node) -> list should be empty
		var ic IMyINode = c
		first, last = DoubleListInterface_remove(first, last, ic)
		if first != zero || last != zero {
			t.Fatalf("remove c failed, expected first and last to be nil; got first=%v, last=%v", first, last)
		}
		if c.GetINext() != zero || c.GetIPrev() != zero {
			t.Fatal("removed node c was not correctly isolated")
		}
	})
}

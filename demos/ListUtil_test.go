package demos

// Test Single and Double linked lists
// LLM

import "testing"

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
	SingleList_addFirst(&head, a)
	if head != a {
		t.Fatalf("expected head=a; got %v", head)
	}
	if a.Next != nil {
		t.Fatalf("expected a.Next=nil; got %v", a.Next)
	}

	SingleList_addFirst(&head, b)
	if head != b {
		t.Fatalf("expected head=b; got %v", head)
	}
	if b.Next != a {
		t.Fatalf("expected b.Next=a; got %v", b.Next)
	}

	SingleList_addFirst(&head, c)
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
	SingleList_addFirst(&head, a)
	SingleList_addFirst(&head, b)
	SingleList_addFirst(&head, c)

	// Pick should remove c first
	p := SingleList_pick(&head, func() *MyNode {
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
	p = SingleList_pick(&head, func() *MyNode { t.Fatal("creator called"); return nil })
	if p != b {
		t.Fatalf("expected pick=b; got %v", p)
	}
	if head != a {
		t.Fatalf("expected new head=a; got %v", head)
	}

	// Next pick → a
	p = SingleList_pick(&head, func() *MyNode { t.Fatal("creator called"); return nil })
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
	p := SingleList_pick(&head, creator)
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
	SingleList_pool(&head, a)
	if head != a {
		t.Fatalf("expected head=a; got %v", head)
	}
	if a.Next != nil {
		t.Fatalf("expected a.Next=nil; got %v", a.Next)
	}

	// Push b → list: b -> a
	SingleList_pool(&head, b)
	if head != b {
		t.Fatalf("expected head=b; got %v", head)
	}
	if b.Next != a {
		t.Fatalf("expected b.Next=a; got %v", b.Next)
	}

	// Push c → list: c -> b -> a
	SingleList_pool(&head, c)
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
		DoubleList_push(&first, &last, a)
		if first != a || last != a {
			t.Fatalf("push [a] failed, first=%v, last=%v", first, last)
		}
		if a.Prev != nil || a.Next != nil {
			t.Fatal("[a] links wrong")
		}

		DoubleList_push(&first, &last, b)
		if first != a || last != b {
			t.Fatal("push b failed")
		}
		if a.Next != b || b.Prev != a || b.Next != nil {
			t.Fatal("b links wrong")
		}

		DoubleList_push(&first, &last, c)
		if first != a || last != c {
			t.Fatal("push c failed")
		}
		if b.Next != c || c.Prev != b || c.Next != nil {
			t.Fatal("c links wrong")
		}
	})

	t.Run("foreach", func(t *testing.T) {
		ids := []int{}
		DoubleList_foreach(&first, func(n *MyDoubleNode) {
			ids = append(ids, n.ID)
		})
		if ids[0] != 1 || ids[1] != 2 || ids[2] != 3 {
			t.Fatalf("foreach failed, got %v", ids)
		}
	})

	t.Run("remove", func(t *testing.T) {
		// list is a <-> b <-> c
		// remove b -> a <-> c
		DoubleList_remove(&first, &last, b)
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
		DoubleList_remove(&first, &last, a)
		if first != c || last != c {
			t.Fatal("remove a failed")
		}
		if c.Prev != nil || c.Next != nil {
			t.Fatal("c links wrong after removing a")
		}

		// remove c -> empty
		DoubleList_remove(&first, &last, c)
		if first != nil || last != nil {
			t.Fatal("remove c failed")
		}
	})

	t.Run("addFirst", func(t *testing.T) {
		// list is empty
		d := &MyDoubleNode{ID: 4}
		e := &MyDoubleNode{ID: 5}

		DoubleList_addFirst(&first, &last, d)
		if first != d || last != d {
			t.Fatal("addFirst d failed")
		}

		DoubleList_addFirst(&first, &last, e)
		if first != e || last != d {
			t.Fatal("addFirst e failed")
		}
		if e.Next != d || d.Prev != e {
			t.Fatal("addFirst e links wrong")
		}
	})
}

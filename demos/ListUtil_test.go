package demos

// Test Single and Double linked lists
// LLM

import "testing"

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

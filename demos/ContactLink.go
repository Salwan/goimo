package demos

////////////////////////////////////// ContactLink
// (oimo/dynamics/ContactLink.go)
// A contact link is used to build a constraint graph for clustering rigid bodies. In a constraint graph, rigid bodies are nodes and constraints are edges. See also `JointLink`.

type ContactLink struct {
	prev *ContactLink
	next *ContactLink

	contact *Contact
	other   *RigidBody
}

func NewContactLink() *ContactLink {
	return &ContactLink{}
}

func (cl *ContactLink) GetNext() *ContactLink {
	return cl.next
}
func (cl *ContactLink) SetNext(x *ContactLink) {
	cl.next = x
}
func (cl *ContactLink) GetPrev() *ContactLink {
	return cl.prev
}
func (cl *ContactLink) SetPrev(x *ContactLink) {
	cl.prev = x
}

// TODO

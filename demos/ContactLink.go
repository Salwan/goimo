package demos

////////////////////////////////////// ContactLink
// (oimo/dynamics/ContactLink.go)
// A contact link is used to build a constraint graph for clustering rigid bodies. In a constraint graph, rigid bodies are nodes and constraints are edges. See also `JointLink`.

type ContactLink struct {
	prev    *ContactLink
	next    *ContactLink
	contact *Contact
	other   *RigidBody
}

func NewContactLink() *ContactLink {
	return &ContactLink{}
}

// TODO

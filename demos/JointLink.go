package demos

///////////////////////////// JointLink
// (oimo/dynamics/constraint/joint/JointLink.go)
// A joint link is used to build a constraint graph for clustering rigid bodies. In a constraint graph, rigid bodies are nodes and constraints are edges. See also `ContactLink`.

type JointLink struct {
	prev  *JointLink
	next  *JointLink
	joint *Joint
	other *RigidBody
}

func NewJointLink(joint *Joint) *JointLink {
	return &JointLink{
		joint: joint,
	}
}

// TODO

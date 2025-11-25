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

// TODO
// /**
// 	* Returns the contact of the link.
// 	*/
// public inline function getContact():Contact {
// 	return _contact;
// }

// /**
// 	* Returns the other rigid body of the link. This provides a quick access from a
// 	* rigid body to the other one of the contact.
// 	*/
// public inline function getOther():RigidBody {
// 	return _other;
// }

// /**
// 	* Returns the previous contact link in the rigid body.
// 	*
// 	* If the previous one does not exist, `null` will be returned.
// 	*/
// public inline function getPrev():ContactLink {
// 	return _prev;
// }

// /**
// 	* Returns the next contact link in the rigid body.
// 	*
// 	* If the next one does not exist, `null` will be returned.
// 	*/
// public inline function getNext():ContactLink {
// 	return _next;
// }

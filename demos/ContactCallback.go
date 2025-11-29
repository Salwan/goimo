package demos

///////////////////////////////// ContactCallback
// (oimo/dynamics/callback/ContactCallback.go)
// A callback class for contact events. Contact events between two shapes will occur in following order:
//
// 1. `beginContact`
// 2. `preSolve` (before velocity update)
// 3. `postSolve` (after velocity update)
// 4. (repeats 2. and 3. every frame while the shapes are touching)
// 5. `endContact`

type IContactCallback interface {
	// This is called when two shapes start touching each other. `c` is the contact of the two shapes.
	beginContact(c *Contact)
	// This is called every frame **before** velocity solver iterations while two shapes are touching. `c` is the contact for the two shapes.
	preSolve(c *Contact)
	// This is called every frame **after** velocity solver iterations while two shapes are touching. `c` is the contact for the two shapes.
	postSolve(c *Contact)
	// This is called when two shapes end touching each other. `c` is the contact of the two shapes.
	endContact(c *Contact)
}

type ContactCallback struct{}

func NewContactCallback() *ContactCallback {
	return &ContactCallback{}
}

func (cc *ContactCallback) beginContact(c *Contact) {}

func (cc *ContactCallback) preSolve(c *Contact) {}

func (cc *ContactCallback) postSolve(c *Contact) {}

func (cc *ContactCallback) endContact(c *Contact) {}

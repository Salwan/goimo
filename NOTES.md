Porting Notes
========================================

* Use float64 for Float
* Use int for Int
* Use enum pattern for Haxe enumerations
* Keep to name conventions but minor corrections are ok (Settings instead of Setting for example)
* Every struct should be allocated/stored as a pointer to match Haxe by-refs behavior
* There's no clear definition, but IVec3 appears to be a macro Vec3 implemented as an `Array<Dynamic>` rather than a struct with its own math library in `M.hx`, may need to understand Haxe's macros but this is definitely done for performance
* For Haxe `Any`, use Go's `any` (alias of: `interface{}`)

## Modifications

List of suggested modifications:

* RigidBody uses an adhoc linked list via _next/_prev RigidBody, just use list for this -> many other object types use the same pattern

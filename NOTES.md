Porting Notes
========================================

* Use float64 for Float
* Use int for Int
* Use enum pattern for Haxe enumerations
* Keep to name conventions but minor corrections are ok (Settings instead of Setting for example)
* Every struct should be allocated/stored as a pointer to match Haxe by-refs behavior
* There's no clear definition, but IVec3 is a flattened Vec3 array with its own math library in `M.hx`.
* For Haxe `Any`, use Go's `any` (alias of: `interface{}`)
* For initial consistency, reimplement all `M.*` math funcs in MathUtil. Can gradually switch to normal math funcs later when things are working.
* According to LLM, any struct in Go up to 64 bytes can be kept in registers and copied quickly. Passing and using Vec3/Quat by pointer wastes CPU cycles due to additional indirection and hurts optimizer. Mat3 (72 bytes) non-performance critical code can be by-value, while performance sensitive can be py-pointer. Mat4/Transform definitely need to be by pointer as generated asm will memory copy those and thats a higher cost than pointer indirection.
* Mark structs that are supposed to implement an interface (or 3) with a comment
* Mark methods that are supposed to override embedding/interface with a comment
* Enums should use const style TypeName_VALUENAME to match Haxe's namings

## Modifications

List of suggested modifications:

* RigidBody uses an adhoc linked list via _next/_prev RigidBody, just use list for this -> many other object types use the same pattern so this has a large overhead -> plan after porting is done

## OimoPhysics design

They are using a very cool system to optimize all math operations by flattening Vec3/Mat3/etc objects to their individual components. The way they do this is via the `M.hx` file which implements a lot of Haxe macro magic to modify function calls directly so they don't involve objects wrangling but direct primitive types.

Go doesn't need this now as it allocates small types on the stack automatically and inlines things where possible. However, a similar approach might later be developed to implement manual SIMD math to speed up operation. Either way I'm avoiding allocations whenever `M.*` math functions are employed.

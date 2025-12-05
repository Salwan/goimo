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
* Mark structs that are supposed to implement an interface with a comment: `// implements X`
* Mark methods that are supposed to override embedding/interface with a comment: `// override`
* Enums should use const style TypeName_VALUENAME to match Haxe's namings
* Whenever DoubleList_* or SingleList* are used remember that the design is changed so new head/tail are returned by function rather than modified via pointer indirection

## OimoPhysics design

They are using a very cool system to optimize all math operations by flattening Vec3/Mat3/etc objects to their individual components. The way they do this is via the `M.hx` file which implements a lot of Haxe macro magic to modify function calls directly so they don't involve objects wrangling but direct primitive types.

Go doesn't need this now as it allocates small types on the stack automatically and inlines things where possible. However, a similar approach might later be developed to implement manual SIMD math to speed up operation, for that floats have to be packed into vectors. Either way I'm avoiding allocations whenever `M.*` math functions are employed.

### Minimum and Maximum macros

These are used to execute one of three parts based on minimum or maximum of 3 values. To match their logic (handling equality fallthrough the same way) use:

##### M.compare3min():

```go
if a < b && a < c {
    doA()
} else if a >= b && b < c {
    doB()
} else {
    doC()
}
```

##### M.compare3max():

```go
if a > b && a > c {
    doA()
} else if a <= b && b > c {
    doB()
} else {
    doC()
}
```

### Macro List Foreach

Haxe's macro `M.list_foreach` is designed so that if the current item is removed within the loop it won't cause an issue as the next item is fetched before the body of the loop not after. To match this behavior use this pattern AND BEWARE OF continue, break:

```go
for c := self.somethingList; c != nil; {
    next := c.next

    // body of loop that could delete c (nullfying next item)
    // CRITICAL! any continue or break, you need to set c = next before otherwise you got an infinite loop

    c = next
}
```

### do-while(false)+break

Haxe runs a block of code inside a `do {} while(false);` and uses `break` to jump out of the block.

Go can use labels but I'd rather match Haxe's approach for now:

```go
for range 1 {
    // can break out anytime
}
```

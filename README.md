# libgcxx

A single-header C++11 **conservative, generational, mark-sweep garbage collector**.

Add `gc.hpp` to your project, include it, and allocate with `gc<T>()`. There is no library to link,
no runtime to start, no configuration to set, and no initialization call to make.

```cpp
#include "gc.hpp"

struct Node { Node* next; int value; };

int main() {
    Node* a = gc<Node>();               // allocate and construct; scan class inferred
    Node* b = gc<Node>();               // same
    a->next = b;                        // ordinary pointer assignment; no write barrier needed

    int*  arr = gc<int[]>(1000);        // runtime-length array, value-initialized
    char* buf = (char*)gca(65536);      // atomic memory: never scanned
    auto* s   = gcs<Node>();            // forced conservative scan

    (void)arr; (void)buf; (void)s;
    gc_collect();                       // usually unnecessary
}
```

## Features

- **Conservative.** The collector has no type information. It scans the stack, registers, static data
  and every allocated object's payload word by word. It is designed to **retain too much rather than
  ever miss a mark**.
- **Generational, with a nursery.** Objects are promoted to an old generation after surviving a few
  cycles, so most collections touch only the young generation.
- **No write barrier.** Marking completes inside a single stop-the-world window; sweeping is
  incremental and paced by allocation volume.
- **Lock-free allocation.** Each thread owns a private heap and a partial-chunk list. Only chunk
  exhaustion touches shared state.
- **No destructors.** The collector never calls user code, including on weak-reference expiry.
- **Verified internals.** Chunk metadata carries a redzone and checksum, free-list links are encoded
  with per-chunk keys, and a self-check can validate overall heap consistency.
- **Portable.** Windows, Linux, Android, macOS, iOS and FreeBSD, on x86-64 and AArch64.

## Requirements

- **64-bit only.** 32-bit targets are rejected at compile time.
- **C++11.** The MSVC toolchain family is built as C++14, because its standard library requires it.
- **Threads.** The header uses `std::thread` and platform thread primitives, so link the platform
  thread library.

## Allocation

Three entry points differ in exactly one respect: **whether the memory is conservatively scanned**.

| Entry point | Scan decision | Use it for |
|---|---|---|
| `gc<T>(args...)`, `gc<T[]>(n)`, `gc(n)` | **Automatic.** Built-in types and types declared plain via `gc_declare_plain<T>()` are not scanned; everything else is. Untyped `gc(n)` is scanned in full. | The everyday choice |
| `gcs<T>(...)`, `gcs<T[]>(n)`, `gcs(n)` | **Forced scan**, even for a built-in `T`. | Fallback when type deduction may be wrong |
| `gca<T>(...)`, `gca<T[]>(n)`, `gca(n)` | **Never scanned.** The caller guarantees there are no pointers inside. | Byte buffers, strings, data tables, bitmaps |

All three share the same allocation path, pointer reverse lookup and reclamation.

```cpp
struct Foo { Foo* next; int v; };

Foo*  p  = gc<Foo>(1, 2.0);        // contains pointers -> scanned
int*  n  = gc<int>(42);            // built-in type -> not scanned
char* b  = gc<char[64]>();         // static-length array
int*  a  = gc<int[]>(1000);        // runtime-length array, value-initialized
int*  z  = gc<int[]>(0);           // n == 0 returns a valid pointer
void* r  = gc((std::size_t)128);   // raw memory, zeroed
int*  f  = gcs<int>(42);           // scanned even though int is built-in
char* t  = (char*)gca(65536);      // never scanned
```

**Common semantics**

- Memory is aligned as `malloc` is. Any type with `alignof(T) <= 16` can be allocated; a stricter
  alignment requirement is a compile-time error.
- A single object may not exceed `0xFFFFFF00` bytes (about 4 GiB - 256).
- **Do not `free` this memory and do not run destructors on it.** Reclamation happens automatically
  once the whole object becomes unreachable.
- Allocation failure **throws `std::bad_alloc`**; it never returns `nullptr`.
- `gc(n)`, `gcs(n)` and `gca(n)` always return **zeroed** memory, exactly like `calloc`. `gc<T>(...)`
  zeroes before constructing, except when `T` is trivially default constructible and would write the
  whole block anyway.
- **Interior pointers resolve.** A pointer to any position inside an object keeps that object alive,
  including a member address, an array element, the middle of a large object, or a negatively biased
  address such as `p - 24`.
- An array carries a single object header, so a pointer to any element keeps the whole array alive.
- Arrays follow `new U[n]()` exactly: elements are value-initialized, `n == 0` still returns a valid
  pointer, and the array form accepts no constructor arguments.

**`gca` is the one to be careful with.** Violating its contract does not cost a little speed, it costs
a dangling pointer, and it produces no diagnostic at all, because never scanning is precisely what it
promises. When in doubt, use `gc` or `gcs`.

## `gc_allocator<T>`

A C++11 Allocator backed by the GC heap, for containers that hold GC pointers.

```cpp
std::vector<Foo*, gc_allocator<Foo*>> v;
```

| Member | Semantics |
|---|---|
| `allocate(n, hint = nullptr)` | Allocates `n * sizeof(T)`, zeroed. Throws `std::bad_alloc` if `n > max_size()`. |
| `deallocate(p, n)` | A no-op; the GC reclaims the block once it is unreachable. |
| `max_size()` | `SIZE_MAX / sizeof(T)`. |
| `construct`, `destroy`, `address` | Standard compatibility members. |
| `operator==`, `operator!=` | Any two instances compare equal; the allocator is stateless. |
| `value_type`, `pointer`, `size_type`, and the remaining standard typedefs, plus `rebind<U>::other` | As required by the Allocator requirements. |

`static_assert(alignof(T) <= 16)` applies here too.

**Containers that store GC pointers must use this allocator.** With the default `std::allocator` the
elements live on the `malloc` heap, which the collector does not scan, so those pointers are not
roots and their targets get reclaimed. Containers that store no GC pointers, such as
`std::vector<int>`, are fine either way.

## Lifecycle

### `bool gc_collect()`

Forces one complete major collection. Returns `true` when the cycle reclaimed memory, and `false`
when it reclaimed nothing, for either of two reasons: the cycle was **cancelled** because a thread
did not reach a safe point in time, or **another thread is already collecting**. Neither is an error,
and the caller cannot distinguish them from the return value.

Collection is driven automatically by allocation volume, so normal programs do not need this.

### `gc_init()`

A macro, optional and idempotent. It anchors the main thread's stack-scan upper bound to the caller's
frame. The collector is already live from static-initialization time, so calling it is harmless and
not calling it is the normal case.

### `void gc_dispose()`

Releases every block of memory the collector obtained, and resets its state. The library remains
usable afterwards.

The caller must guarantee that **no other thread is using the library** at that moment. Unlike
`gc_collect()`, which reclaims unreachable objects, this releases **all** memory, so every pointer
becomes invalid.

### `void gc_refresh_roots()`

Re-collects the root ranges of the static data area and of loaded shared libraries. Root ranges are
gathered once automatically, so this is needed **only** after a runtime `dlopen` whose library holds
GC pointers in its globals. It is idempotent and thread-safe.

## Debugging

### `std::size_t gc_dump(std::FILE* out = stdout)`

Writes a human-readable summary of the heap: totals, a histogram by slot size, per-thread list
lengths, and diagnostic counters. It does not allocate and does not trigger a collection.

It walks the object chains only while the heap is idle. While a collection is in progress it prints a
single explanatory line and returns, because the chains are being modified concurrently. For a
complete dump, call it after `gc_collect()` returns. The output format is not a stable contract.

### `void* gc_is_gc_pointer(const void* p)`

Returns the internal object pointer when `p` refers to a live object on the GC heap, otherwise
`nullptr`. Interior pointers count as hits, using the same predicate as marking.

It does **not** consider reachability: a non-null result only says "this is a GC object". To ask
whether an object will be reclaimed, use a weak reference.

## Weak references

`gc_weak<T>` is a handle that becomes expired once its target is unreachable.

```cpp
int* p = gc<int>(10);      // p is a root, so the object stays alive
gc_weak<int> w = p;        // register a weak reference
if (w) { int v = *w; }     // target still alive
// once p goes away, the next collection reclaims the object and w expires
```

| Member | Semantics |
|---|---|
| `gc_weak()`, `gc_weak(nullptr)` | Empty handle. |
| `gc_weak(T* p)` | Registers a weak reference. Not explicit. |
| copy | Registers an independent entry. |
| move | Transfers the registration; the source becomes invalid. |
| `get()`, `lock()` | The target if it is still alive, otherwise `nullptr`. |
| `expired()`, `explicit operator bool` | Expiry test. |
| `operator*`, `operator->` | Dereference. |
| `reset()`, `reset(T* p)` | Deregister, or re-register with a new target. |
| `index()` | Diagnostic index. Not a pointer, and not a liveness test. |

**A weak reference does not count as a reference.** `gc_weak<int> w = gc<int>(10);` compiles, but the
object has no other reference, so the first collection reclaims it. Dereferencing an expired handle
is undefined behaviour.

## Ephemerons

`gc_ephemeron<K, V>` makes a value reachable **only while its key is reachable**.

```cpp
v->owner = k;                      // the value points back at the key
gc_ephemeron<Key, Value> e;
e.set(k, v);
if (e) { Value* got = *e; }
```

| Member | Semantics |
|---|---|
| `gc_ephemeron()`, `gc_ephemeron(K* k, V* v)` | Empty, or registered by construction. |
| copy / move | Independent registration; transfer, respectively. |
| `set(K* k, V* v)` | Register or replace a pair. `k == nullptr` is equivalent to `reset()`. |
| `reset()` | Deregister. |
| `get()`, `lock()` | The value when the key is alive, otherwise `nullptr`. Non-null means valid. |
| `key()` | Reads the key back, or `nullptr` when it is unreachable. |
| `expired()`, `explicit operator bool`, `operator*`, `operator->`, `index()` | As for `gc_weak`. |

This exists for one specific shape: when the value points back at the key, an ordinary weak reference
would let the value keep the key alive, so the key would never die and the handle would never expire.
An ephemeron folds the value into the same decision, so key and value are reclaimed together.

## Declaring a type pointer-free

```cpp
struct Vec3 { float x, y, z; };
template <> constexpr bool gc_declare_plain<Vec3>() { return true; }

Vec3* v = gc<Vec3>();    // now treated as containing no pointers
```

This is a **promise, not a hint**. If `T` actually contains pointers, those pointers will not be
treated as roots and their targets will be wrongly reclaimed. Specialize it in the global namespace,
before the first use of `gc<T>()`, and use `gcs<T>()` instead when unsure. Built-in types and memory
from `gca` need no declaration.

## Platform support

| Operating system | x86-64 | AArch64 |
|---|---|---|
| Windows 10 / 11 | Supported, test suite run | - |
| Linux (glibc) | Supported, test suite run | Supported, cross-compiled and run under emulation |
| Android | Compiles, API 21 and later | Compiles, API 21 and later |
| macOS, iOS | Implemented, not yet verified on hardware | Implemented, not yet verified on hardware |
| FreeBSD 14 | Supported, test suite run | - |
| NetBSD, OpenBSD, DragonFly BSD | Not supported | Not supported |

The last row is a deliberate non-support rather than an untested area. On those systems neither the
static-root nor the dynamic-root path can be compiled in, so an object referenced only by a global
variable would be reclaimed. Guessing the required symbol names risks a silent missed scan, which is
worse than saying so plainly. If you must use the library there, do not keep GC pointers only in
globals; keep a copy on the stack or inside a reachable object.

Linux with musl, and architectures other than the two above, are unexercised.

## How it works

- **Tri-color marking with a white-bit flip.** `WHITE0` and `WHITE1` alternate, flipped once per major
  cycle, so the heap never has to be walked merely to reset colors.
- **16-byte object header**: next pointer, size, color, age, scan class. The payload starts at offset
  16 and slots are 16-byte aligned, so payloads are naturally aligned for any `alignof(T) <= 16`.
- **64 KiB chunks with size classes.** A small-object chunk serves exactly one slot size, so
  resolving an object from a candidate pointer is a constant-time index plus two comparisons. Large
  objects own chunks exclusively.
- **Per-thread heaps.** Object lists, the partial-chunk list and debt accounting are thread-private;
  only chunk exhaustion takes the global lock.
- **Objects allocated during a collection are never reclaimed by that cycle.** This is what keeps the
  collector correct without a write barrier, and it makes old-to-young assignment safe.
- **Physical memory is returned.** Retired chunks keep their address space but their data pages can be
  decommitted after a short delay, so a peak does not stay resident.

## Threading

Collection is a **cooperative** stop-the-world. The initiating thread sets a request; each other
thread reaches a safe point at its next allocation or lock acquisition, scans its own stacks and
registers, and parks. Once everyone is in place, marking completes atomically and the white bit flips.
Threads that are already parked are released only after marking finishes.

**Every thread should periodically allocate, or at least contend on the internal lock.** A thread
parked for a long time in pure computation, in `join`, or on a condition variable never reaches a safe
point, so collections keep being cancelled and the heap keeps growing. On Windows the collector can
fall back to suspending such a thread; elsewhere it can only cancel.

**If the collector cannot stop the threads, it cancels rather than marking while broken.** A cancelled
cycle reclaims nothing and reports `false`. It never produces a dangling pointer.

## Configuration

There is none. The `GCXX_*` macros you may notice in the header exist for the project's own
cross-compilation and ablation runs; a normal build defines none of them, and the defaults are the
configuration the library is designed and measured in. The safety machinery is always on.

## Installation

`gc.hpp` is the entire library. Copy it into your project, or add this repository as a submodule, and
add its directory to your include path:

```cpp
#include "gc.hpp"
```

Compile as C++11 or later, and link the platform thread library (`-pthread` on POSIX). With MSVC, use
`/std:c++14` or later, because the MSVC standard library requires it. Nothing else is needed.

## Limitations

- **The collector never calls destructors.** Release non-memory resources yourself.
- **Over-retention is expected.** A stale stack slot or an integer that looks like an address can keep
  an object alive past its last use. An assertion of the form "this object must have been collected"
  is not decidable inside the thread that allocated it.
- **There is no way to free an individual object.**
- **Pause time is in the millisecond range** for a heap with hundreds of thousands of live objects,
  and is dominated by root scanning.
- **No finalizers, no incremental marking, no precise collection.** See the design notes above; each
  was considered, and where a measurement exists it is recorded in the header.
- **The header also exposes a namespaced diagnostics interface used by the project's own tests.** It
  is not part of the stable public API; its fields and signatures change with the implementation.

## License

MIT. See [LICENSE](LICENSE).

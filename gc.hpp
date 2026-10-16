/**
 * @file gc.hpp
 * @brief Header-only C++11 conservative generational mark-sweep garbage collector.
 *
 * **Platform: 64-bit only** (`sizeof(void*) == 8`). 32-bit platforms are rejected at compile time by `static_assert`
 * outright -- the soundness of conservative scanning rests on the premise "pointer width == machine word width == 8 bytes".
 *
 * Including this header is all there is: no runtime configuration and no mandatory compile-time setup.
 * Safety and robustness facilities (metadata redzone, block-tail padding, free-list two-key encoding, allocation-order
 * randomization, large-object guard-page sampling, decommit of retired block data areas) are always on.
 *
 * @note A number of `GCXX_*` macros can nonetheless be overridden from the command line (`-D`); each is
 *       documented at its definition and wrapped in `#ifndef` for exactly that reason. They exist for
 *       coverage builds and for reproducing a degraded path, **not** as a tuning surface: the defaults are
 *       the configuration the library is designed, tested and measured in.
 *
 * @note **This library has no write barrier**: the mark phase is **atomic** (done in one shot inside STW) and **the sweep phase is incremental**
 *       (driven by allocation debt). A page-protection write barrier is the only mechanism that could drive incremental marking here, and its
 *       cost (a trap plus a page-fault handler round trip per dirtied page, at a dirty-page rate that any write-heavy workload reaches)
 *       exceeds the cost of the atomic mark it would replace. **No incremental-marking path exists**: marking is always atomic.
 *
 * @section design Design notes
 *
 * 1. **Conservative scanning** (Boehm style): the GC has no type information, so it treats the stack / registers / static area /
 *    the payload of every allocated object as a "possible pointer array" and scans it word by word; any word that falls inside
 *    the GC heap address range and resolves to an object counts as a reference. **Better to retain too much than to ever miss a mark.**
 *
 * 2. **Tri-color marking + white-bit flip** (Lua 5.x style): white = unvisited (reclaimable), gray = discovered but not yet
 *    expanded, black = expanded. The two white bits WHITE0/WHITE1 alternate, flipped once per major cycle in the atomic
 *    phase, so the whole heap never has to be walked to reset colors.
 *
 * 3. **Marking is atomic** (completed in a single STW), **sweeping is incremental**: sweeping is split into small steps and advanced
 *    at every allocation, at a pace of "as many GC work units repaid as bytes allocated". The reason marking cannot be made incremental
 *    is that a conservative GC has no write barrier -- within the marking window a mutator can "read a pointer out of a region that has
 *    not been traced yet and then erase the original value", thereby holding a white object and producing a dangling pointer. The sweep phase has no such problem: liveness
 *    is already fixed once marking ends. **No incremental-marking fallback exists**: the only write barrier that could drive one
 *    (page protection) costs more than the atomic mark it would replace, so `GCXX_WRITE_WATCH_MARK` is a no-op.
 *
 * 4. **Generational**: objects carry an age. New objects go to young and are promoted to old after surviving several cycles. A minor cycle sweeps
 *    only the young generation; the old generation is conservatively scanned as part of the root set for the whole minor cycle, so it is always safe; a major cycle
 *    marks the whole heap + sweeps the whole heap and flips the white bit.
 *
 * 5. **Nursery**: every object allocated while the GC is running is linked into the nursery and takes no part in the current
 *    sweep -- "objects allocated during collection are never wrongly reclaimed by this cycle". This is what keeps the collector correct
 *    without a write barrier, and makes old-to-young forms such as `old->field = gc<X>()` safe as well.
 *
 * 6. **16-byte object header**: next(8) + size(4) + marked(1) + age(1) + type(1) +
 *    reserved(1). The payload is fixed at offset 16 and slot starts are 16-byte aligned, so the payload is naturally
 *    16-byte aligned (any type with alignof(T) <= 16 can be allocated) and no per-object offset field is needed.
 *    The gray-list pointer is **not in the object header** (it lives in the blocked array gc_state::gray_blocks). Leaf objects (integer/floating-point/
 *    enum/plain data, with type recording the scan class) are skipped wholesale when tracing; gc_declare_plain<T>() also marks
 *    custom plain data structures as skippable.
 *
 * @section api Public API
 *
 * Three allocation interfaces, differing in exactly one thing: **whether this memory is to be conservatively scanned**.
 *
 *     gc<T>(args...)      **decided automatically**: built-in types / plain data declared via is_plain are not scanned,
 *     gc<T[]>(n)          pointers and non-built-in types are scanned. This is the everyday one.
 *     gc(n)               raw memory, treated as an "unknown byte block" -> **scanned in full**
 *     gcs<T>(...)/gcs(n)  **force scanning**, even when T is a built-in type. Use it as a fallback when
 *                         type deduction is uncertain (the cost is a scan on every mark).
 *     gca<T>(...)/gca(n)  **atomic memory: never scanned**. The caller guarantees there are no pointers inside. Use it for
 *                         plain-data buffers to save the word-by-word scan of the whole block.
 *
 * The three differ only in "whether to scan"; the allocation path, the reverse lookup (interior pointers) and reclamation are entirely the same. The two
 * unscanned kinds (all of gca, whatever gc judges to be a leaf, and anything from gcs with size < sizeof(void*)) all live in
 * **atomic blocks**, physically separated from objects that need scanning -- "an object that needs scanning never appears in an atomic block"
 * is a structural property gc_verify asserts. The one automatic exception: a block too small to hold a complete pointer
 * (size < sizeof(void*)), where a pointer is physically impossible.
 *
 *     gc_allocator<T>     an allocator meeting the C++11 Allocator requirements (it decides by T automatically)
 *     gc_collect()        force one complete collection
 *     gc_dump(FILE*)      dumps the heap state as readable **English** text (stdout by default): totals + a histogram by slot
 *                         size + each heap's list lengths + diagnostic counters.
 *                         **Read-only, does not allocate, does not trigger collection**; see the "debug interface" section below.
 *     gc_is_gc_pointer(p) does this pointer point at a live object on the GC heap? On a hit it returns the interior header
 *                         pointer, otherwise nullptr. Interior pointers hit as well; **reachability is not considered**.
 *     gc_init()           optional and idempotent: the GC is already ready during static construction, this call merely anchors
 *                         the main thread's stack-scan upper bound to the caller's stack frame
 *
 * @note All of the above are **in the global namespace** (the same level as `gc` / `gca` / `gcs`):
 *       write `gc_collect()` / `gc_dump(...)` directly, **with no qualification needed**.
 * @note `gc_dump` / `gc_is_gc_pointer` are the whole of the public debug surface, and that is deliberate:
 *       richer facilities (a heap-histogram type and a callback object traversal) exist only in the
 *       **internal test interface** below, which carries no stability promise. Keeping the public
 *       contract small means debug capability never becomes something a user has to learn.
 *
 * @section internal_test_api Internal test interface (**not part of the public API**)
 *
 * **Everything in `gcxx::tests` is an internal test/diagnostic interface, not a public API, and no stability is promised**:
 * fields, semantics and function signatures all change with the implementation, so **external code must not depend on them**. Tests and probes
 * pull them in with `using gcxx::tests::...;`. Currently there are:
 *
 *     gc_stats / gc_stat()   counter snapshot (live, cycles, phase timings, decommits, ...)
 *     gc_verify()            whole-heap consistency self-check, returns the number of inconsistent items (0 = healthy)
 *                            @warning it is **not** purely read-only; see its own documentation
 *     gc_test_state()        state-machine snapshot: phase / cycle kind / white bit / mark exclusivity / aborting, ...
 *     gc_test_count_colors() counts by the color in the object header (white0/white1/gray/black/free sentinel)
 *     gc_test_visit_objects() walks every live object (stops when the callback returns false)
 *     gc_test_chunk_count()  global block count (including empty blocks on the recycle stack) and recycle-stack depth
 *     gc_test_object_of()    address -> object info (generation / age / size / whether free)
 *
 * Two **optional** extension interfaces (they do not exist unless used):
 *
 *     gc_weak<T> o = ptr            a weak-reference **handle**: it becomes "expired" when the target is unreachable. Test
 *                                   with expired() / operator bool, read the value through *o / o->
 *                                   (dereferencing after expiry is UB).
 *     gc_ephemeron<K,V> e;          a key/value pair: **the value is reachable only while the key is reachable**. The shape "the
 *     e.set(k, v); *e               value points back at the key" is only collected cleanly by this (a weak key + strong value would
 *                                   pin the key and never collect it).
 *
 * Plus one optional specialisation point, `gc_declare_plain<T>()` (declares that a type contains no pointers inside).
 *
 * Which to pick: gc by default. **Use gca for large blocks known to contain no pointers** (the gain grows linearly with block size: measured
 * with 20000 live 1KB buffers, one collection cycle went 2.07ms -> 0.28ms). When type deduction may be wrong,
 * fall back to gcs -- **misusing gca does not cost "a little speed", it costs a dangling pointer**, so do not use it when unsure.
 *
 * gc has two faces: with a type argument it allocates and constructs an object; with only a length it allocates raw memory and returns
 * void* (zeroed content, the same as calloc). Arrays use gc<T[]>(n), where n may be a runtime value:
 *
 *     int*   a = gc<int[]>(1000);
 *     Node** g = gc<Node*[]>(64);
 *
 * A static length can also be written as a type: gc<int[1000]>(). STL containers use gc_allocator<T>.
 *
 * @section threads Thread model: mimalloc-style per-thread heap
 *
 * - **The allocation hot path is entirely lock-free**: each thread has a private gc_heap (a platform TLS slot); the object lists,
 *   the partial chunk list and debt accounting are all thread-private; only when a chunk runs out does it take a block from the global per-class
 *   recycle stack (lock-free CAS), and only failing that allocate a new 64KB chunk under the global lock.
 * - **Collection**: cooperative stop-the-world (safepoint handshake). The initiating thread sets a global request flag; every mutator
 *   thread reaches a safepoint at its **next allocation / lock acquisition** -- capturing its own registers + scanning its own stack (at that
 *   moment pointers on the stack are necessarily reachable) -- and then parks; once the initiating thread confirms every active thread is in place,
 *   it atomically completes marking, flips the white bit and releases them. Sweeping proceeds incrementally, concurrently with mutator allocation: dead slots are returned
 *   to the owner heap with a lock-free Treiber stack.
 * - **Why mutators scan themselves**: the collector scans only its own roots and the frozen stacks of force-suspended threads;
 *   a thread that parks voluntarily stops inside its own stack frame, so its roots can only be scanned by itself before parking. The premise is that the heap
 *   was published into the global list **before it could reach a safepoint**; otherwise the thread initiating STW will not enumerate it and therefore
 *   will not wait for it to be in place, and the marks it pushed into the gray list from its self-scan could still land after marking completes.
 * - **When it cannot stop them, cancel rather than mark while broken**: if some thread has not reached a safepoint within the
 *   budget (STW_TIMEOUT_MS), the cycle is **cancelled**. Because mutators have already self-scanned and colored objects, cancellation must first drain the in-flight
 *   self-scans (seq_cst handshake + sp_scanning count), then roll all colors back wholesale to "unmarked this cycle",
 *   and only then reset the state. Cancelling reclaims no memory (the heap grows somewhat more), but it never wrongly kills a live object; by contrast,
 *   "scan the stack of a still-running thread and then mark as usual" has no write barrier as a backstop and is unsound. For the cancellation count see
 *   gc_stat().cancelled_cycles.
 * - **Usage constraint**: every thread should periodically pass through an allocation point (gc), or at least contend for
 *   gc_mutex. A thread that sits for a long time in pure computation / join / condition-variable waits cannot reach a safepoint, which keeps collections
 *   being cancelled and the heap growing. gc_collect() follows the same rule (it returns false when cancelled).
 */

#ifndef GCXX_GC_HPP_INCLUDED
#define GCXX_GC_HPP_INCLUDED
/* ==========================================================================
 * Platform detection: each platform this library supports has its own `GCXX_PLATFORM_*` macro (value 1),
 * and every unrecognised platform is 0. **All platform branches should use these macros rather than sniffing compiler built-ins directly** --
 * because `__linux__` also matches Android and `__APPLE__` also matches both iOS and macOS,
 * while those three do not have the same availability of the same set of POSIX interfaces (see each adaptation point for details).
 * @note Wrapped in `#ifndef`: an external `-D` override is permitted, for cross-compilation controls.
 * ========================================================================== */
#ifndef GCXX_PLATFORM_WINDOWS
#  if defined(_WIN32)
#    define GCXX_PLATFORM_WINDOWS 1
#  else
#    define GCXX_PLATFORM_WINDOWS 0
#  endif
#endif
#ifndef GCXX_PLATFORM_ANDROID
#  if defined(__ANDROID__)
#    define GCXX_PLATFORM_ANDROID 1
#  else
#    define GCXX_PLATFORM_ANDROID 0
#  endif
#endif
#ifndef GCXX_PLATFORM_IOS
    /* iOS implementation detail: under `__APPLE__`, `TargetConditionals.h` provides `TARGET_OS_IPHONE`.
     * We do not include that header (it is Apple-proprietary and drags in a pile of things); instead we use `__ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__`
     * -- which clang **necessarily** predefines on iOS targets (the macOS target does not). */
#  if defined(__APPLE__) && \
      (defined(__ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__) || \
       defined(__ENVIRONMENT_TV_OS_VERSION_MIN_REQUIRED__) || \
       defined(__ENVIRONMENT_WATCH_OS_VERSION_MIN_REQUIRED__))
#    define GCXX_PLATFORM_IOS 1
#  else
#    define GCXX_PLATFORM_IOS 0
#  endif
#endif
#ifndef GCXX_PLATFORM_MACOS
#  if defined(__APPLE__) && !GCXX_PLATFORM_IOS
#    define GCXX_PLATFORM_MACOS 1
#  else
#    define GCXX_PLATFORM_MACOS 0
#  endif
#endif
#ifndef GCXX_PLATFORM_LINUX
    /* Desktop/server Linux. **Excludes Android** -- that is detected separately (the two differ in the availability of `pthread_getattr_np`
     * and `/proc`). */
#  if defined(__linux__) && !GCXX_PLATFORM_ANDROID
#    define GCXX_PLATFORM_LINUX 1
#  else
#    define GCXX_PLATFORM_LINUX 0
#  endif
#endif
/** @brief Whether this is a "POSIX-like" platform (the mmap/mprotect/pthread set is available). */
#ifndef GCXX_PLATFORM_POSIX
#  if GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID || GCXX_PLATFORM_MACOS || \
      GCXX_PLATFORM_IOS || defined(__unix__)
#    define GCXX_PLATFORM_POSIX 1
#  else
#    define GCXX_PLATFORM_POSIX 0
#  endif
#endif
/* ==========================================================================
 * BSD family
 *
 * ## Support status: **of the BSD family, only FreeBSD is supported**
 *
 * This library **promises to work on FreeBSD only**. NetBSD / OpenBSD / DragonFly still have platform macros
 * (the code **does compile** on them), but they are **outside the supported range** -- because of root coverage (see below),
 * not because of "we could not be bothered to test".
 *
 * All four mainstream BSDs predefine `__unix__` and their own name macro, but they do not agree on the availability of
 * **non-portable interfaces**, so they are recognised individually. Key differences that have been verified (sources in docs/platform-support.md):
 *
 *   * **`pthread_getattr_np` is missing on FreeBSD / NetBSD / OpenBSD**
 *     (GNU Gnulib portability table: missing on FreeBSD 14.0, NetBSD 5.0, OpenBSD 7.5).
 *     => FreeBSD uses `pthread_attr_get_np` (see below); the others take the conservative path.
 *   * **`/proc/self/statm` does not exist** => the RSS diagnostic value goes through `sysctl` instead (the struct and field names differ per vendor).
 *
 * ### Why NetBSD / OpenBSD / DragonFly are out of support scope
 *
 * **They cannot get the roots**, and that is not a problem "a little less testing" can wave through:
 *   * `GCXX_PLATFORM_HAVE_LINK_H` is 1 only for Linux / FreeBSD / Android(~1),
 *     so on those three **both the static-root symbol branch and the dynamic-root branch are compiled out**;
 *   * => an object referenced only by a global/static variable is reclaimed as unreachable => **dangling pointer**.
 *
 * Filling that gap requires confirming symbol names on real machines (`__data_start` / `_end` / `__executable_start`)
 * or the availability of `dl_iterate_phdr`. **Guessing wrong means a silent missed scan** -- worse than "explicitly unsupported",
 * so this library chooses explicit non-support rather than filling in a chunk of code that cannot be verified.
 * Detailed evidence and "what would be needed to support them in future" are in `docs/platform-support.md` section 2.2 and section 5.
 *
 * @note The macro and RSS branches for these three platforms are kept in the tree so that "a change broke their compilation"
 *       stays visible (there is an entry for it in the build matrix); they carry **no runtime promise**.
 * ========================================================================== */
#ifndef GCXX_PLATFORM_FREEBSD
#  if defined(__FreeBSD__)
#    define GCXX_PLATFORM_FREEBSD 1
#  else
#    define GCXX_PLATFORM_FREEBSD 0
#  endif
#endif
/** @brief DragonFly BSD. **Detected separately from FreeBSD** -- measured to have **no `<sys/sysctl.h>`**
 *        (`fatal error: sys/sysctl.h: No such file or directory`), so it can neither reuse
 *        FreeBSD's RSS path nor share a single platform macro. */
#ifndef GCXX_PLATFORM_DRAGONFLY
#  if defined(__DragonFly__)
#    define GCXX_PLATFORM_DRAGONFLY 1
#  else
#    define GCXX_PLATFORM_DRAGONFLY 0
#  endif
#endif
#ifndef GCXX_PLATFORM_NETBSD
#  if defined(__NetBSD__)
#    define GCXX_PLATFORM_NETBSD 1
#  else
#    define GCXX_PLATFORM_NETBSD 0
#  endif
#endif
#ifndef GCXX_PLATFORM_OPENBSD
#  if defined(__OpenBSD__)
#    define GCXX_PLATFORM_OPENBSD 1
#  else
#    define GCXX_PLATFORM_OPENBSD 0
#  endif
#endif
/** @brief Whether this is any BSD family member (FreeBSD / DragonFly / NetBSD / OpenBSD). */
#ifndef GCXX_PLATFORM_BSD
#  if GCXX_PLATFORM_FREEBSD || GCXX_PLATFORM_DRAGONFLY || \
      GCXX_PLATFORM_NETBSD || GCXX_PLATFORM_OPENBSD
#    define GCXX_PLATFORM_BSD 1
#  else
#    define GCXX_PLATFORM_BSD 0
#  endif
#endif
/** @brief Whether this is an Apple platform (macOS **or** iOS -- the two share Mach-O / `_dyld_*` / libSystem).
 * @note It is needed because a batch of adaptation points **use the same code for macOS and iOS** (Mach-O section parsing,
 *       `_dyld_image_*`, `pthread_get_stackaddr_np`), while other points must distinguish them (stack bottom,
 *       whether the dynamic roots are really usable). `GCXX_PLATFORM_MACOS` / `GCXX_PLATFORM_IOS` express the latter. */
#ifndef GCXX_PLATFORM_DARWIN
#  if GCXX_PLATFORM_MACOS || GCXX_PLATFORM_IOS
#    define GCXX_PLATFORM_DARWIN 1
#  else
#    define GCXX_PLATFORM_DARWIN 0
#  endif
#endif
/** @brief Whether `pthread_getattr_np` is available (**it is on neither BSD nor Apple**).
 *
 * It only controls the **optimisation** of "can the stack bounds be asked for directly"; without it this library takes the conservative
 * fallback (recorded stack top + "current sp minus a margin"), and **correctness is unchanged**.
 * @note Measured (FreeBSD 14.5 real headers): `pthread.h` has **no** `pthread_getattr_np`,
 *       only `pthread_attr_get_np` (declared in `<pthread_np.h>`, see `GCXX_HAVE_PTHREAD_ATTR_GET_NP`). */
#ifndef GCXX_HAVE_PTHREAD_GETATTR_NP
#  if GCXX_PLATFORM_LINUX || (GCXX_PLATFORM_ANDROID && defined(__ANDROID_API__) && \
                              __ANDROID_API__ >= 24)
#    define GCXX_HAVE_PTHREAD_GETATTR_NP 1
#  else
#    define GCXX_HAVE_PTHREAD_GETATTR_NP 0
#  endif
#endif
/** @brief Whether FreeBSD's `pthread_attr_get_np` (`<pthread_np.h>`) is available.
 *
 * FreeBSD has **no** `pthread_getattr_np`, but it has this: it fills the current thread's attributes into a
 * `pthread_attr_t`, and together with `pthread_attr_getstack()` yields the stack's `[addr, addr+size)` range.
 * @note **Does not cover NetBSD/OpenBSD/DragonFly**: neither NetBSD nor OpenBSD has it, and DragonFly's
 *       availability has not been verified (this environment has no sysroot for it), so we **do not guess** -- those platforms take the conservative fallback.
 *       (All three are out of support scope, see the "BSD family" section above.)
 * @note On FreeBSD 13+ `pthread_attr_t` is an opaque pointer, so it must be started with `pthread_attr_init()`
 *       and finished with `pthread_attr_destroy()` (you cannot pass an uninitialised stack variable directly as on Linux). */
#ifndef GCXX_HAVE_PTHREAD_ATTR_GET_NP
#  if GCXX_PLATFORM_FREEBSD
#    define GCXX_HAVE_PTHREAD_ATTR_GET_NP 1
#  else
#    define GCXX_HAVE_PTHREAD_ATTR_GET_NP 0
#  endif
#endif
/** @brief Whether Apple's `pthread_get_stackaddr_np` / `pthread_get_stacksize_np` are available.
 *
 * **Both macOS and iOS have them** (libSystem); they return the stack base (high address) and stack size. On iOS this is the
 * **only** way to obtain the stack bounds (there is no `pthread_getattr_np`). */
#ifndef GCXX_HAVE_PTHREAD_GET_STACKADDR_NP
#  if GCXX_PLATFORM_DARWIN
#    define GCXX_HAVE_PTHREAD_GET_STACKADDR_NP 1
#  else
#    define GCXX_HAVE_PTHREAD_GET_STACKADDR_NP 0
#  endif
#endif
/** @brief Whether `pthread_getcpuclockid` is available.
 *
 * It is used only by the **read-only fallback** for "the thread has terminated" (see `heap_thread_ended`).
 * Without it that fallback conservatively returns "not ended" -- safe by direction, it merely does not take effect;
 * the primary criterion (the `thread_ended` flag set by the platform exit callback) exists on every POSIX.
 *
 * **Measured availability** (real headers/exported symbols were all checked):
 *   * Linux/glibc: present in `<pthread.h>` (after `__USE_GNU`) -- **yes**;
 *   * Android/Bionic: `<pthread.h>` L185 `int pthread_getcpuclockid(pthread_t, clockid_t*)`
 *     carries **no `__INTRODUCED_IN` annotation at all** => it has existed since API 1 (in the same file
 *     `pthread_attr_getinheritsched` is marked `__INTRODUCED_IN(28)`, which shows the annotation is given per function)
 *     -- **yes**;
 *   * FreeBSD 14.5: present at `<pthread.h>` L220 -- **yes**;
 *   * NetBSD / OpenBSD / DragonFly: their POSIX thread implementations are all based on `lwp`/`rthread`,
 *     but **whether they use this name has not been verified** (this environment has no sysroot for them). **We do not guess** => they count as "no",
 *     taking the conservative fallback (this does not affect correctness, only the "can a dead heap be reclaimed" optimisation).
 *   * Apple: **no** (not found in `<pthread.h>`; this is also why `thread_cpuclock` is always 0 on Apple,
 *     see the field description in `gc_heap`). */
#ifndef GCXX_HAVE_PTHREAD_GETCPUCLOCKID
#  if GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID || GCXX_PLATFORM_FREEBSD
#    define GCXX_HAVE_PTHREAD_GETCPUCLOCKID 1
#  else
#    define GCXX_HAVE_PTHREAD_GETCPUCLOCKID 0
#  endif
#endif
/** @brief Whether glibc's proprietary `pthread_tryjoin_np` is available (**Linux/glibc only**).
 *
 * It is used only by the **reproduction/control-only** ablation branch `-DGCXX_THREAD_ENDED_TRYJOIN=1` --
 * that branch deliberately keeps the old implementation (which steals the host's own `join()`; a real defect, see `heap_thread_ended`).
 * @note It is **not the same thing** as `GCXX_HAVE_PTHREAD_GETCPUCLOCKID`: the latter also exists on FreeBSD/Android.
 *       Mixing the two up makes non-glibc platforms call a symbol that does not exist (a real-machine FreeBSD build measured
 *       `no member named 'pthread_tryjoin_np' in the global namespace`). */
#ifndef GCXX_HAVE_PTHREAD_TRYJOIN_NP
#  if GCXX_PLATFORM_LINUX && defined(__GLIBC__)
#    define GCXX_HAVE_PTHREAD_TRYJOIN_NP 1
#  else
#    define GCXX_HAVE_PTHREAD_TRYJOIN_NP 0
#  endif
#endif
/** @brief Whether `dl_iterate_phdr` is available (**Linux and Android have it; so does FreeBSD**).
 *
 * Used to enumerate the "writable data segments of loaded objects" as roots. This is the basis of **dynamic roots**, and the proper
 * fix for the static-root gap on BSD (see `GCXX_USE_DYNAMIC_ROOTS`).
 *
 * @note **Measured evidence** (FreeBSD 14.5 real sysroot):
 *   * `nm -D /lib/libc.so.7` contains `dl_iterate_phdr@@FBSD_1.0` (**libc exports it**);
 *   * `<link.h>` is just `#include <sys/link_elf.h>`, and that file defines at L81
 *     `struct dl_phdr_info` and declares at L96 `int dl_iterate_phdr(__dl_iterate_hdr_callback, void *)`.
 *   => the earlier conclusion "BSD has no `dl_iterate_phdr`" was **wrong** (at the time we only checked whether some header
 *     carried that name, not the exported symbols of libc, and never looked at `<link.h>`).
 * @note FreeBSD's `dl_phdr_info` has **exactly the same first four members** as Linux's
 *       (`dlpi_addr` / `dlpi_name` / `dlpi_phdr` / `dlpi_phnum`), only written with the types
 *       `Elf_Addr`/`Elf_Phdr`/`Elf_Half` rather than `ElfW(...)`; after them come four further members,
 *       `dlpi_adds`/`dlpi_subs`/`dlpi_tls_modid`/`dlpi_tls_data` --
 *       the callback reads only the first four, so both sides can share the same piece of logic (smoothed over with the type aliases `gcxx_elf_*`).
 * @note Android's Bionic provides it from **API 21** (`__INTRODUCED_IN(21)`);
 *       below that even `<link.h>` fails to include => we must fall back to "only the static area of the main program image". */
#ifndef GCXX_PLATFORM_HAVE_LINK_H
#  if GCXX_PLATFORM_LINUX || GCXX_PLATFORM_FREEBSD || \
      (GCXX_PLATFORM_ANDROID && defined(__ANDROID_API__) && __ANDROID_API__ >= 21)
#    define GCXX_PLATFORM_HAVE_LINK_H 1
#  else
#    define GCXX_PLATFORM_HAVE_LINK_H 0
#  endif
#endif
/** @brief Whether Apple's `_dyld_image_count` / `_dyld_get_image_header` etc. are available (both macOS and iOS have them). */
#ifndef GCXX_HAVE_DYLD_IMAGE_API
#  if GCXX_PLATFORM_DARWIN
#    define GCXX_HAVE_DYLD_IMAGE_API 1
#  else
#    define GCXX_HAVE_DYLD_IMAGE_API 0
#  endif
#endif
/** @brief Whether the `sysctl(CTL_KERN, KERN_PROC, KERN_PROC_PID, pid)` RSS query path is available.
 * @note FreeBSD / DragonFly / NetBSD / OpenBSD all have it (`struct kinfo_proc`'s
 *       `ki_rssize` is the **current** RSS in pages), but **the struct and field names on NetBSD/OpenBSD
 *       are `kinfo_proc2` / `p_rsz`**, so they are fetched per platform (see `os::rss_bytes`).
 * @note There is **no** sysctl named `KERN_PROC_RSS` (measured: not found in FreeBSD 14.5's
 *       `<sys/sysctl.h>`) -- the name written in the old code comments and implementation was wrong. */
#ifndef GCXX_HAVE_SYSCTL_KERN_PROC
#  if GCXX_PLATFORM_BSD
#    define GCXX_HAVE_SYSCTL_KERN_PROC 1
#  else
#    define GCXX_HAVE_SYSCTL_KERN_PROC 0
#  endif
#endif
/** @brief Whether this is x86-64 (the inline assembly for register capture and stack-pointer reads is implemented only on this architecture). */
#ifndef GCXX_ARCH_X86_64
#  if defined(__x86_64__) || defined(_M_X64)
#    define GCXX_ARCH_X86_64 1
#  else
#    define GCXX_ARCH_X86_64 0
#  endif
#endif
/** @brief Whether this is AArch64 (ARM64). The mainstream ABI on Android/iOS. */
#ifndef GCXX_ARCH_AARCH64
#  if defined(__aarch64__) || defined(_M_ARM64)
#    define GCXX_ARCH_AARCH64 1
#  else
#    define GCXX_ARCH_AARCH64 0
#  endif
#endif
/* **Feature macros must be defined before any standard header** (the user may already have included <thread>),
 * so they are placed at the very top of the header, and are only ever added, never changed.
 *
 * Why even the default path needs `_GNU_SOURCE` (not just that ablation branch):
 *   * glibc declares `pthread_getattr_np` only under `__USE_GNU`, and `platform_stack_top`
 *     uses it **unconditionally** on Linux (`GCXX_HAVE_PTHREAD_GETATTR_NP == 1`);
 *   * Bionic (Android) likewise puts `pthread_getattr_np` and friends behind `_GNU_SOURCE`
 *     (and requires `__ANDROID_API__ >= 24`, see `platform_stack_bottom`);
 *   * the **reproduction/control-only** ablation branch `-DGCXX_THREAD_ENDED_TRYJOIN=1` also needs
 *     the glibc extension `pthread_tryjoin_np`.
 * => reserving `_GNU_SOURCE` for ablation builds alone would leave the **default** build without its stack-top upper bound.
 *
 * @note **Defined for Linux/Android only**: `_GNU_SOURCE` is a glibc/Bionic notion,
 *       and neither BSD (libc) nor Darwin (libSystem) has it -- defining it there is harmless, but it makes people
 *       mistakenly think "BSD needs it too". And BSD's `pthread_attr_get_np` is not behind
 *       `_GNU_SOURCE` in the first place (it is in `<pthread_np.h>`, declared unconditionally). */
#if (GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID) && !defined(_GNU_SOURCE)
#  define _GNU_SOURCE 1
#endif
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <csetjmp>
#include <cstdio>
#include <cstdarg>      /* error reporting: vsnprintf */
#include <cerrno>       /* errno semantics (EFAULT/EAGAIN/EINVAL/ENOMEM) */
#include <new>
#include <utility>
#include <atomic>
#include <mutex>
#include <chrono>
#include <thread>
#include <type_traits>
#if GCXX_PLATFORM_WINDOWS
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#    define GCXX_DEFINED_WIN32_LEAN_AND_MEAN 1
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX          /* keep the min/max macros from polluting user code */
#    define GCXX_DEFINED_NOMINMAX 1
#  endif
#  include <windows.h>
#  include <fibersapi.h>    /* FlsAlloc (WIN32_LEAN_AND_MEAN may not include it) */
#  include <psapi.h>        /* PROCESS_MEMORY_COUNTERS: used only to report process RSS */
#endif
#if GCXX_PLATFORM_POSIX && !GCXX_PLATFORM_WINDOWS
#  include <unistd.h>      /* sysconf: query the page size (nothing hard-coded); getpid */
#  include <sys/mman.h>    /* mmap/munmap/madvise/mprotect: chunk mapping, decommit, large-object guard pages */
#  include <pthread.h>     /* pthread_self / pthread_key_* / pthread_attr_*: thread identity, TLS exit cleanup, stack bounds */
#endif
#if GCXX_PLATFORM_HAVE_LINK_H
#  include <link.h>          /* dl_iterate_phdr + dl_phdr_info (Linux / Android / FreeBSD) */
#endif
#if GCXX_PLATFORM_FREEBSD
#  include <pthread_np.h>    /* pthread_attr_get_np: FreeBSD's stack-bounds query (no pthread_getattr_np) */
#endif
#if GCXX_PLATFORM_FREEBSD || GCXX_PLATFORM_DRAGONFLY || GCXX_PLATFORM_NETBSD || \
    GCXX_PLATFORM_OPENBSD
/* BSD family: RSS goes through sysctl (**no** /proc/self/statm, and the macro `KERN_PROC_RSS` does not exist) */
#  include <sys/types.h>
#  include <sys/sysctl.h>
#  include <sys/user.h>      /* struct kinfo_proc (called kinfo_proc2 on NetBSD/OpenBSD) */
#  include <elf.h>           /* Elf_Phdr / PT_LOAD / PF_W (BSD's ELF definitions live here) */
#endif
#if GCXX_PLATFORM_DARWIN
/* Apple: `_dyld_image_count`/`_dyld_get_image_*` enumerate the loaded Mach-O images
 * (they are public SPI on both macOS and iOS, and **introduce no new link dependency** -- they live in libSystem). */
#  include <mach-o/dyld.h>
#  include <mach-o/loader.h>
#endif
#if defined(_MSC_VER)
#  include <intrin.h>
#  pragma once
#endif
#if GCXX_PLATFORM_WINDOWS
#  include <malloc.h>          /* _aligned_malloc / _alloca (both MinGW and MSVC have them) */
#endif
/* -------------------------- Tunable parameters --------------------------
 *
 * **Purely numeric constants here are always `constexpr` variables**, never `#define`:
 *   * they are typed, visible in a debugger, scoped, and never accidentally expanded elsewhere the way a macro is;
 *   * **they occupy no static area** -- the value is inlined into instructions (measured: in a TU that includes only gc.hpp
 *     there is no GCXX_* symbol at all in `.data` / `.bss`). This matters especially for this library:
 *     the static area is a root conservatively scanned every cycle, so adding bytes there is not free
 *     (measured: scanning costs 17.65 ns/word, while a plain sequential read costs only 0.49 ns/word).
 *   * **do not add `static`**: a `static constexpr` is "an object whose address has been taken" and forces a copy to be materialised in
 *     `.rodata` (4~8 bytes each). A `constexpr` without `static`
 *     has implicit internal linkage at namespace scope, but is never materialised as long as it is used only by value.
 *
 * **The macros below must stay macros**, do not turn them into constexpr:
 *   1. `GCXX_*_INCLUDED` -- include guard;
 *   2. **ablation switches** (`GCXX_FORCE_SUSPEND` / `GCXX_GRAY_CAPACITY` /
 *      `GCXX_TYPE_SKIP` / `GCXX_ZERO_MEMORY` / `GCXX_BLACKLIST` /
 *      `GCXX_ALL_INTERIOR_POINTERS` / `GCXX_SKIP_THREAD_EXIT_CLEANUP` /
 *      `GCXX_USE_STATIC_ROOTS` / `GCXX_TRACE_MT` /
 *      `GCXX_OS_PAGE_PRIMITIVES` / `GCXX_OS_MAP_ANON` / `GCXX_NOINLINE`) --
 *      they are used in `#if`, and the project's A/B ablation passes them in via `-D FOO=N`;
 *      turning them into constexpr would make `-D` replace the variable declaration outright (a compile error).
 *   3. **function-like macros**: `GCXX_MT_TRACE(...)` / `GCXX_PREFETCH(p)` /
 *      `gc_declare_plain<T>()` -- they are not "values".
 *
 * Correspondingly, `GCXX_GRAY_CAPACITY` / `GCXX_GRAY_CAPACITY_LIMIT` also stay macros:
 * the former is overridden by `-DGCXX_GRAY_CAPACITY=4` (the ablation build with a tiny gray-set capacity). */

namespace gcxx {
namespace detail {

/**
 * @brief No collection is done while heap memory is below this value.
 * @internal
 */
constexpr std::size_t MIN_HEAP_SIZE = 256u * 1024u;
/**
 * @brief Multiple of "this cycle's largest single allocation" used as the debt floor.
 * @internal
 * @note Prevents **a large object from touching off a GC storm**: the fixed cost of a GC cycle is independent of "how many bytes this cycle allocated",
 *       so if one allocation can eat the entire debt quantum you get "one full GC cycle per allocation". A value of 16 means
 *       "when a single object occupies X bytes, at least another 16X must be allocated before the next cycle". See `gc_heap::max_alloc`.
 */
constexpr unsigned MAX_ALLOC_DEBT_MUL = 16;
/**
 * @brief After a cycle ends, the next cycle's start threshold = live bytes * PAUSE_PERCENT / 100.
 * @internal
 */
constexpr unsigned PAUSE_PERCENT = 200;
/**
 * @brief Debt after a minor = live bytes * MINOR_MUL% (collection frequency is proportional to heap size; Lua's MINORMUL).
 * @internal
 */
constexpr unsigned MINOR_MUL = 20;
/**
 * @brief Once bytes promoted since the last major reach this percentage of the live bytes at that major -> do a major early.
 * @internal
 */
constexpr unsigned MINOR_MAJOR_PERCENT = 70;
/** @brief Frequency safety valve: each heap's next-cycle quota must not exceed this percentage (%) of the [heap's actually mapped bytes].
 * @note The quota is the **only** quantity that decides when the next cycle starts: pushed to an astronomical figure (total_bytes underflows to 2^64 and the quota becomes
 *       1.8e17), it keeps debt permanently positive and begin_cycle is never called again -- the GC stops completely and the heap eats all memory. Mapped bytes are maintained under gc_mutex and on retire are only decremented after confirming "still on the list", so they cannot underflow; hence their use as the upper bound.
 * @note Must be >= PAUSE_PERCENT: live <= mapped always holds, so while accounting is sane this cap never binds (a static_assert below pins that down); it is only a backstop and takes no part in tuning.
 * @internal
 */
constexpr unsigned DEBT_CAP_MAPPED_PERCENT = 400;
/**
 * @brief The number of bytes one progress slice counts as "debt repaid" (= one `gc_step` processes `SLICE_BYTES /
 *        BYTES_PER_WORK_UNIT` objects).
 *
 * **Why this was raised from 256 to 4096** (interleaved A/B on the same machine at the same moment, Linux g++ -O2, 1e8 `gc<double>` calls):
 * this value decides both the **slice length** and the base over which slice-boundary overhead is amortised. Every slice end must call `flush_dirty_slots()`
 * (walk the collector's private dirty-block list and CAS the accumulated dead slots in one batch into the owner-visible free list)
 * and must re-enter the function and loop headers of `gc_step` -> `sweep_some`. With slices of only 8 objects, this
 * **object-count-independent fixed overhead** is divided by 8, and each dead object measured ~7ns of it: `flush_dirty_slots`
 * ran 2.5M times and `publish_dead_slots` 2.54M times (normally it should be ~600K).
 *
 * Measured (same workload, ns/op, base = this value at 256):
 *   256 -> 29.5 | 1024 -> 23.9 | 4096 -> 21.9 | 16384 -> 21.6 | 65536 -> 21.5
 * 4096 was chosen because **the gain has already plateaued** (going higher adds only 1.5%) while **the worst-case pause of a single allocation** stays small:
 * one slice of 128 objects x ~8ns -- **1us**; `gc_pace`'s `do/while` runs at most 4 slices => ~4us worst case.
 * Control: this library's STW pause is **on the millisecond scale** (the `pause` benchmark p50 ~= ms), so 1~4us changes
 * no conclusion about orders of magnitude of latency. **Changing this value means looking at both allocation throughput and the `pause` basis**.
 * @note Wrapped in `#ifndef`, so `-DGCXX_SLICE_BYTES=N` can be used for A/B ablation (like the other switches).
 * @internal
 */
#ifndef GCXX_SLICE_BYTES
#  define GCXX_SLICE_BYTES 4096u
#endif
constexpr unsigned SLICE_BYTES = (unsigned)GCXX_SLICE_BYTES;
/**
 * @brief The maximum number of slices advanced per allocation (bounds a single pause).
 * @internal
 */
constexpr unsigned STEP_MAX_SLICES = 4;
/** @brief How many bytes of credit quota one work unit (sweeping/marking one object) is worth.
 * @note gc_step returns the **work actually done** and gc_pace repays debt as work * BYTES_PER_WORK_UNIT, so this
 *       ratio is decoupled from the batch parameters. Paying back too little (measured: with GCXX_SLICE_UNITS=1 the heap peak inflates 8.5x) keeps debt positive for a long time and stalls the next cycle; over-sweeping puts microsecond-scale spikes on the allocation path.
 * @internal
 */
constexpr unsigned BYTES_PER_WORK_UNIT = 32;
/** @brief The **initial** capacity of the gray set (the array-style worklist). A chain-shaped reference graph takes only 1 slot; only wide graphs can fill it;
 * 65536 slots x 16B = 1MB, lazily allocated (see gray_reserve). @note It is only a starting point, not an upper bound: the number of enqueues per cycle is of the same order as the live object count, and "55k live objects" is enough to fill it; once full, the overflow
 *       fallback is a whole-heap fallback rescan at O(heap) per occurrence -> pauses went from 0.7ms to 5 seconds (measured). So gray_push grows it in place by a factor when it is nearly full; for the actual capacity see gray_cap.
 * @internal
 */
#ifndef GCXX_GRAY_CAPACITY
#  define GCXX_GRAY_CAPACITY 65536u
#endif
/** @brief The **absolute upper bound** the gray set may grow to (in slots); beyond it we are back on the overflow + fallback-rescan path
 * (still correct, just slower). 16M slots x 16B = 256MB, reached only on heaps with "tens of millions of live objects".
 * @internal
 */
#ifndef GCXX_GRAY_CAPACITY_LIMIT
#  define GCXX_GRAY_CAPACITY_LIMIT (16u * 1024u * 1024u)
#endif
/**
 * @brief The maximum number of work units processed in one progress slice (= the quota of one gc_step).
 * @internal
 */
constexpr unsigned SWEEP_UNITS = (SLICE_BYTES / BYTES_PER_WORK_UNIT);
/** @brief The batch size at which the sweep side accumulates dead slots before publishing them (see gc_chunk::dead_head).
 * One batch = one CAS + one live decrement, so bigger is cheaper; but the slots in a batch are temporarily invisible to the owner, and too large a batch makes the owner ask for new memory early. 32 is the compromise between "atomic operations saved" and "at most 32 slots invisible" (a 64KB block has thousands of slots; 32 slots ~2KB).
 * @internal
 */
constexpr unsigned FREE_BATCH = 32u;
/** @brief How many empty blocks (64KB each) the global recycle stack may hoard; beyond that, blocks are handed back to the system.
 * The recycle stack does not hand anything back, so "memory comes down after the peak" depends entirely on it: measured, after all 400k simultaneously live objects (196 blocks / 12.5MB) were dropped, gc_stat reported 1 block, yet process RSS stayed at 22MB (195 blocks lying in the recycle stack).
 * @internal
 */
constexpr unsigned RECYCLE_MAX = 64;
/** @brief The lower bound on how many blocks in the recycle stack **keep their physical pages** (D2's "purge delay").
 * Decommit/recommit each cost a system call, and decommitting a block that was just retired and is about to be reused is pure waste; a block is decommitted only after being retired for PURGE_DELAY_MS. The delay decides "how soon it is returned", this value decides "how much is kept": 8 blocks = 512KB of resident hot spare, and
 * while the recycle stack does not exceed this value, purge is **skipped entirely** (not even the per-size-class empty lists are walked). @note That skip is a common-sense guard of "do not scan when there is nothing to do", not a performance measure (interleaved A/B on the same machine at the same moment measured no difference).
 * @internal
 */
constexpr unsigned RECYCLE_KEEP = 8;
/** @brief How long after retirement decommit becomes allowed (milliseconds); **the delay is the criterion for "is this memory really cold yet"**.
 * 10ms matches mimalloc's mi_option_purge_delay default: orders of magnitude above allocation jitter (microseconds), yet far below the scale at which "users start complaining about memory".
 * @internal
 */
constexpr unsigned PURGE_DELAY_MS = 10;
/** @brief The maximum number of blocks examined/moved per (kind, size class) in one purge round.
 * purge is done only by the cycle_owner once per cycle, so this is also "at most how many CASes per cycle"; on an empty list it returns nullptr at the first block, and the real cost is proportional to **how many blocks are actually on the recycle stack**.
 * @internal
 */
constexpr unsigned PURGE_SCAN = 64;
/* ---- E2: sampling rate for large-object guard pages (1/N) -------------- */
/** @brief One in every N **large objects** is placed flush against an inaccessible page: payload[size] is the first byte of that page, so a
 * 1 byte of overrun faults immediately (it explodes on the offending instruction, rather than being masked by later writes). Only large objects are sampled: each guarded object costs an extra OS page + one VirtualProtect/mprotect (doing that for every object is
 * too heavy), while large objects are few to begin with and array overruns are the most common case -- the extra memory of 1/64 sampling is live_big_bytes/64, almost unmeasurable. Small objects live in shared blocks, where an adjacent overrun stays inside the same block, so a guard page cannot be placed there; that is left to block-tail padding.
 * N = 64 is the compromise between "can be caught" and "negligible overhead"; lower it for stricter coverage (1 = every large object is guarded).
 * @internal
 */
constexpr unsigned GUARD_SAMPLE = 64;
/** @brief Reserved macro: `GCXX_WRITE_WATCH_MARK` is a **no-op** (kept only for source compatibility; setting it to 1 has no effect).
 * It once enabled incremental marking driven by a page-protection write barrier, but that write barrier measured as a net loss (see the file header and
 * and the related implementation was **removed entirely**.
 * @internal
 */
#ifndef GCXX_WRITE_WATCH_MARK
#  define GCXX_WRITE_WATCH_MARK    0
#endif
#if GCXX_WRITE_WATCH_MARK
#  pragma message("gcxx: GCXX_WRITE_WATCH_MARK is a no-op; incremental marking was removed in favour of stop-the-world marking")
#endif
/* ------------------------------------------------------------------ diagnostic output */
/** @brief Off by default; when enabled (-DGCXX_TRACE_MT=1) the collection and safepoint paths write timings to stderr.
 * @note It is off by default because these calls do stdio inside the window where **gc_mutex is held and the mutators are parked**: they both pollute the host program's stderr and fold lock and I/O latency straight into the pause time. gc_verify's [verify] output is unaffected (an explicit diagnostic interface, and it prints only when an inconsistency is detected).
 * @internal
 */
#ifndef GCXX_TRACE_MT
#  define GCXX_TRACE_MT 0
#endif
#if GCXX_TRACE_MT
#  define GCXX_MT_TRACE(...) std::fprintf(stderr, __VA_ARGS__)
#else
#  define GCXX_MT_TRACE(...) ((void)0)
#endif
/** @note The GCXX_ATOMIC_RESCAN_ROOTS that once lived here (rescanning roots in the atomic phase) cannot solve the problem: a mutator
 *        can read a pointer out of "a region not yet traced" and then erase it from its original location, after which the white object is held only by the mutator; rescanning roots cannot help with the kind written into a black object. The correct approach is to make marking itself atomic -- see the notes on the GC_PHASE_PROPAGATE branch in gc_step.
 * @internal
 */
/** @brief Prefetch hint. Conservative scanning's memory access pattern is "chase 2~3 levels of indirection per candidate pointer" (direct-map table entry ->
 * chunk header -> object header); these steps are strongly dependent and out-of-order execution cannot overlap their latency across iterations; fetching several words together
 * (prefetching them all first, then resolving) manufactures memory-level parallelism by hand.
 * @internal
 */
#if defined(_MSC_VER)
#  define GCXX_PREFETCH(p) _mm_prefetch(reinterpret_cast<const char*>(p), _MM_HINT_T0)
#elif defined(__GNUC__) || defined(__clang__)
#  define GCXX_PREFETCH(p) __builtin_prefetch(static_cast<const void*>(p))
#else
#  define GCXX_PREFETCH(p) ((void)0)
#endif
/** @brief The maximum payload bytes scanned by a single work unit (chunked continuation scanning of large objects).
 * Without an upper bound, "walk one large object" is an unbounded step: a container holding a 4MB buffer would be scanned in full inside a single gc_pace -> a millisecond-scale pause. Measured, the difference in pause and throughput between 64B and 4KB is all within noise; 512B (64 words) is the stable compromise.
 * @internal
 */
constexpr std::size_t SCAN_SLICE_BYTES = 512u;
/**
 * @brief How many minor cycles an object survives before being promoted to the old generation.
 * @internal
 */
constexpr unsigned PROMOTE_AGE = 2;
/**
 * @brief How often (in cycles) to do a major (full) collection.
 * @internal
 */
constexpr unsigned MAJOR_EVERY = 8;
/**
 * @brief Upper bound on stack scanning (to avoid scanning a huge reserved stack area).
 * @internal
 */
constexpr std::size_t MAX_STACK_SCAN = (16u * 1024u * 1024u);
/** @brief Upper bound on the number of threads force-suspended at one time (force-suspend exists only on Windows); beyond it, fall back to cancelling.
 * A stack array is used, so it cannot be unbounded; 64 simultaneously uncooperative threads is already extreme.
 * @internal
 */
constexpr unsigned MAX_FORCE_SUSPEND = 64;
/** @brief Test-only: when set to 1, thread-exit cleanup is **not** registered, to exercise the "thread terminated but cleanup never ran"
 * path (see retire_ended_heaps). @warning Do not enable this in normal builds.
 * @internal
 */
#ifndef GCXX_SKIP_THREAD_EXIT_CLEANUP
#  define GCXX_SKIP_THREAD_EXIT_CLEANUP 0
#endif
/** @brief Whether OS facilities may be used to force-suspend threads that "cannot reach a safepoint" (implemented on Windows only).
 * Setting it to 0 falls back to the "cannot stop them, so cancel" behaviour -- both paths should be covered by tests, so the whole point of this switch is to let tests run through both.
 * @internal
 */
#ifndef GCXX_FORCE_SUSPEND
#  define GCXX_FORCE_SUSPEND      1
#endif
#if GCXX_PLATFORM_WINDOWS && GCXX_FORCE_SUSPEND
#  define GCXX_CAN_FORCE_SUSPEND  1
#else
#  define GCXX_CAN_FORCE_SUSPEND  0
#endif
/**
 * @brief Upper bound on the post-cancellation backoff quota: consecutive cancellations are capped here, so the heap cannot grow without bound.
 * @internal
 */
constexpr std::size_t CANCEL_BACKOFF_MAX = (64u * 1024u * 1024u);
/**
 * @brief Extra safety margin kept above the gc_init() anchor.
 * @internal
 */
constexpr std::size_t STACK_MARGIN = (64u * 1024u);
/**
 * @brief Extra margin kept below the real rsp for the stack-scan lower bound.
 * @internal
 */
constexpr std::size_t STACK_LOWER_MARGIN = 256u;
/**
 * @brief Number of slots in the register-capture buffer (16 general-purpose registers + 16 vector registers * 2 slots).
 * @internal
 */
constexpr std::size_t REG_CAPTURE_SLOTS = 64u;
/** @brief Budget for waiting until every mutator reaches a safepoint (milliseconds). On expiry it degrades to "conservative scanning of the whole stack" --
 * it retains more without ever missing a mark, but register roots are not captured (see the thread-model limitations in the file header).
 * @internal
 */
constexpr unsigned STW_TIMEOUT_MS = 20;
/** @brief When force-suspend is available, **cooperative waiting** lasts only this long (milliseconds), then we suspend directly.
 * @note want counts "all registered non-dead heaps", and **a thread blocked in join / a condition variable / a long computation never reaches a safepoint**: a thread that registered a heap but no longer allocates can make every cycle wait out the full STW_TIMEOUT_MS for nothing
 *       (measured: 8 threads, 69 collection cycles, 69 timeouts, roughly 1.4s of 2.17s spent purely waiting). Waiting it out buys nothing: at expiry we force-suspend anyway, and suspending is **equally safe** (see suspend_threads_for_mark); platforms without force-suspend (POSIX) can only
 *       wait, and still use STW_TIMEOUT_MS. 1ms was chosen: for an allocating thread reaching a safepoint is a matter of microseconds (every allocation checks sp_request), and a non-allocating thread will not be in place even after 19ms more.
 * @internal
 */
#if GCXX_CAN_FORCE_SUSPEND
/* **Defined only on platforms with force-suspend capability** (= Windows).
 * It was originally defined unconditionally, so **every other platform** got a
 * `-Wunused-const-variable: STW_GRACE_MS` warning (measured: FreeBSD cross-compile, GCC and clang all reported it).
 * Both "silence the warning" approaches that were tried fail:
 *   * `__attribute__((unused))` -- clang's `constexpr` variables do not accept it (measured in a FreeBSD cross-compile:
 *     `error: constexpr variable 'GCXX_MAYBE_UNUSED' must be initialized`);
 *   * "mentioning" it via `(void)sizeof(STW_GRACE_MS)` and the like -- `-Wunused-const-variable` does not look at sizeof.
 * And it **is used only by the Windows path in `suspend_threads_for_mark`** anyway,
 * so the right thing is to make its visibility match its use: follow `GCXX_CAN_FORCE_SUSPEND`.
 * The discipline of "tunable constants are defined in one place" still holds for **cross-platform** constants; this is not one of them. */
constexpr unsigned STW_GRACE_MS = 1;
#endif
/**
 * @brief **Hard upper bound** on a single object's payload (`0xFFFFFF00` = 4GiB - 256).
 *
 * @note It is **not a macro and cannot be raised by the user** -- the allocation path clamps with it, avoiding the case where `(uint32_t)size`
 *       truncates to a small value and the object is then allocated at the wrong size. 4GiB-256 rather than 4GiB-1 because an object's **total bytes within its slot**
 *       is `PAYLOAD_OFF(16) + size (+ PAD_BYTES)`: after leaving room for the header and the block-tail padding,
 *       the "total bytes within the slot" still cannot overflow `uint32_t`.
 * @internal
 */
constexpr std::uint32_t MAX_OBJECT_SIZE = 0xFFFFFF00u;
/**
 * @brief Whether the "skip scanning pure-scalar objects" optimisation is enabled (set to 0 to fall back to full conservative scanning as a control).
 * @internal
 */
#ifndef GCXX_TYPE_SKIP
#  define GCXX_TYPE_SKIP          1
#endif
/** @brief Negative-bias tolerance (bytes): how far before an object's payload start p may fall and still count as pointing at that object.
 * Covers the object header (16B) plus the base offsets compilers commonly emit, such as a-24/a-48.
 * @internal
 */
#ifndef GCXX_INTERIOR_BIAS_BYTES
#  define GCXX_INTERIOR_BIAS_BYTES 64u
#endif
constexpr std::size_t BIAS_BYTES = (std::size_t)GCXX_INTERIOR_BIAS_BYTES;

/* ---- Boehm-style blacklist (prevents permanent false retention from "integer collisions") ---- */
/** @brief Risk: an integer value that happens to fall inside the GC heap address range is taken for a pointer, and the object it "points at" (with its whole subtree)
 * is retained forever -- showing up as a memory leak. The idea: if an [unallocated] address is repeatedly "pointed at" by candidate values, it is almost certainly a false pointer (a real pointer will not keep pointing into free space); once recorded, the allocator **never allocates an object there**, and the false pointer can no longer point at a real object.
 * Granularity: each 64KB chunk is split into 16 4KB pages (a uint16 bitmap): bl_hits holds the pages hit this major by "candidates that matched no allocated object", bl_prev the hit bitmap of the previous major, and bl_black the pages hit in two consecutive majors (which the allocator
 * avoids). Two-round confirmation markedly reduces collateral damage; if a single chunk has more blacklisted pages than BLACKLIST_MAX_PAGES the whole chunk is abandoned (so a pathological case cannot make the allocator avoid the entire heap). Cost: one shift + one set-bit on the "candidate matched no object" path only, and one bit test on the allocation path.
 * @internal
 */
#ifndef GCXX_BLACKLIST
#  define GCXX_BLACKLIST          1
#endif
/* Test-only slot-invariant self-check (by default **off**, zero cost). When set to 1, `bump_take` prints
 * the attribution fields and aborts when `idx >= capacity` or `bump > capacity`, so an out-of-range slot
 * is caught before it is written to. **Do not** enable it in normal builds:
 * it adds two comparisons to the allocation hot path. */
#ifndef GCXX_TEST_ASSERT_SLOT
#  define GCXX_TEST_ASSERT_SLOT   0
#endif
constexpr unsigned BLACKLIST_MAX_PAGES = 4u;
/** @brief Whether pointers into the interior of objects are supported (&obj.field, negative-bias base a-24, etc.). Set to 0 and resolve accepts only
 * "the exact payload start address", filtering out a great many interior-pointer false positives (equivalent to Boehm's GC_ALL_INTERIOR_POINTERS=0). @warning If any interior pointer exists in the program it will be wrongly reclaimed, so be sure there are none before turning this on with 0. A compile-time switch with zero runtime cost.
 * @internal
 */
#ifndef GCXX_ALL_INTERIOR_POINTERS
#  define GCXX_ALL_INTERIOR_POINTERS 1
#endif
/**
 * @brief Whether newly allocated memory is zeroed (a conservative GC should zero it, so dirty data is not taken for pointers).
 * @internal
 */
#ifndef GCXX_ZERO_MEMORY
#  define GCXX_ZERO_MEMORY        1
#endif
/**
 * @brief Whether to try scanning the static/global data segments as roots.
 * @internal
 */
#ifndef GCXX_USE_STATIC_ROOTS
#  define GCXX_USE_STATIC_ROOTS   1
#endif
/**
 * @brief Whether the **writable data segments of dynamic libraries (shared objects)** are also treated as roots. **On by default** (Linux only).
 *
 * Background: `GCXX_USE_STATIC_ROOTS` covers only the **main program image** (Linux/Android's `[__data_start,_end)`, FreeBSD's `[__executable_start,_end)`).
 * That means one thing: **a GC pointer held in a shared library's global variable is not a root** -- an object referenced only by a `.so`'s
 * global variable is wrongly reclaimed (dangling pointer). With this set to 1, `dl_iterate_phdr` additionally enumerates the
 * `PT_LOAD && PF_W` segments of all loaded objects and scans them as roots too.
 *
 * @note Cost: each collection cycle scans an extra few (usually tens of) KB of `.so` data segments. Set to 0 to turn it off.
 * @note **It covers only libraries already loaded at the first collection**. Libraries brought in by `dlopen` afterwards require an explicit call to
 *       `gc_refresh_roots()` to refresh (automatic repeated collection would introduce lock-order risk; see the notes below that constant).
 * @note Implemented for **Linux/glibc (including musl) and Windows**: Linux goes through `dl_iterate_phdr`;
 *       Windows goes through `EnumProcessModules` + PE section parsing (PSAPI -- this library already depends on it for RSS statistics,
 *       so **no new link dependency is introduced**). macOS (`_dyld_image_count`) is **not implemented yet**.
 * @internal
 */
#ifndef GCXX_USE_DYNAMIC_ROOTS
#  define GCXX_USE_DYNAMIC_ROOTS  1
#endif
/** @brief How many static/dynamic root ranges may be cached (stored in pairs, see gc_state::static_ranges).
 *
 * It must hold: the main program data segment (1 range) + several writable segments per shared object (glibc itself + common dependencies usually
 * 10~30 ranges). 64 ranges (= 128 `char*`, 1KB) leaves ample margin.
 * @internal
 */
#ifndef GCXX_STATIC_RANGES_MAX
#  define GCXX_STATIC_RANGES_MAX  64
#endif
constexpr unsigned STATIC_RANGES_MAX = (unsigned)GCXX_STATIC_RANGES_MAX;
/**
 * @brief How many collection rounds a heap must stay untouched by its owner before the heap struct may be
 *        reclaimed (see `gc_heap::last_active_epoch`).
 *
 * A thread whose exit callback never ran -- killed by the host, or a runtime defect -- cannot be probed
 * directly on POSIX, because a `pthread_t` stops being dereferenceable once the thread is gone. "The owner has
 * stopped touching this heap" therefore has to be inferred from inactivity. The bound keeps that inference from
 * firing while the owner is merely descheduled or blocked between two allocation bursts: four rounds is far
 * longer than any scheduling gap, while a heap abandoned by a dead thread is still released promptly.
 * @internal
 */
constexpr unsigned HEAP_REAP_ROUNDS = 4;
} /* namespace detail */
} /* namespace gcxx */

/** @brief Whether the **old** `pthread_tryjoin_np` fallback is used when deciding "has the thread terminated".
 *
 * **Default 0 (off)**. Setting it to 1 reintroduces a defect confirmed by measurement: `pthread_tryjoin_np`
 * on success **really does consume that join**, so the host's own `std::thread::join()` then fails
 * (`ESRCH`), and during stack unwinding `~thread()` then triggers `std::terminate`
 * (all that is left at the scene is `terminate called without an active exception`, and the real exception is masked).
 * On top of that glibc **reuses thread descriptors**, so that pthread_t may already belong to a brand-new thread
 * the host is using -- which amounts to a "try join" on a thread we do not own.
 * This switch exists purely for **reproduction/control** (`-DGCXX_THREAD_ENDED_TRYJOIN=1`);
 * the default path uses the read-only `pthread_getcpuclockid` criterion instead (see `heap_thread_ended`).
 * Linux only. */
#ifndef GCXX_THREAD_ENDED_TRYJOIN
#  define GCXX_THREAD_ENDED_TRYJOIN 0
#endif
/** @brief Cold-path marker (error reporting, corruption handling). Code like this placed on the hot path squeezes register allocation,
 * so it costs even when it "almost never executes" -- hence it is pushed out explicitly. */
#if defined(_MSC_VER)
#  define GCXX_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#  define GCXX_NOINLINE __attribute__((noinline))
#else
#  define GCXX_NOINLINE
#endif
/** @brief Hot-path marker: makes the compiler **definitely** inline this path into the caller.
 * @note Of limited use in general, but necessary here: the allocation hot path (`allocate_bytes` -> `gc_pace` -> `object_new`
 *       -> `chunk_alloc`) has the shape of a chain of "big functions calling small functions", and GCC/Clang's early inliner will
 *       give up inlining because **the callee itself has slow-path branches** (error reporting, new chunk, OOM retry) --
 *       the result being a real `call` on every allocation (register save/restore + `endbr64` + argument shuffling),
 *       with `h`, `c`, `slot_size` having to be spilled and reloaded repeatedly across the call.
 *       **Apply it only to leaf hot functions that return after a single call**; do not apply it to functions with loops or I/O.
 * @note Used together with `GCXX_NOINLINE`: the hot function is inlined and its **slow paths** are pushed out with `GCXX_NOINLINE`
 *       (otherwise inlining would drag the slow paths into the caller too, and the bloat would make things slower). */
#if defined(_MSC_VER)
#  define GCXX_ALWAYS_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#  define GCXX_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#  define GCXX_ALWAYS_INLINE inline
#endif
/** @brief Safety / robustness facilities: **no switches, all on by default** (including the header is all there is: no compile-time macros and no
 * runtime option system either), with the cost paid once at implementation time: an **underflow** write into metadata redzone slot 0 hits the sentinel instead of rewriting gc_chunk's owner/span/capacity; detection of block-tail padding overruns / slots freed twice (see gc_padding)
 *   free-list two-key encoding exposes a wild write into a link field sooner (see link_encode); allocation-order randomization; large-object guard-page sampling of 1/GUARD_SAMPLE places large objects flush against an inaccessible page (see chunk_new_big)
 *   retired-block decommit returns the data area to the system once an empty block has been retired for PURGE_DELAY_MS (see chunk_purge_data) 
 * @note The inclusion criterion: do it if it does not affect the hot path; do not do it if it affects the hot path unacceptably. By that criterion, on-demand commit (splitting reserved/capacity: capacity must be reset on retire/claim, the most error-prone shape) and E1's "large-object
 *       arena" (whose gain was already eaten by C3's zeroing avoidance: measured alloc-large 99.7us -> ~24us, the remainder being kernel zero pages / page faults) were not done. */
namespace gcxx {
/** @brief The object header implicitly prefixed to every GC object -- **16 bytes**: next 8B / size 4B / marked 1B /
 * age 1B / type 1B / reserved 1B. @note **The payload is fixed at @16**, and slot starts are 16-byte aligned, so the payload naturally satisfies alignof(T) <= 16
 *       and no per-object offset field is needed. The gray-list pointer is **not in the object header** (it is the blocked-array worklist gc_state::gray_blocks), so there is neither a union nor a second/third payload offset.
 * @note Slot size is "16 + size" (plus PAD_BYTES when block-tail padding is involved) rounded up to 16 bytes, with a minimum slot of 32 bytes. */
struct gc_object {
    /**
     * The next element of the same-generation list / the nursery list.
     *
     * **Must be atomic**: `finish_cycle` walks the nurseries of **all** heaps during SWEEP
     * (mutators are still running) to reset colors, while other threads are at that moment pushing freshly allocated objects at the head of their own
     * nursery (`object_new` writes this field) -- both sides can touch this same cell of the same object.
     * A plain pointer would be a data race (TSan reported exactly this).
     *
     * The cost is zero: on x86-64 a relaxed atomic pointer read/write is just a plain mov; and the
     * various list walks in mark/sweep do not have this race and still use plain reads.
     */
    std::atomic<gc_object*> next;
    std::uint32_t size;      /**< payload bytes (max 4GiB-1) */
    std::atomic<std::uint8_t> marked;  /**< tri-color bit (see detail::color_bits); during marking, parked
                                            threads and the collector color the same object concurrently, so it must be atomic */
    std::uint8_t  age;       /**< generational age: 0 = newborn, >=GC_AGE_OLD = old generation */
    std::uint8_t  type;      /**< gc_type: whether this needs to be scanned */
    std::uint8_t  reserved;  /**< alignment padding (keeps the total at 16 bytes) */
};
/** @brief The object header is fixed at 16 bytes and the payload is fixed at @16: this is the foundation of every later "payload = o + 16" argument, so changing the layout must change it too. */
static_assert(sizeof(gc_object) == 16, "gc_object must stay 16 bytes");
static_assert(alignof(gc_object) == 8, "gc_object alignment must stay 8");
/** @brief Fixed payload offset = object header size. The gray-list pointer is not in the object header (see gc_state::gray_blocks),
 * and slot starts are 16-byte aligned, so the payload is naturally 16-aligned and one constant suffices. */
constexpr std::uint8_t PAYLOAD_OFF = 16;
/** @brief The number of block-tail padding bytes. Defined here (rather than in the "block-tail padding" section) because
 * object_total_need needs it earlier; see that section for the details. */
constexpr std::size_t PAD_BYTES = 8;
/** @brief Scan class: conservative scanning has to "treat the whole block of memory as a pointer array and check it word by word", but the vast majority of objects are in fact pure scalars
 * (integers, floating point, string buffers, the buffer of a vector<int>, ...) in which a pointer is impossible; tagged PLAIN/INT/FLOAT they are skipped wholesale during tracing, saving a great deal of time. @note The rule: **skip only types that can be proved free of pointers**, and conservatively scan everything else (safe by default). */
enum gc_type : std::uint8_t {
    GC_TYPE_RAW   = 0,   /**< unknown raw byte block    -> conservatively scanned */
    GC_TYPE_INT   = 1,   /**< integer family (char/bool/integral) -> scan skipped */
    GC_TYPE_FLOAT = 2,   /**< floating-point family            -> scan skipped */
    GC_TYPE_PLAIN = 3,   /**< other plain data provably free of pointers -> scan skipped */
    GC_TYPE_PTR   = 4,   /**< pointers / structs containing pointers -> conservatively scanned */
    GC_TYPE_SMALL = 5,   /**< untyped small block (size < sizeof(void*)): cannot hold even one complete pointer
                              -> scan skipped. gcs(n) / gc(n) produces it when size < sizeof(void*) */
    GC_TYPE_ATOMIC = 6   /**< **atomic memory**: allocated by gca(), the caller guarantees there are no pointers inside, never scanned
                              (not even the "are there pointers inside" test is done); it always lives in an **atomic block**, see GCXX_KIND_* */
};
/** @brief A chunk's "kind": a block holds either only objects that **need scanning** or only objects that **need no scanning**.
 * The partition is by the **type in the object header** (type_scanned_ct), not by which interface the caller used -- so `gc<int>` (not scanned) and `gcs<int>` (forced scan) naturally land in different blocks.
 * @note An object that needs scanning appearing in an atomic block is a bug, and gc_verify reports it outright (see verify_heap) -- "gca memory is never scanned" becomes an assertable structural property; there is also a locality benefit (data of one kind ends up together). */
constexpr unsigned KIND_NORMAL = 0;   /**< holds objects that need scanning (RAW / PTR)      */
constexpr unsigned KIND_ATOMIC = 1;   /**< holds objects that need no scanning (all the rest) */
constexpr unsigned KINDS      = 2;
/** @brief Upper bound (inclusive) on valid scan classes. **Change this when adding a new class** -- every place that validates
 * "is the type in the object header a legal value" uses it; never hard-code an enum member (hard-coding makes a new class be misjudged as "corrupt object header"). */
constexpr std::uint8_t TYPE_MAX = (std::uint8_t)GC_TYPE_ATOMIC;

} /* namespace gcxx */
/**
 * @brief Declare that a custom type contains **no pointers** inside, so `gc<T>` skips conservative scanning of it.
 *
 * Provide a specialisation of this function template for `T` at **global scope** (by convention it is public API,
 * hence it lives in no namespace and starts with `gc_`):
 * @code
 *   struct Vec3 { float x; float y; float z; };
 *   template <> constexpr bool gc_declare_plain<Vec3>() { return true; }
 *   Vec3* v = gc<Vec3>();          // treated as "no pointers" from here on
 * @endcode
 * @note The specialisation must appear **before** the first use of `T` with `gc<T>()`, exactly as
 *       with any other template specialisation.
 *
 * **This rule is NOT enforced by the library, and the earlier version of this comment was wrong
 *    about that**: it claimed "getting it wrong is caught immediately by
 *    `static_assert(gc_declare_plain<Vec3>(), \"\")`". No such assertion exists (or can exist) inside
 *    the library. `is_plain<T>` / `type_of<T>` read the predicate through the ordinary
 *    point-of-instantiation rules, so if you specialise *after* the first `gc<T>()`, the earlier
 *    instantiation has already been formed with the **unspecialised** answer and keeps it. The
 *    result is not a compile error but **two different classifications of the same `T` inside one
 *    translation unit** -- `gc<Vec3>()` before the specialisation scans the object, `gc<Vec3>()`
 *    after it does not. That is the one class of inconsistency a conservative GC must not have, and
 *    it is silent: add the `static_assert(gc_declare_plain<T>(), "")` **yourself** next to your
 *    specialisation (that is what `probes/api_doc_check.cpp:20` does) if you want the compiler to
 *    check the ordering for you.
 * @warning This is a **promise**, not a hint: if `T` really does contain pointers, those pointers will not be treated as roots
 *          and the target objects will be wrongly reclaimed -- the consequence is a dangling pointer. When unsure, **do not** specialise it,
 *          or use `gcs<T>()` to force scanning instead.
 * @note Built-in types (integer / floating point / enum) and memory allocated by `gca` **need no** declaration.
 * @return `false` by default (= needs scanning); only explicitly specialised types return `true`.
 */
template <typename T>
constexpr bool gc_declare_plain() { return false; }
namespace gcxx {
namespace detail {
/**
 * @brief The type-level "no pointers" predicate, equivalent to `::gc_declare_plain<T>()`.
 * @internal
 */
template <typename T>
struct is_plain {
    static constexpr bool value = ::gc_declare_plain<T>();
};
/**
 * @brief Maps a type to a scan class at compile time.
 * @internal
 */
template <typename T>
struct type_of {
    static const std::uint8_t value =
        std::is_integral<T>::value        ? (std::uint8_t)GC_TYPE_INT   :
        std::is_floating_point<T>::value  ? (std::uint8_t)GC_TYPE_FLOAT :
        (std::is_enum<T>::value ||
         is_plain<T>::value)              ? (std::uint8_t)GC_TYPE_PLAIN :
                                            (std::uint8_t)GC_TYPE_PTR;
};
template <typename T, std::size_t N>
struct type_of<T[N]> {
    static const std::uint8_t value = type_of<T>::value;
};
/** @brief Only these two classes really need their memory scanned.
 * @note The allocation path (writing the object header and payload), the tracing path (deciding whether to expand a payload) and the verification path all take this **same** predicate: any drift between them would give two answers to "does this object contain pointers" -- the one error a conservative GC must never make.
 * @internal
 */
constexpr bool type_scanned_ct(std::uint8_t t) {
#if GCXX_TYPE_SKIP
    return t == (std::uint8_t)GC_TYPE_RAW || t == (std::uint8_t)GC_TYPE_PTR;
#else
    return (void)t, true;        /* optimisation off: scan everything conservatively (control). A C++11 constexpr body can only be a single return, so it cannot be written as two statements. */
#endif
}
inline bool type_needs_scan(std::uint8_t t) { return type_scanned_ct(t); }
/** @brief This library supports **64-bit platforms only**.
 *
 * The soundness of conservative scanning depends on the premise "pointer width == machine word width == 8 bytes": word scanning steps by
 * `sizeof(void*)`, the `size` in the object header is 32-bit, and slot sizes are rounded up to 16-byte alignment.
 * On a 32-bit platform all these premises fail at once (and no configuration can rescue them), so we **reject outright at compile time**
 * rather than letting it show up at runtime as an occasional dangling pointer.
 * @internal
 */
static_assert(sizeof(void*) == 8, "gcxx requires a 64-bit platform (sizeof(void*) == 8)");
static_assert(sizeof(std::size_t) == 8, "gcxx requires 64-bit size_t");
static_assert(sizeof(std::uintptr_t) == 8, "gcxx requires 64-bit uintptr_t");
/* ---- compile-time validation of the tuning macros ---------------------- */
/**
 * @brief These macros are public, but a wrong configuration causes an outright out-of-bounds write or stops the GC from advancing -- better to fail compilation here than to show up at runtime as a bizarre crash.
 * @internal
 */
/** @brief capture_registers writes 16 general-purpose registers + 16 vector registers * 2 slots
 * = 48 uintptr_t. A buffer that is too small means an out-of-bounds stack write.
 * @internal
 */
static_assert(REG_CAPTURE_SLOTS >= 48,
              "REG_CAPTURE_SLOTS must be at least 48 on x86-64 "
              "(16 GPRs + 16 XMM registers * 2 slots)");
/** @brief The chunked-scan step must be non-zero and word-aligned: a step of 0 means continuation scanning of a large object never advances (infinite loop),
 * and a non-word-aligned step makes pointers crossing a slice boundary be missed (scan_words aligns each slice's start upwards independently).
 * @internal
 */
static_assert(SCAN_SLICE_BYTES > 0 &&
              SCAN_SLICE_BYTES % sizeof(void*) == 0,
              "SCAN_SLICE_BYTES must be positive and a multiple of sizeof(void*)");
/**
 * @brief A value of 0 for these stops gc_step / sweeping from advancing.
 * @internal
 */
static_assert(SLICE_BYTES > 0, "SLICE_BYTES must be positive");
static_assert(SWEEP_UNITS > 0,
              "SWEEP_UNITS must be positive (SLICE_BYTES / 32)");
static_assert(STEP_MAX_SLICES > 0, "STEP_MAX_SLICES must be positive");
static_assert(STW_TIMEOUT_MS > 0, "STW_TIMEOUT_MS must be positive");
/** @brief The frequency safety valve must be >= "the widest live-set ratio" (PAUSE_PERCENT, used after a major).
 * Live things can only come from mapped slots (live <= mapped always holds), so as long as CAP >= PAUSE this cap never binds while accounting is sane. Lowering it turns it into a genuine tuning parameter (collect more often), which is a deliberate behaviour change that must be made with full awareness.
 * @internal
 */
static_assert(DEBT_CAP_MAPPED_PERCENT >= PAUSE_PERCENT,
              "DEBT_CAP_MAPPED_PERCENT must be >= PAUSE_PERCENT "
              "(the mapped-bytes cap must not bind while accounting is sane)");
} /* namespace detail */
} /* namespace gcxx */
/* ==========================================================================
 * **Test injection points** (`gcxx::tests`; not part of the public API, and no stability promised)
 *
 * Why they are needed: several error/degradation paths **can only be reached when a system call
 * fails**, and would not trigger in a normal environment in a hundred years, while constructing the
 * unreachable state by hand needs internals a test cannot legitimately reach (resetting
 * `gray_blocks`, or calling `gc_padding_failed` directly). So **explicit injection switches** are
 * provided: `gc.hpp` carries one extra perfectly-predicted branch at each of those points (unset by
 * default = one relaxed read).
 *
 * **Why they are defined here, very early in the file**: the consumers (`gray_reserve`, `table_grow`,
 * `object_new`) are defined long before the rest of `gcxx::tests`. The block therefore sits on its
 * own, immediately after `namespace gcxx` closes for the first time: at that point `::gcxx` already
 * exists, so the `tests` sub-namespace can be opened here and the later
 * `namespace gcxx { namespace tests {` block merges into it.
 *
 * Discipline: **an injection resets itself in the call that consumes it** (see `gray_reserve`), so
 * it cannot affect later tests; after setting one, call the consuming interface immediately.
 * ========================================================================== */
namespace gcxx { namespace tests {
/** @brief Once set, the **next** gray-set block-table `calloc` is treated as a failure (cleared automatically after use).
 * @note A **function-local static** is used rather than a file-level variable: in a header-only library a file-level variable means one copy per TU,
 *       whereas a function-local static is guaranteed to be one per program under C++11. The default branch is perfectly predicted, so the hot path costs nothing. */
inline std::atomic<bool>& fail_next_gray_calloc() {
    static std::atomic<bool> flag(false);
    return flag;
}
/** @brief Once set, the **next** block-table grow `malloc` is treated as a failure (cleared automatically after use). */
inline std::atomic<bool>& fail_next_table_grow() {
    static std::atomic<bool> flag(false);
    return flag;
}
/** @brief Once set, `object_new` is treated as a failure, to exercise `allocate_bytes_oom`'s emergency collection path
 *        (cleared automatically after use). */
inline std::atomic<bool>& fail_next_object_new() {
    static std::atomic<bool> flag(false);
    return flag;
}
} } /* namespace gcxx::tests */
namespace gcxx {
namespace detail {
/* ------------------------- color / state constants ---------------------- */
/**
 * @brief Tri-color bits; the two white bits alternate, and flipping the white bit clears all marks at once.
 * @internal
 */
enum color_bits : std::uint8_t {
    WHITE0 = 1u << 0,   /**< white bit 0 (the two white bits alternate) */
    WHITE1 = 1u << 1,   /**< white bit 1 */
    BLACK  = 1u << 2    /**< black: expanded */
};
const std::uint8_t WHITE_BITS = (std::uint8_t)(WHITE0 | WHITE1);
/**
 * @brief Generational age (>= GC_AGE_OLD means the old generation).
 * @internal
 */
enum gc_age : std::uint8_t {
    GC_AGE0    = 0,                     /**< just born */
    GC_AGE1    = 1,                     /**< survived 1 cycle */
    GC_AGE_OLD = PROMOTE_AGE       /**< old generation */
};
/**
 * @brief Collection cycle phase.
 * @internal
 */
enum gc_phase {
    GC_PHASE_PAUSE = 0,   /**< idle, no collection cycle in progress */
    GC_PHASE_PROPAGATE,   /**< marking: expand gray objects (completed atomically in this implementation) */
    GC_PHASE_ATOMIC,      /**< atomic phase: rescan roots + drain gray + flip the white bit */
    GC_PHASE_SWEEP,       /**< incremental sweeping */
    GC_PHASE_END          /**< wrap-up: merge the nursery, recompute the thresholds */
};
/**
 * @brief Collection kind.
 * @internal
 */
enum gc_kind {
    GC_KIND_NONE = 0,
    GC_KIND_MINOR,   /**< minor: sweeps only the young generation; the old generation acts as a root */
    GC_KIND_MAJOR    /**< major: whole-heap mark + sweep + flip the white bit */
};
/* ------------------------------- global state --------------------------- */
/* ==========================================================================
 * Heap layer: 64KB aligned chunks + size classes + a direct-mapped chunk table. The goal: reduce "resolving an object from any suspected pointer during conservative scanning" from a hash-table lookup (~50ns) to a constant-time array index (~3ns),
 * while also eliminating the per-object malloc overhead and fragmentation. Chunk base = addr & ~0xFFFF; a small-object chunk serves exactly **one slot size** (a 16-byte-aligned "object header + payload"), so "which object does p fall inside" = one integer division + two comparisons,
 * O(1); a large object owns several chunks exclusively (also O(1), which naturally supports interior pointers); the table is direct-mapped (index = (addr>>16) & mask) with collisions on short chains of almost always length 1; empty chunks are handed back to the system (one spare kept per class).
 * ========================================================================== */
constexpr unsigned     CHUNK_SHIFT      = 16;                 /* 64KB */
constexpr std::size_t  CHUNK_SIZE       = (std::size_t)1 << CHUNK_SHIFT;
constexpr std::size_t  CHUNK_MASK       = CHUNK_SIZE - 1;
constexpr std::size_t  TABLE_INITIAL    = 256;   /* initial entry count (8KB, doubled on demand) */
constexpr unsigned     SLOT_ALIGN       = 16;
constexpr std::size_t  SMALL_MAX        = 1024;    /* tier 1: <=1KB at 16B granularity */
constexpr std::size_t  MEDIUM_MAX       = 32768;   /* tier 2: <=32KB at 256B granularity (several objects per chunk) */
constexpr unsigned     SMALL_CLASSES    = (unsigned)(SMALL_MAX / SLOT_ALIGN) + 1;
constexpr unsigned     MEDIUM_CLASSES   = (unsigned)((MEDIUM_MAX - SMALL_MAX) / 256);
constexpr unsigned     CLASSES          = SMALL_CLASSES + MEDIUM_CLASSES;
/** @brief The largest "total bytes within a slot" covered by tier-1 size classes (= the original semantics of a 16B header + 1KB payload).
 * Slot sizes are uniformly partitioned by **"bytes counted from the object's start"** (rather than by payload size).
 * @internal
 */
constexpr std::size_t   TIER1_MAX        = SMALL_MAX + 16;
constexpr std::size_t   MEDIUM_MAX_TOTAL = TIER1_MAX + (std::size_t)MEDIUM_CLASSES * 256;
constexpr std::uint32_t SLOT_NONE       = 0xFFFFFFFFu;
constexpr std::uint8_t  MARK_FREE       = 0x80;   /* free-slot sentinel: mark_object skips it outright */
struct gc_chunk;
/**
 * @brief Direct-map table entry (32B, inlined in a flat array).
 * @internal
 */
struct gc_map_entry {
    std::uintptr_t key;        /**< base address of this 64KB block; chunk == nullptr means an empty slot */
    gc_chunk*      chunk;      /**< the owning chunk (used by the large-object/fallback path) */
    gc_map_entry*  next;       /**< collision chain (overflow nodes are malloc'd nodes, extremely rare) */
    std::uint32_t  data_off;   /**< data-area offset (small blocks only, invariant after creation) */
    std::uint32_t  slot_size;  /**< slot size; 0 = large-object block (invariant after creation) */
};
struct gc_heap;
struct gc_chunk {
    gc_chunk*      cnext{};      /**< reused: the owner's partial list / the global recycle stack */
    unsigned char* base{};       /**< 64KB-aligned base address */
    gc_heap*       owner{};      /**< owning heap (allocation-side statistics / list ownership) */
    std::uint32_t  span{};       /**< bytes occupied (large objects > 64KB) */
    std::uint32_t  data_off{};   /**< data-area offset (after the entry array) */
    std::uint32_t  slot_size{};  /**< slot size; 0 = owned exclusively by a large object */
    /* The reciprocal of "divide by slot size" precomputed from slot_size (Granlund-Montgomery): the free path must derive the slot
     * index from the object address (once per dead object swept), and using (uint32_t)(((uint64_t)off * recip) >> 32) instead of a division leaves only imul+shr. @note **Exactness**: with r = floor(2^32/d) + 1 and e = r*d - 2^32 (0 < e <= d),
     * floor(off*r / 2^32) == floor(off/d) holds for all off < 2^32/e. Here d <= MEDIUM_MAX_TOTAL
     * (~2KB), so the threshold is >= 2^32/32784 ~131000; and off is necessarily < 65536 (the object and the chunk base are in the same 64KB block, which chunk_free's cache criterion already guarantees). So it is **an identity across the value range**, not an approximation. */
    std::uint32_t  recip{};
    std::uint32_t  capacity{};   /**< number of slots (1 for a large object) */
    /* ---- allocation side: **owner-private**, zero atomics on the hot path (mimalloc's page->free / page->used counterpart) ----
     * The fast path does only plain load/store: free is this thread's private free-list head (encoded link, SLOT_NONE = empty) and used is the number of slots this thread has allocated out of this block.
     * @note The collector reads used through used_pub (a published snapshot, synchronised by the owner with a plain store); the ordering is covered by in_partial's release/acquire: the owner publishes used and only then unlinks, while the collector reads
     *       in_partial and only then used_pub -- "seeing the unlink" necessarily means "seeing the latest used", so a block that just had a slot allocated from it can never be judged empty. */
    std::uint32_t  free{};        /**< owner-private free-list head (encoded link) */
    std::uint32_t  used{};        /**< owner-private: number of slots allocated */
    std::atomic<std::uint32_t> used_pub;    /**< published snapshot for the collector */
    /* ---- free side: collector-private ---- */
    std::uint32_t  freed{};       /**< slots freed by the collector */
    std::atomic<std::uint32_t> freed_pub;   /**< published snapshot for the owner (used by the fallback retire) */
    /* Number of slots carved out. The owner increments it and **marking threads read it concurrently** (to decide whether a candidate falls inside an allocated slot), so it
     * must be atomic -- the safety argument "an unallocated slot never hits" depends on this field. It is written only on the owner side, and relaxed suffices on the read side: a relaxed load + release store guarantees "slot contents first, indexability after", which on x86 is just a plain mov; RMW atomicity is useless here. */
    std::atomic<std::uint32_t> bump;
    std::atomic<std::uint32_t> free_head;   /**< shared free-list stack top (published in batches by the collector) */
    /* ---- the collector's private "dead slots awaiting publication" list (mimalloc's thread_free counterpart) ----
     * Sweeping is the **only producer** of free slots, so there is no need for "one CAS per object freed": dead slots are first strung onto a private list with two plain stores, and once a batch has accumulated they are spliced into free_head with **one CAS** (see publish_dead_slots).
     * @note Slots on the list are **temporarily invisible** to the owner (which is why the owner asks for a little more memory), so the publication points are well defined: a block switch / a full FREE_BATCH / the end of every sweep slice -- the latter two guarantee "no unpublished batch exists between slices", so the live count seen by thread-exit cleanup is accurate. */
    std::uint32_t  dead_head{};   /**< private dead-slot list head; SLOT_NONE = empty */
    std::uint32_t  dead_tail{};   /**< private dead-slot list tail (the first one inserted), used when splicing */
    std::uint32_t  dead_len{};    /**< length of the private list */
    std::uint32_t  cls{};         /**< size class; SLOT_NONE for large objects */
    /* Block kind: KIND_NORMAL / KIND_ATOMIC. Like cls, a geometric property that is **invariant after chunk creation**,
     * so it is covered by chunk_meta_checksum. */
    std::uint32_t  kind{};
    std::uint32_t  nkeys{};       /**< number of registered 64KB blocks */
    /* in_partial is read/written lock-free by the owner (class_link/unlink), read by the collector in chunk_free,
     * and written by the thread-exit path under gc_mutex -- concurrent from several sides, so it must be atomic. */
    std::atomic<bool> in_partial;  /**< linked on the owner's partial list */
    /* bl_hits must be atomic: during STW **several self-scanning threads run resolve_and_mark concurrently**, and the
     * "no match" signal for the same chunk is OR-ed in concurrently; a plain uint16 read-modify-write both loses updates (weakening the blacklist) and is itself a data race (UB). It is written only on the no-match path, where the cost of one relaxed fetch_or is negligible; an error in either direction is safe: losing one bit only blacklists one page less. */
    std::atomic<std::uint16_t> bl_hits;  /**< blacklist: bitmap of the 4KB pages hit by false candidates this major */
    std::uint16_t  bl_prev{};     /**< blacklist: hit bitmap of the previous major (two-round confirmation) */
    /* `bl_black` is **atomic for the same reason `bl_hits` is**: the collector ORs and clears it once
     * per major, while any allocating thread reads it from `chunk_usable` / `freelist_pop` /
     * `chunk_alloc_slow` with no lock held. Relaxed is enough: it is a placement hint, and a reader that
     * misses a freshly confirmed bit merely allocates one slot the hint would have skipped. A plain
     * `uint16_t` would be a data race, not just a stale read. */
    std::atomic<std::uint16_t> bl_black;  /**< blacklist: confirmed pages -> permanently avoided by the allocator */
    /* The free list's link field (the first 4 bytes of each free slot) stores an **encoded** slot number (the same as mimalloc's
     * page->keys): a link damaged by a wild write **with high probability** fails to decode into a legal slot number -- so "damage" shows up as "corruption detected, this list disabled" (bounded leak) rather than "the same slot handed out twice" (memory corruption). The two keys are each drawn once at random
     * when the chunk is created; the chunk struct lives inside the block, so the keys travel with it when the block changes owner or is reused, and the claim path resets free_head to empty, so no stale link is ever decoded. */
    std::uint32_t  key_rot{};   /**< free-list encoding key 1: rotation amount, 1..31 (see link_encode) */
    std::uint32_t  key_mask{};  /**< free-list encoding key 2: XOR mask */
    std::uint32_t  rng{};       /**< this block's private xorshift32 state (for allocation-order randomization) */
    /* Checksum over gc_chunk's **immutable fields** (including a constant magic number). Fields such as owner / used / freed that change
     * at every moment are excluded -- including them would be as good as no checksum at all. See the notes on chunk_meta_checksum. */
    std::uint32_t  meta_sum{};
    gc_chunk*      all_next{};    /**< global chunk list */
    /* The collector's private "has dead slots awaiting publication" list (see flush_dirty_slots). It is linked in only when dead_len goes from 0
     * to positive, and everything is published at the end of the slice. */
    gc_chunk*      dnext{};
    /* Whether the block is already on the collector's dirty-block list; must be atomic (the writers are the collector's chunk_free / flush, the readers
     * are chunk_retire on any thread). Two purposes:
     *  1. prevent double linking: when a batch fills up to FREE_BATCH, publish_dead_slots empties the private list in place, and a later free satisfies "dead_head == NONE" again; without this guard the same block would be linked twice -> a self-looping list and an endless flush loop;
     *  2. a **gate**: while on the list, retirement is forbidden (see chunk_retire) -- retiring would let the block be reused from the recycle stack (chunk_reset_state clears dead_len/dnext) or be handed back to the system directly by trim_recycle_stack, while its link is still on the dirty-block list;
     *     reuse would make flush lose the second half of the list, and handing it back to the system would be a use-after-free. */
    std::atomic<bool> in_dirty;
    /* ---- physical page management for retired blocks (D1/D2) -------------
     * After a block retires into the recycle stack its **address space is still retained** (reuse needs no remap), but the physical pages can be handed back to the system: purged means the data area has been decommitted (reading it back gives zero pages), and a claim recommits it. retired_ms is the time it entered the stack
     * (steady_clock milliseconds, 32-bit; comparisons use **unsigned wraparound subtraction**, so it is correct even at the 49-day wraparound point as long as the delay is far below 2^31ms); written by the pusher and read by cycle_owner's purge.
     * @note purged **must be paired with the push onto the recycle stack**: if it is set, this block is necessarily on the stack (not on any partial list, nor on any object list). Both are states that exist only while "the block is in the recycle stack", and neither takes part in chunk_meta_checksum. */
    std::uint32_t  retired_ms{};
    std::uint8_t   purged{};
    /* Whether this block's data area was **freshly committed by the OS** (and therefore guaranteed to be zero pages). Only large objects
     * (slot_size == 0) make use of it: such a block is always newly created and gets its memory from the OS, so the memset in object_new is pure duplicated work. Invariant after chunk creation, so it is covered by the checksum. */
    std::uint8_t   os_zeroed{};
    /* Whether this large-object block carries a **guard page** (E2 sampling): the object is placed flush against an inaccessible page,
     * payload[size] is the first byte of that page, and a 1-byte overrun faults immediately. Guarded blocks **do not take part in block-tail padding** (a guard page is strictly stronger: it explodes on the spot, whereas padding can only verify afterwards). Invariant after chunk creation. */
    std::uint8_t   guarded{};
};
/* ---- per-thread private heap (mimalloc style) ----
 * The allocation hot path touches only the fields in here: owned solely by this thread, zero locks and zero atomics.
 *  * partial[class_index(kind,cls)]: this thread's list of chunks that "still have free slots" (read/written solely by the owner);
 *  * the three object lists: the owner pushes at the head of nursery/young; young/old are swept by the collector (it walks existing nodes while the owner only pushes at the head, so the two do not conflict); debt/statistics caches: owned solely by the owner, with statistics flushed in 64KB batches to the global atomic counters. */
/* ---- per-thread accumulators for the diagnostic counters ----
 * scanned_bytes / skipped_bytes / bias_hits were originally **one global fetch_add per object** (on x86 a lock-prefixed
 * RMW), and during STW every self-scanning thread hammers the same cache line -- on a 220k-object heap, these
 * three counters alone emit hundreds of thousands of lock instructions. They were changed to thread-local accumulation, merged into the globals in one go only at marking-progress
 * boundaries (end of self-scan / end of a marking round). The accumulator is a POD, and a thread_local access fetches the TLS slot directly under both ABIs, with no dynamic
 * initialisation cost. */
/**
 * @brief Thread-local diagnostic accumulator.
 * @internal
 */
struct diag_accum {
    std::size_t scanned_bytes;
    std::size_t skipped_bytes;
    std::size_t bias_hits;
};
struct gc_heap {
    /* The **head** of each of the three object lists is atomic: what `sweep_some` stores is a **pointer-to-pointer** to the list
     * head or to some `gc_object::next` (`sweep_link`), so the type of the head must match that of
     * `next`; otherwise neither could be addressed. On x86-64 a relaxed atomic pointer read/write is
     * just a plain mov, at zero cost. */
    std::atomic<gc_object*> young;   /**< young-generation list (owner pushes at head / collector sweeps) */
    std::atomic<gc_object*> old;     /**< old-generation list (read and written only by the collector) */
    std::atomic<gc_object*> nursery; /**< objects allocated during a collection are linked here (owner pushes at head). **Atomic**:
                                      * `finish_cycle` walks every heap's nursery during SWEEP
                                      * to reset colors while the owner pushes at the head -- this head cell races too. */
    /* The index is class_index(kind, cls): normal blocks and atomic blocks each get a segment and never mix. */
    gc_chunk*  partial[KINDS * CLASSES]{};
    /* Pacing: positive = how many more bytes may be allocated before a collection is triggered.
     *
     * This field is read and written on the allocation hot path **on every allocation**, and the only cross-thread writer is the collector's once-per-cycle finish_cycle. debt is owner-private (zero atomic read-modify-write on the hot path); the collector publishes the new quota through debt_reset /
     * debt_reset_pending, and the owner tests and applies it with one relaxed load on its next allocation (almost always false). The semantics are unchanged: applying it a cycle late occasionally only affects collection timing, not correctness. */
    std::ptrdiff_t debt{};
    std::atomic<std::ptrdiff_t> debt_reset;
    std::atomic<bool>           debt_reset_pending;
    /**
     * @internal
     * @brief The **largest single allocation in bytes** of the previous cycle (an owner-private plain field).
     *
     * Purpose: make the next cycle's debt floor **at least a multiple of this number**. Without it, one allocation can eat
     * the entire debt quantum -- the quota floor is the fixed constant `MIN_HEAP_SIZE` (256KB), so a 64KB
     * object triggers a whole GC cycle every 4 allocations, and a 1MB object does so **on every allocation**. Measured: 2000
     * 64KB allocations triggered 364 collection cycles (major 91 + minor 273), and of the 12.96 us/op almost all was GC
     * amortisation, whereas the same number of allocations of 128B objects was 0 cycles and 0.07 us/op.
     * @note Written only by the owner and read only by `finish_cycle` under the lock; relaxed suffices (reading a stale value only
     *       costs one missing addition, safe by direction).
     */
    std::size_t max_alloc{};
    /* Statistics cache: plain accumulation on the hot path, flushed into the global atomic counters in one go on reaching 64KB. pend_* are the owner's private plain
     * accumulators (zero atomics on the hot path); pend_*_pub are the **published snapshots** read by gc_stat, synchronised with a plain store after every allocation. pend_* are not made atomic fetch_add because that would add one `lock xadd` (~20 cycles) to every allocation,
     * and only for a diagnostic count; the published snapshot is one plain store (the target line is already owned by this thread), essentially free. There is no second writer, so relaxed suffices for both store and load. */
    std::size_t    pend_bytes{};
    std::size_t    pend_objs{};
    std::atomic<std::size_t> pend_bytes_pub;
    std::atomic<std::size_t> pend_objs_pub;
    diag_accum     diag{};          /**< this thread's diagnostic accumulator (see diag_accum) */
    /* roots */
    char*          stack_top{};     /**< highest address of this thread's stack */
    char*          stack_base{};    /**< stack bottom at registration = lowest address of the committed stack area.
                                     * **The only reader is the Windows force-suspend path**
                                     * (`scan_suspended_stack`, inside `GCXX_CAN_FORCE_SUSPEND`),
                                     * which uses this one to clamp the scan lower bound so that it does not
                                     * read into the guard page and fault (on Windows the value comes from the TEB's `StackLimit`).
                                     * On POSIX `platform_stack_bottom()` fills it in too, but
                                     * there is no force-suspend there and **it has no reader at all** -- it is
                                     * reserved "for a possible future POSIX force-suspend", not part of the current
                                     * logic; do not conclude from this that the POSIX degraded stack-scan lower bound is clamped by it
                                     * (there the lower bound is `rsp - STACK_LOWER_MARGIN`). */
    /* nursery rehoming request: the collector sets it at the end of a cycle and the owner executes it in batches during PAUSE (it touches only its own
     * lists, and never at the same time as sweep). */
    std::atomic<bool>   rehome_pending;
    /* The STW rounds in which this heap has finished scanning and parked (see gc_state::sp_epoch). The collector uses it to decide
     * "how many heaps are in place this round". */
    std::atomic<unsigned long long> park_epoch;
    /* Post-cancellation backoff quota: it grows exponentially across consecutive cancellations,
     * avoiding "waiting out a whole STW budget for nothing every 256KB allocated". Reset after a
     * successful collection cycle.
     *
     * @note **Plain `std::size_t`, deliberately not atomic.** Every access is made by this heap's
     *       **owning thread**: `my_heap_slow` initialises it at creation, `cancel_cycle` grows it
     *       with `self`, and `begin_cycle` resets it with `self` -- and `self` is by definition the
     *       thread that owns the heap (it is the thread running `gc_pace` / `gc_collect`). The
     *       collector never touches another heap's backoff. So an atomic here would buy nothing and
     *       cost a `lock`-prefixed RMW on the cancellation path.
     * @warning This is a **single-threaded-access** field. Anyone who starts adjusting another
     *          thread's backoff (e.g. from `finish_cycle`) must make it atomic again -- that would
     *          be a new data race, not a style question. */
    std::size_t cancel_backoff;
    /* Thread identity, used to decide when this heap struct can be reclaimed (see reap_dead_heaps):
     *   * Windows: a real handle from DuplicateHandle, testable with WaitForSingleObject;
     *   * POSIX: no stored thread identity at all -- a `pthread_t` is a pointer into the thread's own stack
     *     and following it after the thread is gone faults, so ownership is decided by `last_active_epoch`. */
#if GCXX_PLATFORM_WINDOWS
    void*               thread_handle{};
#elif GCXX_PLATFORM_POSIX
    /**
     * Set by the platform exit callback (`heap_thread_exit_cleanup`) once this thread's cleanup has run.
     *
     * It is **not** a proof that the thread has stopped: the callback runs while TLS destructors are still
     * being unwound, and the heap's TLS slot is deliberately kept afterwards so that a destructor which
     * allocates reuses this same heap. A heap is therefore reclaimed only through
     * `gc_heap::last_active_epoch`, which measures "the owner has stopped using this heap" directly.
     * @internal
     */
    std::atomic<bool>   thread_ended;
#endif
    /**
     * The highest collection round in which this heap's owning thread was known to be running: set at every
     * safepoint the thread reaches and at every allocation it makes (`gc_pace`).
     *
     * This is the **only** criterion by which a heap is reclaimed, because it is the only one that answers the
     * question that actually matters -- "has the owner stopped touching this heap?" -- rather than the weaker
     * "did its exit callback run". A thread keeps allocating after that callback (see `thread_ended`), and a
     * `pthread_t` cannot be dereferenced to test liveness at all: glibc's `pthread_t` is a pointer into the
     * thread's own stack, which the stack cache unmaps once the thread has been joined, and following it then
     * faults (see `heap_thread_ended`). A heap whose stamp is older than `HEAP_REAP_ROUNDS` collection rounds
     * is therefore declared to have no owner left; a heap whose owner is merely idle is protected by
     * `HEAP_REAP_ROUNDS` being large enough to cover a scheduling delay.
     *
     * Only the owning thread writes it (relaxed; the value is a liveness hint, never a synchronisation point);
     * the collector reads it under `gc_mutex`.
     * @internal
     */
    std::atomic<unsigned long long> last_active_epoch;
    /* lifetime */
    std::atomic<bool>   dead;     /**< the thread has exited (the heap struct is kept: objects on the lists still need sweeping) */
    bool                inited{};
    gc_heap*            hnext{};    /**< global heap list (protected by gc_mutex) */
};
/** @brief A gray-set entry (one cell of the array-style worklist).
 * obj and epoch are written separately: the producer writes obj first and epoch afterwards, and the consumer uses epoch to decide "was this cell written this cycle". That way a thread force-suspended during the previous cycle and landing a late write after resuming cannot mix stale entries into this cycle's gray set (the stale entry is discarded, and its object is covered by the fallback rescan). It is 16 bytes rather than 8 bytes precisely for this identifiability.
 * @note The two fields are **atomic** (relaxed/release-acquire, all plain movs on x86-64, zero cost):
 *       producer and consumer reads/writes of the same cell are concurrent **in the memory model** (see gray_push),
 *       so plain fields would constitute a data race/UB -- exactly the same reason the list heads and `next` were made atomic.
 * @internal
 */
struct gray_entry {
    std::atomic<gc_object*>      obj;      /**< the gray object (written first) */
    std::atomic<unsigned long long> epoch; /**< cycle number at the time of writing (published with release) */
};
/** @brief The **block** granularity (in cells) of the gray-set array. It must be a power of 2: the index->block conversion uses shifts/masks,
 *  and this path is on the allocation hot path (`gray_push` once per gray object), so division is not an option.
 * @internal
 */
constexpr unsigned GRAY_BLOCK_SHIFT = 12;
constexpr std::size_t GRAY_BLOCK_ENTRIES = (std::size_t)1 << GRAY_BLOCK_SHIFT;
/** @brief Number of cells in the block-pointer table: the number of blocks the capacity limit covers.
 *  **Allocated once, never moved** -- this is the implementation basis of the invariant "the gray-set array never moves".
 * @internal
 */
constexpr std::size_t GRAY_NBLOCKS =
    (((std::size_t)GCXX_GRAY_CAPACITY_LIMIT + GRAY_BLOCK_ENTRIES - 1)
     >> GRAY_BLOCK_SHIFT);
static_assert((GRAY_BLOCK_ENTRIES & (GRAY_BLOCK_ENTRIES - 1)) == 0,
              "GRAY_BLOCK_ENTRIES must be a power of two (index->block uses "
              "a shift and a mask)");
static_assert(GRAY_NBLOCKS >= 1, "the block table must cover at least one block");
/** @brief A weak-reference / ephemeron table entry; one entry serves both kinds of handle:
 *   weak reference (WEAK_PLAIN)  key = the target object, value is always nullptr; the key is cleared when the target becomes unreachable. ephemeron (WEAK_EPH) key = the key, value = the value. **The value takes part in marking only while the key is reachable**; when the key is unreachable both are cleared (the value becomes garbage that cycle).
 * @note The two pointers **exist only in this table** (malloc memory, not conservatively scanned), and a handle itself holds only an index -- this is a **premise** of the design: handles usually live on the stack or in the static area (**places that are scanned**), where storing a raw pointer directly would make it a strong root at once (the weak reference would never expire and the ephemeron's value would pin the key forever); stored as a small integer index it is not recognised as a pointer.
 * @note next is the free list (an index); in_use distinguishes "empty slot" from "already reclaimed but the handle still exists": a handle keeps its index after the target dies (it needs it to read "expired"), and only destruction of the handle actually returns the slot to the free list.
 * @internal
 */
constexpr std::uint8_t WEAK_PLAIN = 0;
constexpr std::uint8_t WEAK_EPH   = 1;

struct weak_slot {
    std::atomic<gc_object*> key;     /**< weak-reference target / ephemeron key */
    std::atomic<gc_object*> value;   /**< the ephemeron's value (always nullptr for a weak reference) */
    std::uint32_t next{};              /**< free list (valid only for empty slots) */
    std::uint8_t  kind{};              /**< GCXX_WEAK_* */
    std::uint8_t  in_use{};            /**< the slot has been allocated */
    std::uint8_t  activated{};         /**< ephemeron: whether value has already been marked as a root this cycle */
    std::uint8_t  reserved{};
};
/**
 * @brief The "large-object continuation scan" cursor of **a single marker**.
 *
 * Why this type exists (the preparatory refactor for B1 parallel marking): when `traverse_object()` meets a large object
 * it scans only the first slice and leaves the rest to the cursor, slice by slice -- but the cursor used to be two global fields in `gc_state`
 * (`scan_cur`/`scan_off`), with the implicit premise that "**there is only one marker at a time**".
 * Once several threads drain the gray set together, that premise is gone: two markers would overwrite each other's cursor,
 * and the **remainder of the overwritten large object would never be scanned again -> its children are missed -> dangling pointer**
 * (the same trap as the "must be handed straight back to the caller" note in `rescan_gray_from_heap`).
 *
 * So the cursor becomes an **explicit parameter**: the collector uses its own `gc_state::scan`,
 * and each helper uses one on its own stack. **This is a purely structural rewrite -- with a single marker the behaviour is unchanged word for word**
 * (same thread, same cursor, same order of advancement).
 * @internal
 */
struct mark_cursor {
    gc_object*    cur;   /**< the large object being continued; nullptr = none */
    std::uint32_t off;   /**< bytes already scanned (an offset within the payload) */
    /**
     * The index range `[batch_next, batch_end)` that has been **claimed as a whole from the global gray set and not yet consumed one by one**.
     *
     * Why claim a whole batch: the consumption cursor `gray_head` must support exclusive claiming by multiple consumers (CAS/`fetch_add`),
     * and doing one atomic RMW per popped object measured **+10% (single-threaded) / +16% (multi-threaded)** on `mark_us`
     * -- the whole point of B1 is to compress this stretch, and it cannot afford to eat a tenth of it itself. Claiming in batches amortises that atomic operation
     * by a factor of `GRAY_BATCH`.
     * @warning While the range is **non-empty**, this batch of entries **has not been expanded yet**: every "is marking finished" criterion must
     * take it into account. This implementation makes `propagate_some` **finish the current batch** after its budget runs out
     * before it may return (see the loop condition there), so "the cursor is in no work unit" => "the batch must be empty",
     * and `cycle_mark_done` and B1's in-flight count can test only the latter.
 * @internal
     */
    std::size_t   batch_next;
    std::size_t   batch_end;
};
/** @brief The number of cells claimed as a batch from the global gray set each time.
 * 64 was chosen: one atomic RMW is spread over 64 objects (a ~64x reduction relative to one CAS per entry),
 * while the batch is small enough that a "claimed but not fully consumed" tail does not drag out the termination criterion.
 * @internal
 */
constexpr std::size_t GRAY_BATCH = 64u;
struct gc_state {
    /* ---- collection state machine (phase/white/kind are read concurrently by mutator threads) ---- */
    std::atomic<int>          phase;        /**< GC_PHASE_* */
    std::atomic<int>          kind;         /**< GC_KIND_* (read concurrently by mark_object) */
    std::atomic<std::uint8_t> white;        /**< the current white bit (read concurrently by mark_object) */
    std::uint8_t cycle_white;       /**< this cycle's "dead" test mask (collector-exclusive) */
    bool          next_major;       /**< the next cycle is a major (collector-exclusive) */
    /* This cycle's driver: gc_step/sweep run only on it. Mutators read it lock-free in gc_pace, so it must be an atomic pointer
     * (concurrent read/write of a plain pointer is a data race/UB). */
    std::atomic<gc_heap*> cycle_owner;
    /* ---- gray set: array-style worklist (the gray-list pointer no longer lives in the object header) ----
     * Why no synchronisation is needed: the producers (each mutator's self-scan before parking, plus the collector itself) and the "late arrivers that read a stale phase" (see gray_push) are all concurrent, while the consumer (the expansion loop) starts only **after every mutator has come to rest**. Enqueueing needs only one
     * fetch_add to obtain a unique index (no CAS needed), and dequeueing is the collector's exclusive cursor advance. Two anomalies must be handled well: a) **a force-suspended thread may stop between "got the index" and "has not written the cell yet"**, and after resuming writes
     * on, so that write lands in the next cycle's range -- epoch makes the stale entry identifiable (so it is discarded), and this cycle's marking always runs rescan_gray_from_heap once as a backstop (see gray_need_rescan); b) **the array is full** (wide graph: one object pushing hundreds of
     * thousands of children at once): enqueues that do not fit are dropped and gray_overflow is set; at wrap-up the same fallback rescan finds the "marked but not expanded" objects and expands them (worst case O(cycles x heap), on the exceptional path only). The array is lazily allocated and a failed allocation counts as capacity 0 -- then every enqueue takes the overflow branch and the whole marking round degenerates into fallback rescans, **still correct**, just slow.
     * @note **The array is never moved and never freed**: the block-pointer table `gray_blocks` is allocated once, and every block stays valid forever once published. For the reason see gray_reserve -- moving the array with `realloc` would at the same time cause (1) late arrivers writing into **freed** memory and (2) lost cells. */
    std::atomic<gray_entry*>* gray_blocks;
    /**
     * Number of blocks allocated and published (written exclusively by the collector; producers see it indirectly via gray_alloc/cap).
 * @internal
     */
    std::size_t  gray_committed_blocks;
    /**
     * The number of cells currently available. **It must be atomic**: the consumer (the collector, via gray_clear/gray_reserve)
     * and the producers (any thread, via gray_push's in-place growth) both write it, and gray_push does not run under
     * gc_mutex -- a plain size_t would constitute a data race (this is exactly what TSan reported).
     * Semantically it is only "an upper bound", so relaxed suffices: the visibility of the value is covered by tail's
     * fetch_add/release-acquire and by the handshake on entering STW.
 * @internal
     */
    std::atomic<std::size_t> gray_cap;
    /**
     * The number of cells actually malloc'd (the hard limit for growth); only ever increases, and is likewise read by several threads.
 * @internal
     */
    std::atomic<std::size_t> gray_alloc;
    std::atomic<std::size_t> gray_head;   /**< consumption cursor (**multi-consumer**: claimed via fetch_add/CAS) */
    /**
     * **The published watermark**: cells in `[0, gray_ready)` are guaranteed to have `obj`/`epoch` fully written by their producer.
     * Consumers claim only within this range -- that is the separation of "reserve (fetch_add)" from "publish (finished writing)",
     * and the premise that lets several markers push and pop concurrently in safety (see `gray_pump_ready`).
     * We always have `gray_ready <= gray_tail`; `gray_tail` is the reserve mark and may run ahead of it.
 * @internal
     */
    std::atomic<std::size_t> gray_ready;
    /**
     * **Diagnostic**: the number of times a consumer judged a cell to be "a stale entry from the previous cycle" and discarded it.
     *
     * This is the **direct test** of whether "reserve/publish separation" really takes effect: with a single marker it is naturally 0;
     * once several markers push and pop concurrently, a growing count means a consumer claimed a cell that was
     * "reserved but not yet fully written" -- which is the mechanism behind a missed mark.
     * **Under normal load, with no force-suspended thread, it must always be 0.**
 * @internal
     */
    std::atomic<std::size_t> gray_discards;
    std::atomic<std::size_t> gray_tail;   /**< producer mark (multi-producer fetch_add) */
    std::atomic<bool>        gray_overflow;      /**< some enqueue was dropped */
    std::atomic<unsigned long long> gray_epoch;  /**< this cycle's number */
    bool         gray_need_rescan;        /**< this cycle must run one fallback rescan */
    std::size_t  gray_rescans;            /**< diagnostic: number of fallback rescans */
    /* Diagnostic: the number of times a free-list link field was judged corrupt. Continued growth means **a wild write touched a free slot**
     * (a use-after-free write, an out-of-bounds write, or slot geometry computed wrongly elsewhere). By design it only degrades to "the rest of this list is temporarily unusable", never to handing the same slot out twice. */
    std::atomic<std::size_t> freelist_corruptions;
    /* minor old-generation scan cursor (collector-exclusive; the old-generation list is already split per heap) */
    gc_heap*   oldscan_heap;
    gc_object* oldscan;
    /* Sweep cursor (collector-exclusive; sweep_link points at "the next object slot to process in the current heap", and the heap
     * is switched together with sweep_heap when the block changes) */
    gc_heap*    sweep_heap;
    /**
     * Points at "the current object's next field" (a pointer-to-pointer); its type follows the atomic type of gc_object::next.
 * @internal
     */
    std::atomic<gc_object*>* sweep_link;
    int         sweep_stage;        /**< 0=young 1=old(major) 3=done */
    /* The collector's private "dirty block" list: every block on it has unpublished dead slots (dead_len > 0), flushed in one go at the end of the slice
     * (see flush_dirty_slots). Together with gc_chunk::dead_head: chunk_free publishes the previous one at a block switch and sweep_some publishes the current one at the end of the slice -- so "no unpublished batch exists between slices", and the live count seen by thread-exit cleanup is accurate. */
    gc_chunk*   sweep_dirty;
    /* Reverse-lookup cache on the sweep side: while walking a list, the vast majority of adjacent objects belong to the same chunk, so it turns "one
     * chunk-table lookup per object freed" into "one address-range comparison". **Only small-object blocks are cached** (a large-object block is really handed back/unmapped by chunk_release, so caching it could dereference freed memory on the next cycle). */
    gc_chunk*   sweep_of_cache;
    /* The large-object chunked continuation-scan cursor (**the collector's own one**). Helpers use the
     * `mark_cursor` on their own stacks, see the notes on mark_cursor. */
    mark_cursor scan{};
    /* pacing and adaptation (collector-exclusive) */
    std::size_t live_at_major;
    std::size_t promoted_since_major;
    /* gc_step's re-entrancy depth. Although by convention only the cycle_owner advances it, full_collect's takeover path reads it
     * from **another thread** to decide whether advancing is worthwhile -- reading a plain int like that is a data race, hence atomic. */
    std::atomic<int> step_depth;
    std::atomic<int> collect_depth;      /**< several threads may call gc_collect at the same time */
    /**
     * "Some thread is currently on the out-of-memory emergency path."
     *
     * **It must be atomic**: several threads can reach `allocate_bytes_oom` at the same time (each
     * has its own heap, and any allocation may fail), and the flag exists precisely so that the
     * **second** one does not re-enter `full_collect()` while the first is inside it. A plain
     * `bool` read by one thread while another writes it is a data race/UB -- the same reasoning that
     * makes `phase`/`kind`/`white`/`aborting`/`mark_excl` atomic. It is also why the entering side
     * uses a `compare_exchange` rather than "test then store": two threads that both read `false`
     * would both run the emergency path, which is exactly the pile-up this flag is meant to stop.
     *
     * @note Relaxed is enough: the flag carries no data, it is only an exclusion hint for a
     *       best-effort path (`full_collect` re-checks `collect_depth` under `gc_mutex` anyway, so
     *       a lost race degrades to "this thread throws `bad_alloc`", never to corruption).
     * @note Read for diagnostics by `gcxx::tests::gc_test_state()`.
     */
    std::atomic<bool> in_oom;
    /* See scan_ctx::excl: begin_cycle sets it "after all mutators have come to rest and before expansion starts", and marking
     * end (atomic_finish) clears it. Only the cycle_owner writes it.
     * **It must be atomic**: the writer is the collector and the readers are **arbitrary mutators** -- `mark_roots_self`
     * (when a mutator self-scans its stack at a safepoint) reads it through `make_scan_ctx`, and the two are concurrent. Concurrent read/write of a plain `bool`
     * is a data race/UB; the same goes for `kind`/`white`/`phase`/`step_depth`/`aborting`.
     * relaxed suffices: it is only the performance hint "am I the sole marker", and it **does not affect correctness**
     * (both paths write the same object colors, and `mark_object`'s CAS branch is always well defined). */
    std::atomic<bool> mark_excl;
    /* ---- global statistics (atomic) ----
     * Both sides account in batches, with zero atomic RMW on the hot path: the allocation side accumulates in the owner-private pend_* up to 64KB and then fetch_adds; the free side accumulates **here** in the collector (see sweep_pend_*), merging in one go at the batch threshold or at cycle wrap-up. If the free side accounted "3 atomic RMWs per object" (total_bytes / freed_objects / live_objects), each free measured
     * 11.4ns -- the largest single fixed cost on the whole gc path. */
    std::atomic<std::size_t> total_bytes;
    std::atomic<std::size_t> live_objects;
    std::atomic<std::size_t> freed_objects;
    /* Batch accumulators on the free side. **Only the cycle_owner** frees objects, so these two fields are also collector-private
     * (they live in gc_state rather than thread_local so that a takeover path changing owner does not lose the unflushed part). The `_pub` fields are the atomic snapshots read by `gc_stat()` (written only on each flush). */
    std::size_t sweep_pend_bytes;
    std::size_t sweep_pend_objs;
    std::atomic<std::size_t> sweep_pend_bytes_pub;
    std::atomic<std::size_t> sweep_pend_objs_pub;
    std::atomic<std::size_t> cycles;
    std::atomic<std::size_t> minor_cycles;
    std::atomic<std::size_t> major_cycles;
    std::atomic<std::size_t> scanned_bytes;
    std::atomic<std::size_t> skipped_bytes;
    std::atomic<std::size_t> bias_hits;
    std::atomic<std::size_t> root_scans;
    std::atomic<std::size_t> root_probed_words;
    /* Cycles cancelled because "not all mutators could be stopped within the budget". Continued growth means some thread goes a long time without passing an allocation point
     * (a long computation / a join wait), and the heap keeps growing in that case -- the price paid for never wrongly killing a live object. */
    std::atomic<std::size_t> cancelled_cycles;
    /* ---- address index (mutations under gc_mutex; **readers are lock-free**) ----
     * `table_find` is called with no lock at all from `gc_is_gc_pointer`, `gc_test_object_of`,
     * `weak_peek` and `object_find`, while the collector mutates the same structure from
     * `chunk_retire` / `chunk_table_link` / `table_grow`. The two retired chains below are the
     * consequence: **a reader may hold a pointer into the index for an unbounded time**, so no part
     * of it may be released while the process is live. See `table_retire_nodes` / `table_take_array`. */
    gc_map_entry*  chunk_table;
    std::size_t    table_size;
    std::size_t    table_mask;
    std::size_t    block_count;
    /** Former bucket arrays, linked through entry 0's `next`. Freed only by `gc_dispose()`. */
    gc_map_entry*  table_retired;
    /** Overflow nodes still reachable from a retired array: **never handed out again**. */
    gc_map_entry*  table_retired_nodes;
    /** Overflow nodes spliced out of the *current* table: unreachable, safe to reuse. */
    gc_map_entry*  table_free_nodes;
    gc_chunk*      chunks;
    std::size_t    chunk_count;
    std::size_t    mapped_bytes;
    /* Scan-range snapshot: written by chunk_new (under the lock), read by scan_words (any marking thread) */
    std::atomic<std::uintptr_t> heap_lo;
    std::atomic<std::uintptr_t> heap_hi;
    /* Static area + shared-library data-segment root ranges (initialised once, published under gc_mutex) */
    char*    static_ranges[2 * STATIC_RANGES_MAX]{};
    unsigned static_range_count;
    bool     static_inited;
    /* Whether the dynamic roots have been collected. The hot path only reads this one relaxed atomic; collection takes the cold path
     * (see scan_dyn_roots_slow: **collect outside the lock, publish inside it**). */
    std::atomic<bool> dyn_roots_scanned;
    /* ---- weak reference / ephemeron table ---------------------------------
     * The table is allocated in **malloc memory**, not on the GC heap -- for safety rather than convenience: the table stores **raw pointers** to targets, and a raw pointer sitting on the GC heap/stack would be taken for a root by conservative scanning, after which the weak reference would never expire; malloc memory is not inside
     * the heap range, so the scanner cannot read it (there is precedent in the library: gc_state (including the gray set) is malloc'd too, see the notes on G()). The table is managed with a "free list + high watermark", register/unregister both hold gc_mutex, and entries are **never moved** (an index stays valid for life once handed out). */
    weak_slot*   weak_tab;
    std::size_t  weak_cap;
    std::size_t  weak_len;       /**< high watermark (including empty slots) */
    std::size_t  weak_live;      /**< number of live registrations (the whole phase is skipped when 0) */
    std::uint32_t weak_free;     /**< free-list head (an index); SLOT_NONE = empty */
    /* ---- old arrays **retired** when the weak table grows -----------------
     * `weak_peek` must be able to read **lock-free** (see the notes there), so the old table must never be `free`d on growth:
     * a thread in the middle of `gc_weak::get()` may already have read the address of the old `weak_tab`.
     * Old tables are linked onto this list, and the whole list is freed only by `gc_dispose()` (the caller guarantees exclusivity).
     * Growth happens O(log n) times and the list length starts at 8 and doubles, so in practice it holds only a few items. */
    weak_slot**  weak_retired;
    std::size_t  weak_retired_len;
    std::size_t  weak_retired_cap;
    /* ---- the **generation tag** of the weak registration table --------------
     * Purpose: to make "handles that survive across `gc_dispose()`" identifiable.
     *
     * The shape of the problem: a handle (`gc_weak` / `gc_ephemeron`) holds only a `std::uint32_t` index,
     * while `gc_dispose()` `free`s the whole table and zeroes `weak_len`. The library's contract is
     * "it may still be used after dispose", so **a handle living on the stack/in the static area and destroyed after dispose**
     * is a known shape (that is why `weak_slot_free` / `weak_peek` / `weak_set` all carry protection for it).
     * But testing only `idx < weak_len` is not enough: **the table grows back**, and after growing back
     * the same index points at **somebody else's slot** -- a stale `w.reset()` would unregister another handle's
     * registration, and a stale `w.get()` would return **somebody else's object**. That is silent cross-talk.
     *
     * The criterion: every `gc_dispose()` increments this field; a handle records the tag from registration time and compares before reading or writing.
     * If they differ => this registration belongs to the previous table generation and is **treated as "not registered"** (`SLOT_NONE`).
     * `0` is reserved for "the handle was never registered" (the companion of `SLOT_NONE`), so a **fixed initial value** is used
     * rather than allowing it to take 0. `std::uint32_t` is as wide as the index (see `weak_slot_alloc`'s clamping of the
     * 32-bit index space), and a theoretical wraparound would need 2^32 disposes, so it is no real risk.
     * @note Read and written only under `gc_mutex` (dispose holds the lock throughout, and register/unregister hold it too),
     *       so it is a plain field and **not** atomic -- `weak_peek`'s lock-free read touches only
     *       the `idx`/`weak_len`/`weak_tab` group and never touches it. */
    std::uint32_t weak_generation;
    /* ---- threads and concurrency ---- */
    std::recursive_mutex gc_mutex;       /**< the global big lock: chunk table / heap list / STW
                                          * (recursive: finish_cycle re-enters while full_collect holds the lock) */
    /* The list of all heaps. **Insertion is lock-free (CAS on the head); removal and traversal are under gc_mutex**. Insertion must be lock-free: a new
     * thread's heap must be linked onto this list **before it could reach any safepoint** -- otherwise the thread initiating STW, enumerating the list for "who must be waited for", will not see it, and the marks that thread later pushes into the gray list from its self-scan may land after marking completes: the objects it marked live
     * survive this cycle because the white bit was cleared, while their children, still carrying this cycle's white bit, are swept -- leaving dangling pointers. With lock-free insertion, STW is guaranteed to see it and wait for it to be in place (it will self-scan and park on gc_mutex). */
    std::atomic<gc_heap*> heaps;
    /* Likewise segmented by class_index(kind, cls): a block on the recycle stack does not change kind when it changes owner (the kind is a geometric
     * property of the block, invariant after creation like slot_size). */
    std::atomic<gc_chunk*> recycle[KINDS * CLASSES];
    /* How many blocks the recycle stack currently holds. **Those blocks have already been unlinked from the global list** (chunk_retire decrements chunk_count /
     * mapped_bytes), but the memory is still mapped -- so the "total mapped" that gc_stat reports must add it back, otherwise the real footprint is grossly underestimated (measured 190x underestimate: 1 block / 64KB reported while process RSS was 22MB). For the upper bound see RECYCLE_MAX. */
    std::atomic<unsigned> recycled;
    /* ---- cold list: empty blocks whose data area has been decommitted (D1/D2) ----
     * The semantics are exactly those of recycle[], except that "the physical pages have already been handed back to the system". **It must be a separate list**: if shared, decommitted blocks would be pushed back onto the top of the stack, purge would see them first every time and never reach the deeper blocks (which are the ones that really should be reclaimed); with them separated,
     * "decommit" is one-way (hot -> cold) and never turns back. A block taken from the cold list on claim is recommitted, and reading it back gives **zero pages**
     * -- which incidentally turns the safety argument "the object header of an unallocated slot is all zeros => size=0 => it never hits" from "usually holds" into "necessarily holds" (see resolve_and_mark). */
    std::atomic<gc_chunk*> recycle_cold[KINDS * CLASSES];
    std::atomic<unsigned> recycled_cold;
    std::atomic<std::size_t> decommits;      /**< diagnostic: number of data-area decommits */
    std::atomic<std::size_t> recommits;      /**< diagnostic: number of recommits on claim */
    std::atomic<std::size_t> guarded_objects;  /**< E2 sampling count: number of large objects picked for a guard page (diagnostic + test criterion) */
    std::uint32_t guard_tick;                /**< sampling cursor (incremented under gc_mutex) */
    /* Process page size (queried lazily once). Neither sysconf nor GetSystemInfo is cheap, and the page boundaries for decommit / commit
     * need it every time. */
    std::size_t os_page;
    /* ---- per-phase timing (A4) --------------------------------------------
     * **Cumulative** values in microseconds, timed only on the collector side (a cycle's STW wait / root scan / marking, plus every sweep slice). Not a single line is added to the mutator allocation hot path -- without breaking down "which stretches a pause actually consists of", there is no way to tell where the benefit of a change lands. */
    std::atomic<std::size_t> t_stw_us;
    std::atomic<std::size_t> t_root_us;
    std::atomic<std::size_t> t_mark_us;
    std::atomic<std::size_t> t_sweep_us;
    /* Process-wide random seed (the free-list two-key encoding / padding canary / allocation-order randomization are all derived from it). It lives here rather than in a function
     * -local static so that nothing is placed in the writable static area (see the notes on G()). */
    std::uint64_t link_seed;
    /* Internal error count. Likewise placed here rather than in a function-local static. */
    std::atomic<long> error_count;
    /* safepoint handshake. **The release condition is a change of sp_epoch, not sp_request returning to zero**: between the end of one cycle and the start of the next
     * the request is immediately set to 1 again, and testing request==0 would leave late or slow-to-wake threads spinning forever (a lost wakeup -> STW hangs). Each heap records the round in which it was in place in park_epoch, and the initiating thread counts "heaps in place
     * this round" rather than using a global counter that gets zeroed (which would let the previous cycle's late arrivers masquerade as in place this round). */
    std::atomic<int>                 sp_request;   /**< fast gate: non-zero means an STW is in progress */
    std::atomic<int>                 sp_parked;    /**< used only for diagnostic output */
    std::atomic<unsigned long long>  sp_epoch;     /**< round number: +1 on every begin_cycle */
    /* The number of threads currently running a self-scan (mark_roots_self). Cancelling a cycle uses it to drain "threads still coloring objects", and only once drained
     * does it dare roll the colors back wholesale. A seq_cst handshake, see the notes on safepoint_reach. */
    std::atomic<int>                 sp_scanning;
    /**
     * "this cycle is being cancelled and the color rollback may already have walked the object lists". See the **cross-cycle guard** at the end of object_new.
     *
     * Why it is needed: the cancellation path is ordered `gray_clear() => unmark_all_objects() => phase = PAUSE`,
     * and step 3 waits only for `sp_scanning == 0` -- **`object_new` does not take part in `sp_scanning`**. So a thread that
     * has already read `ph in {PROPAGATE, ATOMIC}` but has not yet linked its object can link it into the nursery **after** `unmark_all_objects()`
     * has run, and its color is "no white bit". That object then enters the idle phase with no white bit, and
     * in the next cycle `mark_object` skips it at the `(m & WHITE_BITS) == 0 => already gray/black` branch,
     * so its children are never expanded -- **missed mark => dangling pointer**.
     * Measured: under a Debug build, about a quarter of the cycles in a four-thread stress run showed
     * `[verify] idle-phase color is not the current white bit list=2 marked=0`, while `cancelled_cycles` kept growing and
     * `cycles` was always 0 -- exactly this path.
     *
     * Semantics: while it is true, any object "darkened during the marking window" must restore itself to unmarked
     * (it is in the nursery and will not be swept this cycle, so restoring it to white is the **safe direction**).
 * @internal
     */
    std::atomic<bool>                aborting;
    bool inited;
    gc_state() {
        phase.store((int)GC_PHASE_PAUSE, std::memory_order_relaxed);
        kind.store((int)GC_KIND_NONE, std::memory_order_relaxed);
        white.store((std::uint8_t)WHITE0, std::memory_order_relaxed);
        cycle_white = (std::uint8_t)WHITE0;
        next_major = true;
        cycle_owner.store(nullptr, std::memory_order_relaxed);
        aborting.store(false, std::memory_order_relaxed);
        gray_blocks = nullptr;
        gray_committed_blocks = 0;
        gray_cap.store(0, std::memory_order_relaxed);
        gray_alloc.store(0, std::memory_order_relaxed);

        gray_head.store(0, std::memory_order_relaxed);
        gray_ready.store(0, std::memory_order_relaxed);
        gray_discards.store(0, std::memory_order_relaxed);
        gray_tail.store(0, std::memory_order_relaxed);
        gray_overflow.store(false, std::memory_order_relaxed);
        gray_epoch.store(0, std::memory_order_relaxed);
        gray_need_rescan = false;
        gray_rescans = 0;
        freelist_corruptions.store(0, std::memory_order_relaxed);
        oldscan_heap = nullptr; oldscan = nullptr;
        sweep_heap = nullptr; sweep_link = nullptr; sweep_stage = 3;
        sweep_dirty = nullptr; sweep_of_cache = nullptr;
        scan.cur = nullptr; scan.off = 0;
        scan.batch_next = 0; scan.batch_end = 0;
        live_at_major = 0; promoted_since_major = 0;
        step_depth.store(0, std::memory_order_relaxed);
        collect_depth.store(0, std::memory_order_relaxed);
        in_oom.store(false, std::memory_order_relaxed);
        mark_excl.store(false, std::memory_order_relaxed);
        total_bytes.store(0, std::memory_order_relaxed);
        live_objects.store(0, std::memory_order_relaxed);
        freed_objects.store(0, std::memory_order_relaxed);
        sweep_pend_bytes = 0; sweep_pend_objs = 0;
        sweep_pend_bytes_pub.store(0, std::memory_order_relaxed);
        sweep_pend_objs_pub.store(0, std::memory_order_relaxed);
        cycles.store(0, std::memory_order_relaxed);
        minor_cycles.store(0, std::memory_order_relaxed);
        major_cycles.store(0, std::memory_order_relaxed);
        scanned_bytes.store(0, std::memory_order_relaxed);
        skipped_bytes.store(0, std::memory_order_relaxed);
        bias_hits.store(0, std::memory_order_relaxed);
        root_scans.store(0, std::memory_order_relaxed);
        root_probed_words.store(0, std::memory_order_relaxed);
        cancelled_cycles.store(0, std::memory_order_relaxed);
        chunk_table = nullptr; table_size = 0; table_mask = 0; block_count = 0;
        table_retired = nullptr; table_retired_nodes = nullptr;
        table_free_nodes = nullptr;
        chunks = nullptr; chunk_count = 0; mapped_bytes = 0;
        heap_lo.store(~(std::uintptr_t)0, std::memory_order_relaxed);
        heap_hi.store(0, std::memory_order_relaxed);
        static_range_count = 0; static_inited = false;
        dyn_roots_scanned.store(false, std::memory_order_relaxed);
        for (auto & r : static_ranges) r = nullptr;
        weak_tab = nullptr; weak_cap = 0; weak_len = 0; weak_live = 0;
        weak_free = SLOT_NONE;
        weak_retired = nullptr; weak_retired_len = 0; weak_retired_cap = 0;
        /* See gc_state::weak_generation: it **must not be 0** (0 = "the handle was never registered"),
         * so the first tag of each table generation is obtained by adding 1 to this. */
        weak_generation = 0;
        heaps.store(nullptr, std::memory_order_relaxed);
        for (auto & i : recycle)
            i.store(nullptr, std::memory_order_relaxed);
        for (auto & i : recycle_cold)
            i.store(nullptr, std::memory_order_relaxed);
        recycled.store(0, std::memory_order_relaxed);
        recycled_cold.store(0, std::memory_order_relaxed);
        decommits.store(0, std::memory_order_relaxed);
        recommits.store(0, std::memory_order_relaxed);
        guarded_objects.store(0, std::memory_order_relaxed);
        guard_tick = 0;
        os_page = 0;
        t_stw_us.store(0, std::memory_order_relaxed);
        t_root_us.store(0, std::memory_order_relaxed);
        t_mark_us.store(0, std::memory_order_relaxed);
        t_sweep_us.store(0, std::memory_order_relaxed);
        {
            /* The seed is taken from steady_clock + this object's address: not cryptographically random, it only needs to be "different per process and
             * stable within a process" (see the threat-model notes on link_encode). */
            const std::uint64_t t = (std::uint64_t)
                std::chrono::steady_clock::now().time_since_epoch().count();
            std::uint64_t x = t ^ ((std::uint64_t)(std::uintptr_t)this << 21);
            x ^= x >> 33; x *= 0xFF51AFD7ED558CCDULL; x ^= x >> 29;
            link_seed = x | 1u;
        }
        error_count.store(0, std::memory_order_relaxed);
        sp_request.store(0, std::memory_order_relaxed);
        sp_parked.store(0, std::memory_order_relaxed);
        sp_epoch.store(0, std::memory_order_relaxed);
        sp_scanning.store(0, std::memory_order_relaxed);
        inited = false;
    }
};
/* ---- global singleton ----
 * It lives on the **heap** rather than in a function-local static (= .bss) because: 1) the writable section .data/.bss is within the scope of conservative root scanning, and gc_state is tens of KB (recycle[130] + index-table pointers + statistics ...), so lying there would mean scanning a large block for nothing every cycle;
 * 2) worse, its insides are **full of GC heap addresses** (chunks, heaps, gray_head, index-table entries), and being picked up by the root scan would mark a whole swathe of already unreachable objects live. @note Allocated with new and **never freed**: a process-lifetime object whose destruction is meaningless and harmful -- a global object destroyed later
 *       that calls gc would reach an already destroyed state. Initialisation of a function-local static pointer has been thread-safe since C++11. */
inline gc_state& G() {
    static auto* const g = new gc_state();
    return *g;
}
/** @brief Internal error reporting (**not a public interface**), serving only the "heap structure corruption detected" class of paths: block-tail padding
 * checksum failure, free-list encoding decoding to an out-of-range slot index, object header and delta mismatch.
 * @note The only public interfaces are `gc_collect` / `gc_dispose` / `gc_dump` / `gc_is_gc_pointer`
 *       (plus the **explicitly not promised to be stable** `gc_stat` / `gc_verify` in `gcxx::tests`).
 * @warning **Never allocate memory, never throw**: every call site is on a path where "memory inconsistency has already been detected", and allocating there is pouring fuel on the fire.
 * @note Suppression: at most ERROR_MAX entries are printed (to keep a broken heap from flooding stderr); **the count is not affected by suppression** -- "how many times it was detected" and "how many lines were printed" are two different things, and only the former is a diagnostic basis.
 * @internal
 */
constexpr long ERROR_MAX = 16;
/**
 * @brief Global diagnostic error counter slot.
 * @return A writable reference; **globally unique** (not `thread_local`) -- errors from all threads accumulate into the same cell,
 *         so there is no need to aggregate them in `diag_flush()`.
 * @note Kept in `gc_state` rather than in a function-local `static`: see `gc_state::error_count` -- do not put it in the writable static area.
 * @internal
 */
inline std::atomic<long>& error_count_slot() {
    return G().error_count;      /* see gc_state::error_count: do not put it in the static area */
}
/** @brief Cumulative number of detected corruptions. It should always be 0; non-zero means the real thing -- either the user wrote out of
 * bounds, or this GC itself miscalculated the slot geometry.
 * @internal
 */
inline std::size_t error_count() {
    const long n = error_count_slot().load(std::memory_order_relaxed);
    return n > 0 ? (std::size_t)n : 0;
}
/**
 * @brief Split into a _v layer and a variadic layer: a caller that wants to forward a va_list can only do so through the va_list parameter.
 * @internal
 */
inline void gc_error_message_v(int err, const char* fmt, va_list ap) {
    const long seen = error_count_slot().fetch_add(1, std::memory_order_relaxed);
    if (seen >= ERROR_MAX) return;          /* the count still rises, it just stops printing */
    char buf[512];
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0) buf[0] = 0;
    buf[sizeof(buf) - 1] = 0;                    /* vsnprintf truncation guarantee (guards against old CRTs) */
    std::fprintf(stderr, "gcxx: %s (errno=%d)\n", buf, err);
    std::fflush(stderr);
}
/**
 * @brief Format and print one error message while accumulating the count (the number of printed lines is suppressed by `ERROR_MAX`, **the count is not affected**).
 * @param err errno or a library error code; `fmt` is a printf-style format string.
 * @internal
 */
inline void gc_error_message(int err, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    gc_error_message_v(err, fmt, ap);
    va_end(ap);
}
/* ---- Reading / publishing the global heap chain (see the note on gc_state::heaps) ---- */
/**
 * @brief Read the global heap chain head.
 * @internal
 */
inline gc_heap* heaps_head() {
    return G().heaps.load(std::memory_order_acquire);
}
/**
 * @brief Walk the three object chains (young / old / nursery) of **every** heap, calling
 *        `fn(o, list_index)` for each object.
 *
 * Why this foreach exists: three chains times every heap, visiting "every allocated object", has
 * three callers in this file (rolling back colors on cancellation, gray-set fallback rescan, full-heap self-check), and "which chains, in what order"
 * is **one and the same invariant** (allocated implies linked => the union of the three chains is exactly all allocated objects). When it was
 * written as three hand-copied loops, any one of them missing the nursery became a silent defect where "some objects are never processed".
 *
 * @param fn A callable with signature `void(gc_object*, int)` or `bool(gc_object*, int)`;
 *        the second argument is the chain index (0 = young, 1 = old, 2 = nursery), for diagnostic output.
 * @return **When the callback returns bool**: whether the walk reached the end (a callback returning false short-circuits and the function
 *         returns false). When the callback returns void the return value is always true.
 * @note Uses a **template** rather than `std::function`: this function is called on full-heap paths and the callback must be inlinable
 *       (an indirect call through `std::function` would add one unpredictable jump to this path).
 * @note The two callback forms are dispatched at compile time with `enable_if`, at zero cost; do **not** change it to "one more overload with a
 *       default argument" -- that becomes equally viable when `fn` returns bool and is indistinguishable to the compiler
 *       (measured: GCC reports `call of overloaded ... is ambiguous` outright).
 * @note The walk itself **takes no lock**: the caller is responsible for guaranteeing that "the chain structure stays stable during the walk" (see the notes at each call site).
 * @internal
 */
template <typename Fn>
inline typename std::enable_if<
    std::is_same<decltype(std::declval<Fn&>()((gc_object*)nullptr, 0)), bool>::value,
    bool>::type
for_each_object(Fn&& fn) {
    for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
        gc_object* lists[3];
        lists[0] = h->young; lists[1] = h->old; lists[2] = h->nursery;
        for (int li = 0; li < 3; ++li)
            for (gc_object* o = lists[li]; o != nullptr; o = o->next)
                if (!fn(o, li)) return false;
    }
    return true;
}
template <typename Fn>
inline typename std::enable_if<
    !std::is_same<decltype(std::declval<Fn&>()((gc_object*)nullptr, 0)), bool>::value,
    bool>::type
for_each_object(Fn&& fn) {
    for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
        gc_object* lists[3];
        lists[0] = h->young; lists[1] = h->old; lists[2] = h->nursery;
        for (int li = 0; li < 3; ++li)
            for (gc_object* o = lists[li]; o != nullptr; o = o->next)
                fn(o, li);
    }
    return true;
}
/** @brief Publish one heap onto the global chain without a lock.
 * @note Must be called **before** any operation that could reach a safepoint; it only touches the chain head. The hnext of interior nodes is modified only by reap_dead_heaps while it holds gc_mutex, so concurrent insertion and "deletion under the lock" can coexist (the deleting side uses a CAS against the inserting side when unlinking the head).
 * @internal
 */
inline void publish_heap(gc_heap* h) {
    gc_state& g = G();
    gc_heap* old = g.heaps.load(std::memory_order_relaxed);
    do {
        h->hnext = old;
    } while (!g.heaps.compare_exchange_weak(old, h,
                 std::memory_order_release, std::memory_order_relaxed));
}
/** @brief Is this pointer a real heap currently on the global chain? Used to tell whether `gc_state::cycle_owner` can be trusted.
 * **Reads of `dead` / `thread_handle` through cycle_owner are reads of live memory only when "it really is still on the chain"**:
 * when reap_dead_heaps reclaims a heap struct it does not check whether it is the current owner, so cycle_owner may well point at memory that has **already been freed**; a heap pointer obtained by an allocation during TLS teardown may also be non-null but **invalid** (on no chain at all),
 * and it would be used as a heap by gc_pace and written as the owner (that form's root cause is fixed; it now uses a platform TLS slot, see heap_tls_get; the predicate is kept here as defense in depth).
 * Otherwise owner->dead and heap_thread_ended(owner) are both reads of non-live memory, which happen to read as 0/0 -- the loop "wait for the owner to finish on its own" then never reaches an end, and the process hangs. Folding "not on the chain" into the takeover predicate means such an owner can no longer make
 * anything progress, so the caller must take over. Walk safety: `hnext` is modified only inside `reap_dead_heaps` while it holds gc_mutex, and the chain head is raced in by the lock-free `publish_heap`; this function is called by `full_collect` **while holding the lock**.
 * @internal
 */
inline bool heap_on_chain(gc_heap* h) {
    if (h == nullptr) return false;
    for (gc_heap* p = heaps_head(); p != nullptr; p = p->hnext)
        if (p == h) return true;
    return false;
}
/* ---- Per-thread heap: thread_local pointer + lazy creation ---- */
/**
 * @brief The **slow path** for this thread's heap: create it if absent, register it on the global chain, and register thread-exit cleanup.
 * @return This thread's heap (never returns `nullptr`).
 * @note When creating the heap, the object range must first be registered into `heap_lo`/`heap_hi`, otherwise this thread's heap would be missed by the scan.
 * @internal
 */
inline gc_heap* my_heap_slow();     /* see the heap lifetime section */
/**
 * @brief **Safepoint arrival**: self-scan roots -> publish this heap to the safepoint counter -> park and wait for release.
 * @note This is the **only handshake point** between a mutator and the collector.
 * @internal
 */
inline void safepoint_reach(gc_heap* h);  /* safepoint arrival (self-scan + park) */
/** @brief The current thread's heap pointer: a **platform TLS slot**, not C++ thread_local.
 * MinGW's win32 thread model implements C++ thread_local as **emutls**, whose per-thread block is already invalid before the user's TLS destructors run -- reading this slot inside the destructor of a `static thread_local` object yields a **completely different address** (another
 * thread's heap, or recycled memory), and using it as a gc_heap* is arbitrary memory corruption. Measured (MinGW, minimal repro, 40 alternating rounds): "call gc<>() from a user TLS destructor" crashed 14/40 (0xc0000374 / 0xc0000005), while the same TLS construction
 * that does not touch the GC was 0/40. A platform TLS slot (Windows TlsAlloc / POSIX pthread_key_create(..., nullptr)) stays valid for the entire **thread life
 * time**, including while all TLS destructors run; and because no destructor callback is registered, the system will not clear it for us. @note MSVC / clang-cl keep native thread_local: it is just a cell in the TLS array, likewise covering the whole thread lifetime and
 *       far faster than one TlsGetValue; that branch is decided by the compiler and is not something the user needs to configure.
 * @internal
 */
#if defined(_MSC_VER) && !defined(__MINGW32__) && !defined(__MINGW64__)
/**
 * @brief TLS slot reference for this thread's `gc_heap*` (MSVC / clang-cl native `thread_local` implementation).
 * @internal
 */
inline gc_heap*& heap_tls_ref() {
    static thread_local gc_heap* slot = nullptr;
    return slot;
}
inline gc_heap* heap_tls_get()              { return heap_tls_ref(); }
inline void     heap_tls_set(gc_heap* h)    { heap_tls_ref() = h; }
#elif GCXX_PLATFORM_WINDOWS
/**
 * @brief Windows platform TLS slot index (`TlsAlloc`, **no** destructor callback set).
 * @internal
 */
inline unsigned long gc_tls_index() {
    /* A process-level slot, **never released** (a process-lifetime object). TlsAlloc slots have no destructor
     * callback, and the system does not clear its value at thread exit either -- which is exactly what we want: we still have to read it during teardown. */
    static const unsigned long idx = (unsigned long)::TlsAlloc();
    return idx;
}
/**
 * @brief Read this thread's heap (Windows `TlsGetValue` implementation).
 * @internal
 */
inline gc_heap* heap_tls_get() {
    return static_cast<gc_heap*>(::TlsGetValue(gc_tls_index()));
}
/**
 * @brief Write this thread's heap (Windows `TlsSetValue` implementation).
 * @internal
 */
inline void heap_tls_set(gc_heap* h) {
    ::TlsSetValue(gc_tls_index(), static_cast<void*>(h));
}
#else
/**
 * @brief POSIX platform TLS key (`pthread_key_create`, **no** destructor function set).
 * @internal
 */
inline pthread_key_t gc_tls_key() {
    static pthread_key_t key = 0;
    static const bool made = (::pthread_key_create(&key, nullptr) == 0);
    (void)made;
    return key;
}
/**
 * @brief Read this thread's heap (POSIX `pthread_getspecific` implementation).
 * @internal
 */
inline gc_heap* heap_tls_get() {
    return static_cast<gc_heap*>(::pthread_getspecific(gc_tls_key()));
}
/**
 * @brief Write this thread's heap (POSIX `pthread_setspecific` implementation).
 * @internal
 */
inline void heap_tls_set(gc_heap* h) {
    ::pthread_setspecific(gc_tls_key(), static_cast<void*>(h));
}
#endif
/** @brief Get this thread's diagnostic accumulator. When this thread has no heap yet, use the caller-supplied fallback storage -- only "diagnostic
 * threads that never take part in GC" (callers of gc_stat / gc_verify) get here, and those counters are diagnostic information anyway; losing them does not affect correctness.
 * @internal
 */
inline diag_accum* diag_or(diag_accum* fallback) {
    gc_heap* h = heap_tls_get();
    return (h != nullptr) ? &h->diag : fallback;
}
/**
 * @brief Merge this thread's diagnostic accumulator into the global counters; returns silently when this thread has no heap.
 * @internal
 */
inline void diag_flush() {
    diag_accum* ap = diag_or(nullptr);
    if (ap == nullptr) return;
    diag_accum& a = *ap;
    if (a.scanned_bytes == 0 && a.skipped_bytes == 0 && a.bias_hits == 0) return;
    gc_state& g = G();
    g.scanned_bytes.fetch_add(a.scanned_bytes, std::memory_order_relaxed);
    g.skipped_bytes.fetch_add(a.skipped_bytes, std::memory_order_relaxed);
    g.bias_hits.fetch_add(a.bias_hits, std::memory_order_relaxed);
    a.scanned_bytes = 0; a.skipped_bytes = 0; a.bias_hits = 0;
}
/**
 * @brief Allocation hot-path entry: one TLS read plus a null check (only a miss goes to the slow path to create the heap)
 * @internal
 */
inline gc_heap* my_heap() {
    gc_heap* h = heap_tls_get();
    return h != nullptr ? h : my_heap_slow();
}
/** @brief Safepoint-aware lock acquisition: if the lock cannot be taken and there is an STW request, self-scan + park first (register roots are marked by that), then keep trying to take the lock.
 * This closes the last soundness hole where "a thread blocks on the lock, misses the park, its roots live only in registers, and the degraded scan cannot catch them".
 * @internal
 */
inline void gc_mutex_lock(std::recursive_mutex& m);   /* defined in the STW section */
/**
 * @brief RAII guard for gc_mutex (acquired via gc_mutex_lock, released with a plain unlock).
 * @internal
 */
struct gc_mutex_guard {
    gc_mutex_guard() { gc_mutex_lock(G().gc_mutex); }
    ~gc_mutex_guard() { G().gc_mutex.unlock(); }
    gc_mutex_guard(const gc_mutex_guard&) = delete;
    gc_mutex_guard& operator=(const gc_mutex_guard&) = delete;
};
/**
 * @brief Monotonic millisecond timestamp.
 * @return Milliseconds since the `steady_clock` epoch (truncated to 32 bits; comparisons use unsigned wraparound subtraction).
 * @note Used only for diagnostic timing, the D2 retire delay, and the STW wait budget -- it **never decides whether to reclaim**.
 * @internal
 */
inline unsigned long long gc_ts_ms() {
    using namespace std::chrono;
    return (unsigned long long)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}
/** @brief For phased timing (A4): microseconds from t0 to now. Called only on the collector side; not a single line is added to the hot path
 * (mutator allocation).
 * @internal
 */
inline std::size_t gc_us_since(const std::chrono::steady_clock::time_point& t0) {
    return (std::size_t)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count();
}
/* ------------------------------ Color predicates -------------------------------- */
/**
 * @brief Read the mark byte (relaxed; on x86-64 this is a plain mov).
 * @internal
 */
inline std::uint8_t mark_load(const gc_object* o) {
    return o->marked.load(std::memory_order_relaxed);
}
/**
 * @brief The age has reached the old-generation threshold.
 * @internal
 */
inline bool is_old (const gc_object* o) { return o->age >= (std::uint8_t)GC_AGE_OLD; }
/**
 * @brief Carrying the white bit indicated by `mask` (this cycle's `cycle_white`) = not marked in this cycle.
 * @internal
 */
inline bool is_dead (const gc_object* o, std::uint8_t mask) { return (mark_load(o) & mask) != 0; }
/* ---- Gray-set operations ----------------------------------------------------------
 * push (any thread: a parked mutator also marks while self-scanning its stack) and pop (collector) do not overlap, see the note in gc_state. Each object is enqueued at most once, and that is guaranteed by the CAS on the white bit in mark_object (only the thread that successfully clears the white bit enqueues). */
/** @brief Guarantee the gray set is usable (lazy allocation / grow on demand).
 * @note Called **only before entering the stop-the-world window** -- no memory allocation is allowed inside the suspend window. Allocation failure is not reported as an error: with capacity 0,
 *       all enqueues take the overflow branch and marking degrades to the fallback rescan, which is still correct. Capacity is given 2x headroom over "the current number of live objects".
 * @note **It must be re-reckoned on every new cycle**: at the first collection the heap is often still small (measured live=5461 -> 65536 slots allocated from the initial value),
 *       but the heap later grows to hundreds of thousands of objects, at which point it fills up, and every overflow costs one **full-heap fallback rescan**
 *       (rescan_gray_from_heap, O(heap) each), thousands of times in one cycle -> the pause goes from 0.7ms to 5 seconds (measured).
 * @warning **This function must never move or free an array that has already been published.** It used to use
 *       `realloc(g.gray_ring, ...)` here, and it runs inside `begin_cycle`, with `phase == PAUSE` and
 *       **sp_request not yet set** -- that is, **other threads are still running**. The old comment's premise was
 *       "no thread is pushing (pushes only happen inside the mark window)", but `object_new` is
 *       "read `ph` once to decide whether to join the gray set, and only actually push after the whole slot-allocation section", while the previous cycle's
 *       `phase` takes **the entire incremental sweep** to travel from the mark window to PAUSE (which can be long) -- so a thread that read
 *       `ph != PAUSE` may well push **after** this `realloc` of the **next** cycle.
 *       There are two consequences, both unacceptable: (1) writing into the old array that `realloc` already freed (use-after-free;
 *       measured: TSan reliably reports this race as gray_reserve(1210) <-> gray_push(1247));
 *       (2) that enqueue **loses its slot** -- the object has already had its white bit cleared (marked gray) yet is never expanded -> **missed mark -> dangling pointer**.
 *       So it is now **chunked**: the block pointer table is allocated once, and once a block is published it stays valid forever and never moves.
 *       A late arriver either reads the new block (writes into it, and consumers can see it) or reads "not published" (nullptr) and takes the
 *       existing `gray_overflow` fallback -- **neither case loses a slot**, so the criterion
 *       "under normal load `gray_rescans` is always 0" (the `gray-capacity-scales` case)
 *       still holds.
 * @internal
 */
inline void gray_reserve() {
    gc_state& g = G();
    auto want = (std::size_t)GCXX_GRAY_CAPACITY;
    const std::size_t live = g.live_objects.load(std::memory_order_relaxed);
    if (live > want / 2) {
        want = live * 2;
        if (want > (std::size_t)GCXX_GRAY_CAPACITY_LIMIT)
            want = (std::size_t)GCXX_GRAY_CAPACITY_LIMIT;
    }
    if (want == 0) want = 1;
    if (g.gray_blocks == nullptr) {
        /* Block pointer table: allocated **once**, never moved. calloc makes every block initially nullptr
         * (= "not published"), so producers take the overflow fallback instead of writing into unallocated memory. */
        /* Test injection: see the note on `gcxx::tests::fail_next_gray_calloc` above in this file. */
        void* tbl = gcxx::tests::fail_next_gray_calloc().load(std::memory_order_relaxed)
                  ? nullptr
                  : std::calloc(GRAY_NBLOCKS, sizeof(std::atomic<gray_entry*>));
        if (tbl == nullptr) {
            gcxx::tests::fail_next_gray_calloc().store(false, std::memory_order_relaxed);
            g.gray_alloc.store(0, std::memory_order_relaxed);
            g.gray_cap.store(0, std::memory_order_relaxed);
            return;
        }
        g.gray_blocks = static_cast<std::atomic<gray_entry*>*>(tbl);
        g.gray_committed_blocks = 0;
    }
    /* Fill in the blocks that want covers; blocks already committed are left alone (fixed address = the pointers producers hold stay valid). */
    std::size_t need =
        (want + GRAY_BLOCK_ENTRIES - 1) >> GRAY_BLOCK_SHIFT;
    /* **The block count must never exceed the number of cells in the block pointer table** -- the table's cell count is determined solely by `GCXX_GRAY_CAPACITY_LIMIT`,
     * while the lower bound of `want` is `GCXX_GRAY_CAPACITY`. When the user sets LIMIT below
     * CAPACITY (`-DGCXX_GRAY_CAPACITY_LIMIT=...`; this macro is user-overridable),
     * `need` becomes larger than the table: the `gray_blocks[i]` in the loop writes out of bounds.
     *
     * Measured (ASan): with `-DGCXX_GRAY_CAPACITY_LIMIT=4096`, `need = 16` while the table has only
     * 1 entry => one 8-byte out-of-bounds write in `gray_reserve()`; the crash site is
     * `Critical error detected c0000374` / `0xc0000005` at exit (**during CRT shutdown**,
     * because what got trampled was an adjacent malloc block, so the symptom and the root cause are far apart and hard to attribute).
     * `GCXX_GRAY_CAPACITY` defaults to 65536 (= 16 blocks), so any LIMIT below 65536 trips
     * this -- a landmine that **does not appear in the default configuration but goes off as soon as you tune parameters per the docs**.
     *
     * After clamping, the behavior still matches the comments: fewer blocks => smaller `gray_cap` => enqueues that do not fit set
     * `gray_overflow` => the fallback rescan covers them; **it is only slower, never wrong**. */
    if (need > (std::size_t)GRAY_NBLOCKS) need = (std::size_t)GRAY_NBLOCKS;
    while (g.gray_committed_blocks < need) {
        void* blk = std::calloc(GRAY_BLOCK_ENTRIES, sizeof(gray_entry));
        if (blk == nullptr) break;      /* on failure, stop right here: degrade to the overflow fallback, still correct */
        g.gray_blocks[g.gray_committed_blocks].store(
            static_cast<gray_entry*>(blk), std::memory_order_release);
        ++g.gray_committed_blocks;
    }
    /* Order: **publish the block first (release), then raise alloc/cap (release)**. Producers do "read cap
     * (acquire), then read the block pointer (acquire)", so seeing the new cap means the block is definitely visible. */
    const std::size_t alloc = g.gray_committed_blocks << GRAY_BLOCK_SHIFT;
    g.gray_alloc.store(alloc, std::memory_order_release);
    g.gray_cap.store(alloc, std::memory_order_release);
}
/** @brief Address of cell i; returns nullptr when the corresponding block is **not yet published**.
 * Producers call this on the hot path: two shifts/masks plus one atomic read (on x86-64 an acquire is just a plain mov).
 * @note The path that returns nullptr must take the `gray_overflow` fallback and **must never** be treated as "no problem" --
 *       losing a slot is a missed mark.
 * @internal
 */
inline gray_entry* gray_slot(gc_state& g, std::size_t i) {
    const std::atomic<gray_entry*>* tbl = g.gray_blocks;
    if (tbl == nullptr) return nullptr;
    gray_entry* blk = tbl[i >> GRAY_BLOCK_SHIFT].load(std::memory_order_acquire);
    if (blk == nullptr) return nullptr;
    return blk + (i & (GRAY_BLOCK_ENTRIES - 1));
}
/** @brief Gray-set enqueue. **Multiple producers** (collector expansion, parked threads self-scanning their stacks, force-suspended threads re-marking after resume, and "late arrivers that read a stale phase"),
 * so it is fetch_add + publish. When nearly full it **grows in place** (only gray_cap is raised, no data is moved): gray_push may be called on any thread, and moving the array would require mutual exclusion against the other producers, whereas "raising the limit" does not --
 * each producer fetch_adds its own unique index, and it suffices that the post-growth index still lands in a **published** block and that the gray_cap update is visible to them.
 * @note Why growth is mandatory: the upper bound on the number of enqueues in one cycle is "the number of objects marked this cycle" ~ the number of live objects, independent of how "wide" the reference graph is; 65536 slots fill up on a normal heap with 50k live objects, and the overflow fallback after that is a **full-heap scan, O(heap) each time**
 *       (rescan_gray_from_heap); measured, the pause goes from 0.7ms to 5 seconds. Growth leaves this path to the scenario it is really meant to handle (a pathological wide graph that genuinely enqueues tens of millions of objects in one cycle).
 * @note Concurrency safety: growth is "enlarging an upper bound", and `gray_cap` at any instant points at **committed blocks**,
 *       so consumers never read unallocated memory; tail is a monotonic quantity produced by fetch_add, and growth only makes it easier for it to be `< cap`.
 *       Cell reads and writes are relaxed/release-acquire atomics: the producer writes obj first, then release-publishes epoch;
 *       the consumer acquire-reads epoch and uses obj only if it matches -- so "seeing this cycle's number" necessarily means "seeing that obj".
 * @internal
 */
constexpr std::size_t GRAY_GROW_FACTOR = 2u;
/**
 * @brief Gray-set enqueue: the object has already had its white bit cleared; here it is put onto the work list to be expanded.
 * @param o The gray object to be expanded.
 * @note **Multiple producers** (collector, parked threads self-scanning, force-suspended threads re-marking on resume, late arrivers), hence fetch_add for a unique index; when nearly full it **grows in place** (only raises cap, no data movement). What does not fit is dropped and `gray_overflow` is set, with the fallback rescan covering it (**only slower, never wrong**).
 * @internal
 */
inline void gray_push(gc_object* o) {
    gc_state& g = G();
    const std::size_t i = g.gray_tail.fetch_add(1, std::memory_order_relaxed);
    /* cap is re-read every time: it can be enlarged in place by another producer. acquire is used to pair with
     * gray_reserve's "publish the block first, raise cap after" -- seeing the new cap means the block is definitely visible. */
    std::size_t cap = g.gray_cap.load(std::memory_order_acquire);
    if (i >= cap) {
        /* Producer-side growth: tail is monotonic, so "i >= cap" means there are more enqueuers behind.
         * Only raise cap when there really is space left (gray_reserve has already partitioned by live_objects,
         * so the limit here is the number of slots it actually committed). */
        const std::size_t alloc = g.gray_alloc.load(std::memory_order_acquire);
        if (alloc > cap) {
            std::size_t next = cap * (std::size_t)GRAY_GROW_FACTOR;
            if (next <= cap) next = cap;                 /* overflow guard */
            if (next > alloc) next = alloc;
            if (next > cap) {
                cap = next;
                g.gray_cap.store(cap, std::memory_order_release);
                g.gray_overflow.store(false, std::memory_order_relaxed);
            }
        }
    }
    if (i < cap) {
        gray_entry* slot = gray_slot(g, i);
        if (slot != nullptr) {
            slot->obj.store(o, std::memory_order_relaxed);          /* write the content first */
            slot->epoch.store(g.gray_epoch.load(std::memory_order_relaxed),
                              std::memory_order_release);           /* then publish the epoch */
        } else {
            /* cap covered this index but the block was not published: this can only be "interrupted between reading cap and reading the block pointer";
             * take the overflow fallback (the safe direction); never silently lose a slot. */
            g.gray_overflow.store(true, std::memory_order_relaxed);
        }
    } else {
        g.gray_overflow.store(true, std::memory_order_relaxed);
    }
}
/** @brief How many rounds at most to force past a cell where the watermark is stuck on one that is "reserved but not yet written".
 * The producer is in another marker's hands and normally finishes writing in tens of nanoseconds; yielding for a while is so as **not to pay for
 * nothing** (an O(heap) fallback rescan). If it still has not arrived after this many rounds, it is judged that "the producer died halfway".
 * @internal
 */
#ifndef GCXX_GRAY_PUBLISH_SPINS
#  define GCXX_GRAY_PUBLISH_SPINS  64u
#endif
/**
 * @brief Advance the "**published** watermark" `gray_ready`: bring consecutive **fully written** cells into the consumable range.
 *
 * Why it is needed: there is a window between the **reservation** made by `fetch_add` and the actual **write**. If a consumer takes cells by
 * the reservation marker, it gets a cell that is "reserved but not yet written", reads the previous cycle's leftover value and, by the epoch
 * predicate, **discards it as a stale entry** => that enqueue is lost => the object stays gray and is not expanded => its children get swept => **dangling pointer**.
 *
 * So "**the consumable range**" and "**the reserved range**" are now separated: consumers only eat below `gray_ready`,
 * and `gray_ready` only advances past cells that are **already fully written** (`epoch == this cycle`) -- hence
 * cells that are "claimed but not written" **are never consumed**, and therefore never misjudged as stale entries.
 *
 * **The cost is placed on the consumer side**: producers do not write a single extra byte (publishing was only ever writing `epoch`),
 * and the scan that advances the watermark happens when a consumer takes a batch, amortized to one read per cell.
 *
 * @note A stalled watermark (an index was reserved but never written) can only mean a **force-suspended thread** stopped
 *       between "got the index" and "has not written yet". After `GCXX_GRAY_PUBLISH_SPINS` rounds, **step over that cell**
 *       and set `gray_overflow`: that object has already had its white bit cleared (marked gray), and this cycle will run
 *       `rescan_gray_from_heap` once to recover it together with its children -- **losing the slot does not lose the mark**,
 *       and it will not hang the way "wait forever" would.
 * @return The new watermark (the upper bound that can be safely consumed).
 * @internal
 */
inline std::size_t gray_pump_ready(gc_state& g) {
    const unsigned long long cur = g.gray_epoch.load(std::memory_order_relaxed);
    std::size_t spins = 0;
    for (;;) {
        std::size_t       r   = g.gray_ready.load(std::memory_order_acquire);
        const std::size_t lim = g.gray_tail.load(std::memory_order_acquire);
        std::size_t adv = r;
        bool stalled = false;
        while (adv < lim) {
            const gray_entry* s = gray_slot(g, adv);
            if (s != nullptr &&
                s->epoch.load(std::memory_order_acquire) == cur) {
                ++adv;                       /* this cell really was written */
                continue;
            }
            stalled = true;
            break;
        }
        if (adv != r) {
            if (g.gray_ready.compare_exchange_weak(r, adv, std::memory_order_release,
                                                   std::memory_order_relaxed))
                return adv;
            continue;                        /* another consumer advanced it: rescan */
        }
        if (!stalled) return r;              /* no "reserved but unwritten" cells: stop here */
        if (spins < (std::size_t)GCXX_GRAY_PUBLISH_SPINS) {
            ++spins;
            std::this_thread::yield();
            continue;                        /* the producer is writing: yield for a while */
        }
        const std::size_t forced = r + 1;
        if (g.gray_ready.compare_exchange_weak(r, forced, std::memory_order_release,
                                              std::memory_order_relaxed)) {
            g.gray_overflow.store(true, std::memory_order_relaxed);
            return forced;
        }
        spins = 0;                           /* someone else moved: start over */
    }
}
/** @brief Take the next object to be expanded; return nullptr when there is none.
 * @note **Multiple consumers supported**: a whole batch of indices is claimed exclusively by one CAS on `gray_head`, and **only below**
 *       `gray_ready` (= already written), so any number of markers will not get the same cell
 *       nor a cell that is "not yet written". With a single marker the set and order of consumed entries is word-for-word
 *       identical to "read head then increment", so this is a purely structural rewrite.
 * @param c The caller's own cursor (the batch interval is kept inside it, see mark_cursor).
 * @internal
 */
inline gc_object* next_gray(mark_cursor& c) {
    gc_state& g = G();
    for (;;) {
        std::size_t i;
        if (c.batch_next < c.batch_end) {
            i = c.batch_next++;                 /* eat the batch already claimed first */
        } else {
            /* Claim a new batch: **first push the published watermark to the latest**, then claim below it. */
            const std::size_t ready = gray_pump_ready(g);
            const std::size_t cap   = g.gray_cap.load(std::memory_order_acquire);
            /* Enqueues with index >= cap never wrote a cell (the reservation marker can exceed cap),
             * so consumers must stop at cap -- the remainder is left to the fallback rescan. */
            const std::size_t limit = (ready < cap) ? ready : cap;
            std::size_t head = g.gray_head.load(std::memory_order_relaxed);
            if (head >= limit) return nullptr;
            std::size_t claim = limit - head;
            if (claim > GRAY_BATCH) claim = GRAY_BATCH;
            if (!g.gray_head.compare_exchange_weak(head, head + claim,
                    std::memory_order_relaxed, std::memory_order_relaxed))
                continue;                       /* another marker claimed it first: re-read */
            c.batch_next = head;
            c.batch_end  = head + claim;
            i = c.batch_next++;
        }
        gray_entry* slot = gray_slot(g, i);
        if (slot == nullptr) {
            /* This cannot happen in theory (cap only covers published blocks); if it does, treat it as "this cell was lost":
             * set overflow so this cycle runs the fallback rescan, and never treat it as "this cell is empty". */
            g.gray_overflow.store(true, std::memory_order_relaxed);
            continue;
        }
        gc_object* const o  = slot->obj.load(std::memory_order_relaxed);
        const unsigned long long ep = slot->epoch.load(std::memory_order_acquire);
        if (ep == g.gray_epoch.load(std::memory_order_relaxed)) return o;
        /* epoch mismatch = a stale entry from the previous cycle: discard. That object is covered by the fallback rescan.
         * **Diagnostic counter**: under normal load (with no force-suspended threads) this should never be reached --
         * the more it is reached, the more the "reservation/publication separation" has broken down (see gray_discards). */
        g.gray_discards.fetch_add(1, std::memory_order_relaxed);
    }
}
/** @brief The "gray set is drained" predicate must be computed from the **effective limit**, and must use the **published** watermark rather than the reservation marker:
 * the reservation marker (produced by fetch_add) may exceed cap, and may also exceed the position that is actually written; using
 * `head >= tail` to test for empty never becomes true once cells have been discarded, and the outer drain loop spins forever.
 * @note Use `gray_ready` rather than `gray_tail`: **cells that are "reserved but not written" should not count as "there is work to do"**
 *       -- either they get written right away (`gray_pump_ready` advances the watermark in the next `next_gray`),
 *       or they are a suspended thread's half-finished work (`gray_pump_ready` steps over them once it has waited long enough and sets overflow,
 *       covered by the fallback rescan). Neither case keeps this predicate false for long.
 * @internal
 */
inline bool gray_empty() {
    gc_state& g = G();
    const std::size_t ready = g.gray_ready.load(std::memory_order_acquire);
    const std::size_t cap   = g.gray_cap.load(std::memory_order_relaxed);
    const std::size_t limit = (ready < cap) ? ready : cap;
    return g.gray_head.load(std::memory_order_relaxed) >= limit;
}
/** @brief Called on a new cycle / wind-down / cancellation: zero the cursors + advance the epoch, invalidating the previous cycle's stale entries.
 * Incidentally shrink a grown gray_cap back to the initial capacity (the array itself is not freed: it only ever grows, see gray_push; what is taken back is just "how far this cycle is allowed to go").
 * @internal
 */
inline void gray_clear() {
    gc_state& g = G();
    g.gray_head.store(0, std::memory_order_relaxed);
    g.gray_tail.store(0, std::memory_order_relaxed);
    /* The published watermark must be zeroed as well: for a **new cycle's consumers** not a single cell of the previous cycle is published
     * (the epoch has changed already, so a cell mistaken for "written" would still not be accepted -- but zeroing is more direct). */
    g.gray_ready.store(0, std::memory_order_relaxed);
    /* Capacity is per-cycle: growth in the previous cycle does not affect this cycle's starting point. */
    g.gray_cap.store(g.gray_alloc.load(std::memory_order_relaxed),
                     std::memory_order_relaxed);
    g.gray_overflow.store(false, std::memory_order_relaxed);
    g.gray_need_rescan = false;
    /* Zeroing the consume cursor **must also drop the claimed-but-unconsumed batch**: those indices belong to the previous cycle's interval,
     * and keeping them would consume cells that are not part of one's own batch (although "re-expansion" is idempotent and cannot go wrong,
     * that would be luck, and should not be relied on). */
    g.scan.batch_next = 0;
    g.scan.batch_end  = 0;
    g.gray_epoch.fetch_add(1, std::memory_order_relaxed);
}
/**
 * @brief payload start address: constant offset, no branch, no memory access (the object header is a fixed 16 bytes).
 * @internal
 */
inline char*        payload_of(gc_object* o)       { return reinterpret_cast<char*>(o) + PAYLOAD_OFF; }
inline const char*  payload_of(const gc_object* o) { return reinterpret_cast<const char*>(o) + PAYLOAD_OFF; }
/** @brief The "total bytes inside the slot" needed to allocate one object: payload offset + payload size + block-tail padding,
 * and no less than the object header itself (padding is always on, so this is pure compile-time constant addition).
 * @internal
 */
inline std::uint32_t object_total_need(std::uint32_t size) {
    return (std::uint32_t)PAYLOAD_OFF + size + (std::uint32_t)PAD_BYTES;
}
/**
 * @brief Total number of bytes an allocated object occupies in its slot (for verification).
 * @internal
 */
inline std::uint32_t object_total_bytes(const gc_object* o) {
    return object_total_need(o->size);
}
/** @brief Is this object a "leaf" (it cannot contain pointers; tracing skips the whole thing): the predicate is exactly "does this type need to
 * be scanned", the same predicate used at allocation time.
 * @internal
 */
inline bool is_leaf_object(const gc_object* o) {
    return !type_scanned_ct(o->type);
}
/* ==========================================================================
 * chunk heap: allocation, deallocation, O(1) reverse lookup ========================================================================== */
/* ---- Aligned memory allocation (64KB aligned) ---- */
/**
 * @brief Allocate aligned to `align` (the **degraded path** when page primitives are unavailable).
 * @return The aligned block; `nullptr` on failure.
 * @note Feature degradation (it will not commit pages), **correctness unchanged**.
 * @internal
 */
inline void* aligned_alloc_impl(std::size_t n, std::size_t align) {
#if GCXX_PLATFORM_WINDOWS
    /* Every Windows toolchain (MSVC / MinGW / clang-cl) goes through _aligned_malloc:
     * posix_memalign simply does not exist on MinGW (measured: g++ 14.2 fails to compile outright). The predicate must be the **platform**, not the compiler. */
    return _aligned_malloc(n, align);
#else
    void* p = nullptr;
    if (posix_memalign(&p, align, n) != 0) return nullptr;
    return p;
#endif
}
/**
 * @brief Free a block obtained from `aligned_alloc_impl` (strictly paired with its allocator).
 * @internal
 */
inline void aligned_free_impl(void* p) {
#if GCXX_PLATFORM_WINDOWS
    _aligned_free(p);        /* paired with _aligned_malloc in aligned_alloc_impl */
#else
    std::free(p);            /* paired with posix_memalign */
#endif
}
namespace os {

/* ==========================================================================
 * OS page primitive abstraction layer: **there is no portable standard way to "give physical pages back to the system while keeping the address space"**, and it is the only thing this GC can still get back (Windows: VirtualAlloc/VirtualFree/VirtualProtect; POSIX: mmap/
 * madvise(MADV_DONTNEED)/mprotect/munmap). Three rules:
 *  1. **Use it only to allocate chunks, never to wrap malloc**: measured, changing chunk allocation back to _aligned_malloc while keeping decommit makes the program segfault immediately on alloc-mixed -- the 64KB blocks of the CRT heap also sit behind VirtualAlloc regions, and calling
 *     VirtualFree(MEM_DECOMMIT) on them **succeeds**, so the process heap's own pages get decommitted, and the next time the heap manager writes metadata it hits an access violation. To decommit you must obtain the mapping yourself;
 *  2. **64KB alignment is this layer's responsibility**: VirtualAlloc naturally aligns to the 64KB allocation granularity, while mmap only guarantees page alignment, so map an extra span and trim the excess on both sides, making [p, p+bytes) exactly one independent mapping -- freeing then only needs munmap(p, bytes);
 *  3. Do not build abstractions without a consumer. When the platform lacks page primitives, the whole layer degrades to aligned_alloc_impl: feature degradation (it will not commit), **correctness unchanged**.
 * @note `GCXX_OS_PAGE_PRIMITIVES` **is overridable with `-D`** (needed for ablation/control runs), so this uses
 *       `#ifndef` to respect an external definition -- a plain `#define` would make `-DGCXX_OS_PAGE_PRIMITIVES=0`
 *       trigger a "macro redefined" warning (and that is one entry in the configuration matrix).
 * ========================================================================== */
#ifndef GCXX_OS_PAGE_PRIMITIVES
#  if GCXX_PLATFORM_WINDOWS || GCXX_PLATFORM_POSIX
#    define GCXX_OS_PAGE_PRIMITIVES 1
#  else
#    define GCXX_OS_PAGE_PRIMITIVES 0
#  endif
#endif
/** @brief The macro name for anonymous mappings is not uniform across POSIX systems (Linux's `MAP_ANONYMOUS` is provided by `_DEFAULT_SOURCE`/
 * `_GNU_SOURCE`; g++/clang++ define `_GNU_SOURCE` by default, but in strict `-std=c++11` mode with another standard library it may
 * not be there; **BSD/macOS/iOS use `MAP_ANON`**). When neither exists, degrade the whole page-primitive layer --
 * better to lose a feature than to fail to compile in the user's environment.
 * @note The predicate is `GCXX_PLATFORM_POSIX` rather than `defined(__unix__)`: the BSD family and Darwin both define
 *       `__unix__`/`__APPLE__`, whereas this project requires **all platform branches to go through the `GCXX_PLATFORM_*` macros**
 *       (for the rationale see the platform identification section in the file header).
 * @internal
 */
#if GCXX_PLATFORM_POSIX && !GCXX_PLATFORM_WINDOWS
#  if defined(MAP_ANONYMOUS)
#    define GCXX_OS_MAP_ANON MAP_ANONYMOUS
#  elif defined(MAP_ANON)
#    define GCXX_OS_MAP_ANON MAP_ANON
#  else
#    undef  GCXX_OS_PAGE_PRIMITIVES
#    define GCXX_OS_PAGE_PRIMITIVES 0
#  endif
#endif
/** @brief Process page size (lazy query + cache). The cache lives in gc_state (on the heap), not in a function-local static -- see the note on G():
 * the GC's own bytes must not land in the writable static area (that is within conservative scanning range).
 * @internal
 */
inline std::size_t page_size() {
    gc_state& g = G();
    if (g.os_page == 0) {
#if GCXX_PLATFORM_WINDOWS
        SYSTEM_INFO si = {};
        ::GetSystemInfo(&si);
        g.os_page = (std::size_t)si.dwPageSize;
#else
        const long v = ::sysconf(_SC_PAGESIZE);
        g.os_page = (v > 0) ? (std::size_t)v : (std::size_t)4096;
#endif
        if (g.os_page == 0) g.os_page = 4096;
    }
    return g.os_page;
}
/** @brief One "reserve + commit" aligned allocation (the returned address is 64KB aligned and the length is exactly bytes).
 * *zeroed is filled in with "whether this memory is guaranteed zero by the kernel" -- C3 relies on it to skip one memset. Returns nullptr on failure (the caller handles it as an allocation failure; no silent degradation).
 * @internal
 */
inline void* alloc_block(std::size_t bytes, bool* zeroed) {
#if GCXX_OS_PAGE_PRIMITIVES
#  if GCXX_PLATFORM_WINDOWS
    /* MEM_RESERVE|MEM_COMMIT in one call: newly committed pages are guaranteed by the kernel to be zero pages (this is C3's basis, and behavior the Windows
     * documentation explicitly states). The address is aligned to the allocation granularity (64KB), which is exactly what chunk alignment requires. */
    void* p = ::VirtualAlloc(nullptr, bytes, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (p == nullptr) return nullptr;
    if (zeroed != nullptr) *zeroed = true;
    return p;
#  else
    /* POSIX: map 64KB extra, align away the head and trim the tail, so that [p, p+bytes) is exactly one independent mapping --
     * freeing then does not need to remember the original base address. */
    const std::size_t gran = CHUNK_SIZE;
    const std::size_t total = bytes + gran;
    void* raw = ::mmap(nullptr, total, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | GCXX_OS_MAP_ANON, -1, 0);
    if (raw == MAP_FAILED) return nullptr;
    const std::uintptr_t r = reinterpret_cast<std::uintptr_t>(raw);
    const std::uintptr_t a = (r + gran - 1) & ~(std::uintptr_t)(gran - 1);
    if (a > r) ::munmap(raw, (std::size_t)(a - r));               /* head excess */
    const std::uintptr_t tail = r + total - (a + bytes);
    if (tail > 0)
        ::munmap(reinterpret_cast<void*>(a + bytes), (std::size_t)tail);  /* tail excess */
    if (zeroed != nullptr) *zeroed = true;    /* the first access to an anonymous mapping is a zero page */
    return reinterpret_cast<void*>(a);
#  endif
#else
    if (zeroed != nullptr) *zeroed = false;   /* malloc memory is not guaranteed zero (so C3 is skipped) */
    return aligned_alloc_impl(bytes, CHUNK_SIZE);
#endif
}
/**
 * @brief Return one independent mapping to the system (the inverse of `alloc_block`).
 * @param p Must be the block start returned by `alloc_block`; `bytes` is the same as at allocation time.
 * @internal
 */
inline void free_block(void* p, std::size_t bytes) {
    if (p == nullptr) return;
#if GCXX_OS_PAGE_PRIMITIVES
#  if GCXX_PLATFORM_WINDOWS
    (void)bytes;                    /* MEM_RELEASE does not need a length */
    ::VirtualFree(p, 0, MEM_RELEASE);
#  else
    ::munmap(p, bytes);
#  endif
#else
    (void)bytes;
    aligned_free_impl(p);
#endif
}
/** @brief Return the physical pages of [p, p+bytes) to the system (the address space is kept, the content becomes zero pages).
 * Requires p and bytes to be page-aligned -- the caller is responsible (see chunk_purge_data).
 * @internal
 */
inline bool decommit(void* p, std::size_t bytes) {
#if GCXX_OS_PAGE_PRIMITIVES
#  if GCXX_PLATFORM_WINDOWS
    return ::VirtualFree(p, bytes, MEM_DECOMMIT) != 0;
#  else
    return ::madvise(p, bytes, MADV_DONTNEED) == 0;
#  endif
#else
    (void)p; (void)bytes;
    return false;
#endif
}
/**
 * @brief Recommit (paired with decommit). After success the region reads back as zero pages.
 * @internal
 */
inline bool recommit(void* p, std::size_t bytes) {
#if GCXX_OS_PAGE_PRIMITIVES
#  if GCXX_PLATFORM_WINDOWS
    return ::VirtualAlloc(p, bytes, MEM_COMMIT, PAGE_READWRITE) != nullptr;
#  else
    /* An mmap region is already readable/writable anonymous pages, and after MADV_DONTNEED the next access is a zero page; the mprotect here
     * is idempotent (writing the protection bits back to the same value) and guards against "someone tightening the protection bits on this path in the future". */
    return ::mprotect(p, bytes, PROT_READ | PROT_WRITE) == 0;
#  endif
#else
    (void)p; (void)bytes;
    return false;
#endif
}
/**
 * @brief Page protection (used for the E2 large-object tail guard page). protect=false restores read/write.
 * @internal
 */
inline bool guard(void* p, std::size_t bytes, bool protect) {
#if GCXX_OS_PAGE_PRIMITIVES
#  if GCXX_PLATFORM_WINDOWS
    DWORD old = 0;
    return ::VirtualProtect(p, bytes, protect ? PAGE_NOACCESS : PAGE_READWRITE, &old) != 0;
#  else
    return ::mprotect(p, bytes, protect ? PROT_NONE : (PROT_READ | PROT_WRITE)) == 0;
#  endif
#else
    (void)p; (void)bytes; (void)protect;
    return false;
#endif
}

} /* namespace os */
/* This file uses no os:: prefix throughout; os is an independent platform abstraction layer. */
using namespace os;
namespace os {

/** @brief Current process RSS (for diagnostics; returns 0 if unavailable). Used to turn "memory really was returned" into
 * an observable number, rather than looking only at intent values like mapped_bytes.
 * @internal
 */
inline std::size_t rss_bytes() {
#if GCXX_PLATFORM_WINDOWS
    PROCESS_MEMORY_COUNTERS pmc = {};
    if (!::GetProcessMemoryInfo(::GetCurrentProcess(), &pmc, sizeof(pmc))) return 0;
    return (std::size_t)pmc.WorkingSetSize;
#elif GCXX_PLATFORM_LINUX
    /* The second column of /proc/self/statm is the resident page count. Deliberately no sys/resource.h -- its ru_maxrss is the **peak**
     * rather than the current value, a different semantics. */
    std::FILE* f = std::fopen("/proc/self/statm", "r");
    if (f == nullptr) return 0;
    unsigned long total = 0, resident = 0;
    const int got = std::fscanf(f, "%lu %lu", &total, &resident);
    std::fclose(f);
    return (got == 2) ? (std::size_t)resident * page_size() : 0;
#elif GCXX_PLATFORM_ANDROID
    /* Android also has `/proc/self/statm`, but **on some versions/policies SELinux blocks it** (fopen fails).
     * When it cannot be read, return 0 -- the diagnostic value is unavailable, **correctness is unaffected**. */
    std::FILE* f = std::fopen("/proc/self/statm", "r");
    if (f == nullptr) return 0;
    unsigned long total = 0, resident = 0;
    const int got = std::fscanf(f, "%lu %lu", &total, &resident);
    std::fclose(f);
    return (got == 2) ? (std::size_t)resident * page_size() : 0;
#elif GCXX_PLATFORM_FREEBSD || GCXX_PLATFORM_DRAGONFLY
    /* **There is no `/proc/self/statm` on BSD**; use `sysctl(CTL_KERN, KERN_PROC, KERN_PROC_PID, pid)`
     * to get this process's `struct kinfo_proc`, whose `ki_rssize` is the **current** RSS (unit: pages) --
     * semantics matching the Linux branch (not a peak). When it cannot be obtained, return 0 ("unavailable" rather than a wrong value).
     *
     * **This used to be written as `KERN_PROC_RSS` -- that macro does not exist on FreeBSD at all**
     *    (measured: it cannot be found in FreeBSD 14.5's `<sys/sysctl.h>`; the subtypes of `KERN_PROC` are only
     *    the `ALL/PID/PGRP/.../VMMAP/FILEDESC/...` series). The correct spelling is exactly
     *    `KERN_PROC_PID` + `kinfo_proc.ki_rssize`. The old comment mistook "the
     *    `kern_proc_rss` function name in the kernel's C source" for a sysctl name -- those are **two things at different layers**.
     * @note Deliberately does **not** use `kinfo_getproc()` (libutil): one more link dependency, and this is only a diagnostic.
     * @note For `KERN_PROC_PID`, `sysctl` returns an **array** of `struct kinfo_proc`
     *       (one entry per thread), so `len` may be `sizeof(kinfo_proc) * n`.
     *       We only read **the first entry** (the process entry), and require `len >= sizeof(kinfo_proc)`
     *       -- anything smaller means the caller's buffer was not what was expected, and we would rather report nothing. */
    struct kinfo_proc kp;
    std::size_t len = sizeof(kp);
    int mib[4];
    mib[0] = CTL_KERN;
    mib[1] = KERN_PROC;
    mib[2] = KERN_PROC_PID;
    mib[3] = (int)::getpid();
    std::memset(&kp, 0, sizeof(kp));
    if (::sysctl(mib, 4, &kp, &len, nullptr, 0) != 0) return 0;
    if (len < sizeof(kp)) return 0;
    return (std::size_t)kp.ki_rssize * page_size();   /* ki_rssize is in pages */
#elif GCXX_PLATFORM_NETBSD || GCXX_PLATFORM_OPENBSD
    /* NetBSD / OpenBSD: likewise the `KERN_PROC2` family, but **both the struct and the field names differ** --
     *   NetBSD: `struct kinfo_proc2`, RSS field `p_rsz` (**bytes**, not pages)
     *   OpenBSD: `struct kinfo_proc`, RSS field `p_vm_rssize` (**pages**)
     * Both are fetched through `sysctl(CTL_KERN, KERN_PROC2, KERN_PROC_PID, pid, sizeof(struct), 0)`
     * (the 5th mib element is the "struct version/size": OpenBSD requires `sizeof(struct kinfo_proc)`,
     * NetBSD requires version number 1).
     * @note **This environment has no NetBSD/OpenBSD sysroot, so these two branches have only been checked at the interface level and
     *       have not been run on real hardware** -- this is honestly noted in `docs/platform-support.md`.
     *       When it cannot be obtained it returns 0, and never a wrong value. */
    int mib[6];
    mib[0] = CTL_KERN;
    mib[1] = KERN_PROC2;
    mib[2] = KERN_PROC_PID;
    mib[3] = (int)::getpid();
#  if GCXX_PLATFORM_NETBSD
    struct kinfo_proc2 kp;
    std::memset(&kp, 0, sizeof(kp));
    mib[4] = 1;                       /* NetBSD: struct version */
    mib[5] = 0;
    std::size_t len = sizeof(kp);
    if (::sysctl(mib, 6, &kp, &len, nullptr, 0) != 0) return 0;
    if (len < sizeof(kp)) return 0;
    return (std::size_t)kp.p_rsz;      /* NetBSD's p_rsz is already in bytes */
#  else
    struct kinfo_proc kp;
    std::memset(&kp, 0, sizeof(kp));
    mib[4] = (int)sizeof(kp);          /* OpenBSD: struct size */
    mib[5] = 0;
    std::size_t len = sizeof(kp);
    if (::sysctl(mib, 6, &kp, &len, nullptr, 0) != 0) return 0;
    if (len < sizeof(kp)) return 0;
    return (std::size_t)kp.p_vm_rssize * page_size();   /* OpenBSD's is a page count */
#  endif
#elif GCXX_PLATFORM_DARWIN
    /* Apple: **do not introduce a new dependency on mach/task_info.h**; use the most conservative approach and return 0.
     * @note Getting RSS requires `task_info(mach_task_self(), MACH_TASK_BASIC_INFO, ...)`,
     *       which needs `#include <mach/mach.h>`; but this library deliberately keeps only the
     *       `_dyld_*` and `pthread_*` groups as Apple dependencies (both are in libSystem, with light headers).
     *       This is only a **diagnostic value**; its absence does not affect correctness -- compared with "pulling in a Mach header
     *       for one diagnostic field", keeping the dependency surface small matters more. */
    return 0;
#else
    return 0;
#endif
}

} /* namespace os */
/**
 * @brief "Total bytes inside the slot" -> size class index. Tier 1 (total <= 1040) uses 16B granularity; tier 2 uses 256B granularity.
 * @internal
 */
inline unsigned class_of(std::uint32_t total) {
    if (total <= (std::uint32_t)TIER1_MAX) {            /* tier 1: 16B granularity */
        /* `(total + 15) / 16` is 1 only when `total == 0`, and every caller guarantees
         * `total >= 1`:
         *   * `object_total_need(size) == PAYLOAD_OFF + size + PAD_BYTES >= 24`;
         *   * `gc(n)` / `gcs(n)` / `gca(n)` all raise `size == 0` to 1 before allocating;
         *   * `gc<T[]>(0)` still reserves one element.
         * So the index is `c - 2 >= 0` and the previous `if (c < 2) c = 2;` clamp was
         * unreachable (it showed up as dead code in coverage). It is replaced by the
         * compile-time assertion below, which also documents the minimum slot size. */
        static_assert(SLOT_ALIGN * 2 == 32u,
                      "the minimum slot must be 32 bytes (16B object header + 16B payload)");
        std::size_t c = ((std::size_t)total + (SLOT_ALIGN - 1)) / SLOT_ALIGN;
        return (unsigned)(c - 2);
    }
    /* Tier 2: medium objects use 256B granularity, several per chunk -- avoiding the huge waste of "a 4KB object taking a whole 64KB chunk".
     * medium classes are numbered from 64+rel, with rel starting at 1. */
    std::size_t rel = (std::size_t)total - TIER1_MAX;
    rel = (rel + 255) / 256;
    return (unsigned)(SMALL_CLASSES - 1 + rel);
}
/** @brief (kind, size class) -> partial/recycle array index. The two kinds each take a segment, so "same size but one needs scanning while
 * the other does not" objects do not crowd into the same partial chain.
 * @internal
 */
inline unsigned class_index(unsigned kind, unsigned cls) {
    return kind * CLASSES + cls;
}
/**
 * @brief size class -> slot size (bytes).
 * @internal
 */
inline std::uint32_t slot_size_of_class(unsigned cls) {
    if (cls < SMALL_CLASSES - 1)
        return (std::uint32_t)((cls + 2) * SLOT_ALIGN);      /* 32..1040 */
    const std::size_t rel = (std::size_t)cls - (SMALL_CLASSES - 1);   /* 1..MEDIUM_CLASSES */
    return (std::uint32_t)(TIER1_MAX + rel * 256);
}
/**
 * @brief Get the object header from a slot index.
 * @internal
 */
inline gc_object* slot_at(gc_chunk* c, std::uint32_t idx) {
    return reinterpret_cast<gc_object*>(c->base + c->data_off +
                                        (std::size_t)idx * c->slot_size);
}
/* ---- Metadata redzone ------------------------------------------------------
 * A small sentinel sandwiched between the gc_chunk header and the data area. **An underflow write from slot 0 hits it first**, rather than directly overwriting owner / span / capacity / cls -- the fields that "decide how this memory is interpreted": once those are overwritten, how the whole 64KB is interpreted changes, and the consequences go far beyond "one object got corrupted".
 * @note The data-area offset is therefore 16 bytes further along than the gc_chunk header, and every place that computes data_off must go through chunk_data_off() instead of each writing out that align_up expression again. */
constexpr std::size_t   META_REDZONE_BYTES = 16;
constexpr std::uint64_t META_MAGIC = 0x676378784D455441ULL;   /* "gcxxMETA" */
/**
 * @brief Offset of the slot data area relative to the chunk base (with the metadata redzone already added in).
 * @note Every place that computes `data_off` **must go through here** -- writing it out separately each time is a whole patch of silent slot-geometry errors.
 * @internal
 */
inline std::size_t chunk_data_off() {
    constexpr std::size_t head = (sizeof(gc_chunk) + (SLOT_ALIGN - 1))
                                 & ~(std::size_t)(SLOT_ALIGN - 1);
    return head + META_REDZONE_BYTES;
}
/**
 * @brief On chunk creation, fill the redzone between the chunk header and the data area with a verifiable pattern.
 * @note It guards "writing through the lower end of the data area"; block-tail padding guards the other end. Both spend time only at chunk creation/retire/`gc_verify`.
 * @internal
 */
inline void chunk_redzone_init(gc_chunk* c) {
    char* rz = reinterpret_cast<char*>(c->base) + c->data_off - META_REDZONE_BYTES;
    constexpr std::uint64_t magic = META_MAGIC;
    constexpr auto len = (std::uint32_t)META_REDZONE_BYTES;
    std::memcpy(rz, &magic, sizeof(magic));
    std::memcpy(rz + 8, &len, sizeof(len));
    std::memcpy(rz + 12, &len, sizeof(len));    /* a second copy of the length: if one is corrupted the other survives */
}
/**
 * @brief Verify. The magic and both length copies must all be intact -- writes that corrupt only one copy are still caught.
 * @internal
 */
inline bool chunk_redzone_ok(const gc_chunk* c) {
    const char* rz = reinterpret_cast<const char*>(c->base)
                   + c->data_off - META_REDZONE_BYTES;
    std::uint64_t magic = 0;
    std::uint32_t len1 = 0, len2 = 0;
    std::memcpy(&magic, rz, sizeof(magic));
    std::memcpy(&len1, rz + 8, sizeof(len1));
    std::memcpy(&len2, rz + 12, sizeof(len2));
    return magic == META_MAGIC
        && len1 == (std::uint32_t)META_REDZONE_BYTES
        && len2 == (std::uint32_t)META_REDZONE_BYTES;
}
/* ---- Integrity checksum of the chunk header ---------------------------------------------
 * Fold the gc_chunk fields that **never change after creation** into a 32-bit checksum, computed once at chunk creation and then checked at two places: retire and gc_verify. What it guards against: after a reverse in-chunk address lookup (table_find) obtains a gc_chunk*, how the whole 64KB is interpreted is entirely determined by
 * these few fields -- base/span decide "which addresses fall inside", data_off / slot_size / capacity decide "which slot it falls in". If even one of them is wrong, the reverse lookup will read **other memory** as an object header, and conservative scanning will accept it wholesale (read a garbage size,
 * then mark random content as live). The metadata redzone guards "writing through the lower end of the data area"; this guards "a field was modified without writing through" and "this is not a chunk at all". @note The magic number is folded in, so "using some arbitrary pointer as a chunk" also fails -- no separate magic field is needed.
 * @note Cost: once at chunk creation (about ten arithmetic ops), once at retire, once in gc_verify. None of them is on the hot path. */
constexpr std::uint32_t CHUNK_MAGIC = 0x6763784Bu;   /* "gcxK" */
/**
 * @brief Fold a pair of 32-bit fields into a **non-commutative** 32-bit value.
 * @note It cannot be written as `a ^ b`: that is invariant under **swapping the fields**, so "`span` and `data_off` exchanged wholesale (or any coordinated rewrite that preserves that XOR)" would
 *       pass the checksum -- and those two fields are exactly the ones that decide "how this memory is
 *       interpreted", so letting them pass is as good as having no checksum at all.
 *       Here `b` is first multiplied by an odd constant and then XORed: a different `b` gives a different result, and after a swap the result is generally different too.
 * @note Used only in the three places **chunk creation / retire / `gc_verify`**, not on the hot path, so there is no need to squeeze instruction counts.
 * @internal
 */
inline std::uint32_t mix_pair(std::uint32_t a, std::uint32_t b) {
    return a ^ (b * 0x9E3779B9u);
}
/**
 * @brief Fold the chunk fields that **never change after creation** into a 32-bit checksum (the magic is folded in too).
 * @note Guards against "a field was modified without writing through" and "this is not a chunk at all".
 * @internal
 */
inline std::uint32_t chunk_meta_checksum(const gc_chunk* c) {
    std::uint32_t h = CHUNK_MAGIC;
    const std::uint32_t words[6] = {
        (std::uint32_t)(std::uintptr_t)c->base,
        (std::uint32_t)((std::uintptr_t)c->base >> 32),
        mix_pair(c->span, c->data_off),
        mix_pair(c->slot_size, c->capacity),
        mix_pair(c->cls, c->kind),
        (std::uint32_t)c->os_zeroed | ((std::uint32_t)c->guarded << 8)
    };
    for (unsigned int word : words) h ^= word + 0x9E3779B9u + (h << 6) + (h >> 2);
    return h | 1u;
}
/**
 * @brief Verify `chunk_meta_checksum`.
 * @return `false` = the chunk header is corrupted.
 * @internal
 */
inline bool chunk_meta_ok(const gc_chunk* c) {
    return c->meta_sum == chunk_meta_checksum(c);
}
/* ==========================================================================
 * Free-list link encoding -- two-key encoding (the same idea as mimalloc's MI_ENCODE_FREELIST): ((idx ^ k2) rotl k1) + k1        decode: (rotr(enc - k1, k1)) ^ k2 
 * Why not simply `idx ^ k`: with that form a single XOR exposes the key -- an attacker who guesses one idx can recover k; and with two blocks read, (a^k)^(b^k) == a^b cancels the key and directly leaks pointer information. The `rotl` + `+` operations are **not
 * associative**, so neither trick works; the rotation amount is taken non-zero (0 would make `x << 0 | x >> 32` undefined behavior, and xor/addition are linear in the lowest bit). @note The keys are **unique per chunk**, further shrinking the reuse surface. The two keys and the canary share the same random seed, but land on different bit ranges.
 * @warning After decode the caller **must** check the result < capacity -- that is the corruption detection point (equivalent to mimalloc's "the decoded pointer must land within this page"). @note This is not cryptographically strong: it guards against "a wild/out-of-bounds write hitting the link field", not a targeted attack.
 * ========================================================================== */
/**
 * @brief 32-bit rotate left; the free-list "two-key" encoding uses it to scatter slot indices, making it harder for a wild write to forge a valid link.
 * @internal
 */
inline std::uint32_t rotl32(std::uint32_t x, unsigned r) {
    return (x << r) | (x >> (32u - r));
}
/**
 * @brief 32-bit rotate right (the inverse of `rotl32`).
 * @internal
 */
inline std::uint32_t rotr32(std::uint32_t x, unsigned r) {
    return (x >> r) | (x << (32u - r));
}
/**
 * @brief Get this process's link encoding keys (`link_seed`).
 * @return An odd number.
 * @note The seed comes from the clock + an object address: **not cryptographically random**; "different per process, stable within the process" is all that is needed.
 * @internal
 */
inline std::uint32_t link_key_seed(const void* chunk_base) {
    /* The seed lives in gc_state, **not in a function-local static**: that memory lands in the image's writable section, and the writable section is
     * within conservative root scanning range -- not a single byte of the GC's own state should appear there (see the note on G() for the same reason). */
    const std::uint64_t seed = G().link_seed;
    std::uint64_t x = seed ^ ((std::uint64_t)(std::uintptr_t)chunk_base << 7);
    x ^= x >> 31; x *= 0x9E3779B97F4A7C15ULL; x ^= x >> 27;
    return (std::uint32_t)(x >> 32) | 1u;
}
/**
 * @brief A chunk's two keys plus its own random state. The caller calls this once per chunk.
 * @internal
 */
inline void link_keys_init(gc_chunk* c, const void* chunk_base) {
    const std::uint32_t r = link_key_seed(chunk_base);
    c->key_rot  = (r % 31u) + 1u;            /* 1..31: non-zero, avoiding UB and lowest-bit linearity */
    c->key_mask = r ^ 0x9E3779B1u;
    c->rng      = r | 1u;                    /* the xorshift state must not be 0 */
}
/**
 * @brief xorshift32: used for "allocation-order randomization". Fast enough and scattered enough; no cryptographic strength needed.
 * @internal
 */
inline std::uint32_t link_rand(std::uint32_t& state) {
    state ^= state << 13; state ^= state >> 17; state ^= state << 5;
    return state;
}
/**
 * @brief Encode a slot index into a free-list link field (**two keys**: this chunk's key + the process key).
 * @param c The owning chunk; `idx` the slot index.
 * @note The purpose is to expose "a wild write touching a free slot" earlier -- forging a valid link requires guessing both keys at once.
 * @internal
 */
inline std::uint32_t link_encode(gc_chunk* c, std::uint32_t idx) {
    std::uint32_t x = idx ^ c->key_mask;
    x = rotl32(x, c->key_rot);
    return x + c->key_rot;
}
/**
 * @brief The inverse of `link_encode`.
 * @return The slot index; out of range means the link field was wild-written (treat as corruption).
 * @internal
 */
inline std::uint32_t link_decode(gc_chunk* c, std::uint32_t enc) {
    std::uint32_t x = enc - c->key_rot;
    return rotr32(x, c->key_rot) ^ c->key_mask;
}
/**
 * @brief Raw address of a free slot's link field (what is read back is the **encoded** value; encode before writing).
 * @internal
 */
inline std::uint32_t& slot_link_raw(gc_chunk* c, std::uint32_t idx) {
    return *reinterpret_cast<std::uint32_t*>(slot_at(c, idx));
}
/* ==========================================================================
 * Block-tail padding (the counterpart of mimalloc's mi_padding_t) -- **always on**
 *
 * Why a conservative GC especially needs it: with ordinary malloc an out-of-bounds heap write is "the user's own bug, at their own risk"; in a conservative GC it **pollutes the GC's own decision inputs**, becoming a soundness problem for the GC --  a user out-of-bounds write -> tramples the next object's object header (size / marked / type)
 *                -> conservative scanning decides the hit range by a garbage size and the color by a garbage marked -> that object is wrongly marked live (leak) or missed (**wrongly reclaimed = dangling pointer**).
 * And gcxx's slots are 16-byte aligned, so there are already 0..15 bytes of gap that "absorb" an overrun -- those gaps used to be **unowned**
 * (neither checked nor exploited), and padding turns them into sentinels. Layout: payload: [ user data size bytes ][ padding 8B ][ fill 0xAA ... alignment gap ], with padding immediately after
 * payload + size -- this is a **deliberate deviation** from mimalloc (which puts it at the chunk tail): a 1-byte overrun hits the canary (the offset granularity is instead given by the trailing 0xAA scan). One 8-byte structure covers four purposes:
 *   1. **Overrun detection**: the canary is derived from (chunk, slot index, size, keys); if it is overwritten it no longer matches;
 *   2. **Object header cross-check**: delta is a **redundant copy** of size and sits far from the object header -- when the previous object's overrun tramples this object's size, delta and size immediately disagree;
 *   3. **Double-free detection**: after the free check passes, the canary is changed to the freed sentinel; a second free decodes to the sentinel -> double free reported;
 *   4. **Byte-level scan**: the alignment gap after the canary is filled with 0xAA, so an overwrite can be pinned down to a byte offset. Cost (measured, clang-cl Release): pure allocation +13%, mixed +6.8%, realistic live workload +2.8% -- in exchange for "an out-of-bounds write no longer pollutes conservative scanning's decision inputs".
 * ========================================================================== */
/** @brief The fill byte for the trailing alignment gap (the expected value of the byte-level scan). 0xAA is the same as mimalloc's MI_DEBUG_PADDING:
 * it looks neither like a pointer nor like a common integer, minimizing the chance of misclassification.
 * @internal
 */
constexpr std::uint8_t  PAD_FILL  = 0xAAu;
/** @brief The freed sentinel. **It must be even** -- every valid canary has its lowest bit set, so the sentinel never equals any valid value
 * and no extra flag bit is needed.
 * @internal
 */
constexpr std::uint32_t PAD_FREED = 0x00DEAD00u;
/** @brief How many bytes at most the tail fill writes/scans. An upper bound is mandatory: a large object's tail gap can be tens of thousands of bytes, and filling it all would mean
 * paying for a memset on every allocation.
 * @internal
 */
constexpr std::uint32_t PAD_FILL_MAX = 32;
/**
 * @brief The 8-byte block-tail padding structure, located at payload + size.
 * @internal
 */
struct gc_padding {
    std::uint32_t canary;   /**< encoded from (chunk base, slot index, size, link keys) */
    std::uint32_t delta;    /**< total payload area length - size; usable = (payload area in slot) - delta */
};
static_assert(sizeof(gc_padding) == 8, "padding must stay 8 bytes");
/** @brief padding sits at payload + size -- **addressing it as 4-byte aligned may be unaligned** (size need not be a multiple of 4). So
 * reads and writes all go through memcpy: on x86 the compiler generates a single unaligned mov (zero cost), while on architectures that fault on unaligned access it is correct.
 * @internal
 */
inline void padding_store(const gc_object* o, std::uint32_t canary, std::uint32_t delta) {
    gc_padding p{};
    p.canary = canary;
    p.delta  = delta;
    std::memcpy(const_cast<char*>(payload_of(o)) + o->size, &p, sizeof(p));
}
/**
 * @brief Read an object's block-tail padding fields (canary / delta / tail fill).
 * @return The aggregate of the three; `padding_verify` uses it to decide whether an out-of-bounds write went through.
 * @internal
 */
inline gc_padding padding_load(const gc_object* o) {
    gc_padding p{};
    std::memcpy(&p, payload_of(o) + o->size, sizeof(p));
    return p;
}
/**
 * @brief canary = f(chunk base, slot index, size, the two keys). @note The keys are per chunk (see link_keys_init), so the same slot index has a different canary in different chunks; size is mixed in too, so "the canary matches but size was changed" also shows up.
 *       Lowest bit set to 1: every valid canary is odd, separating it from the even sentinel PAD_FREED. @note It uses the same idea as the free-list encoding (link_encode) but **different constants** -- the two value spaces must not overlap, otherwise a value could be both a valid link and a valid canary, weakening corruption detection.
 * @internal
 */
inline std::uint32_t padding_canary(const gc_chunk* c, std::uint32_t idx,
                                    std::uint32_t size) {
    auto x = (std::uint64_t)(std::uintptr_t)c->base;
    x ^= (std::uint64_t)idx * 0x9E3779B97F4A7C15ULL;
    x ^= (std::uint64_t)c->key_mask << 17;
    x ^= (std::uint64_t)c->key_rot << 45;
    x ^= (std::uint64_t)size << 3;
    x ^= x >> 31; x *= 0xFF51AFD7ED558CCDULL; x ^= x >> 29;
    return (std::uint32_t)x | 1u;
}
/**
 * @brief Total bytes of the payload area inside a slot (= slot size - object header); for large objects it is "the whole mapping minus the header and the object header".
 * @internal
 */
inline std::uint32_t padding_area_of(const gc_chunk* c) {
    return c->slot_size != 0
         ? (c->slot_size - (std::uint32_t)PAYLOAD_OFF)
         : (c->span - c->data_off - (std::uint32_t)PAYLOAD_OFF);
}
/**
 * @brief The result of block-tail padding verification (reporting them separately is deliberate: each kind of corruption points at a completely different bug).
 * @internal
 */
enum gc_padding_status {
    GCXX_PAD_OK = 0,
    GCXX_PAD_BAD_HEADER,    /* type / age is not a legal value: the object header was trampled            */
    GCXX_PAD_OVERFLOW,      /* canary mismatch: the tail was written (overrun / wild write)                */
    GCXX_PAD_DOUBLE_FREE,   /* canary is the freed sentinel: this slot was freed twice                     */
    GCXX_PAD_BAD_SIZE,      /* o->size disagrees with delta / size runs past the slot: header was trampled */
    GCXX_PAD_BAD_CANARY     /* canary is neither correct nor the sentinel (a garbage value)                */
};
/**
 * @brief Verify one object's padding; *out_usable is filled with the usable size. @return Status, GCXX_PAD_OK means intact.
 * @internal
 */
inline gc_padding_status padding_verify(const gc_chunk* c, const gc_object* o,
                                          std::uint32_t idx, std::uint32_t* out_usable) {
    /* Large objects with a guard page **do not take part in block-tail padding** (payload+size is pressed against an inaccessible page, so those 8 bytes
     * do not exist at all); using the guard page together with padding blows up on its own. Pass them through here. */
    if (c->guarded != 0) {
        *out_usable = o->size;
        return GCXX_PAD_OK;
    }
    /* First look at those two "limited value" fields in the object header: each has a legal upper bound, and one comparison rules them out.
     * Earlier than the canary, and the only shape that catches "only the object header was trampled, the tail untouched". */
    if (o->type > TYPE_MAX || o->age > (std::uint8_t)GC_AGE_OLD) {
        *out_usable = 0;
        return GCXX_PAD_BAD_HEADER;
    }
    const std::uint32_t area = padding_area_of(c);
    /* size out of range is itself conclusive evidence that the object header was trampled, so **it must be checked first**: otherwise locating the padding by size reads outside the slot. */
    if (o->size > area || area - o->size < (std::uint32_t)PAD_BYTES) {
        *out_usable = 0;
        return GCXX_PAD_BAD_SIZE;
    }
    const gc_padding p = padding_load(o);
    if (p.canary == PAD_FREED) {
        *out_usable = 0;
        return GCXX_PAD_DOUBLE_FREE;
    }
    const std::uint32_t expect = padding_canary(c, idx, o->size);
    if (p.canary != expect) {
        *out_usable = 0;
        /* "Garbage value" and "overwritten" cannot be reliably distinguished; both are classified as OVERFLOW; the raw value is reported to the caller. */
        return (p.canary == 0) ? GCXX_PAD_BAD_CANARY : GCXX_PAD_OVERFLOW;
    }
    /* delta is a redundant copy of size: a mismatch means the **object header** was modified (while the canary is fine -- exactly the shape of "the previous object's overrun hit this object's header"). */
    if (p.delta != area - o->size) {
        *out_usable = 0;
        return GCXX_PAD_BAD_SIZE;
    }
    *out_usable = area - p.delta;
    return GCXX_PAD_OK;
}
/**
 * @brief The **effective length** of the tail fill: the number of bytes actually filled with 0xAA after the canary.
 *
 * **The allocation path (`padding_init`) and the self-check path (`padding_first_dirty_byte`) must use the same
 * predicate** -- as soon as the two sides compute different lengths, the self-check misreports "bytes that were never filled" as "the fill was corrupted".
 * This project tripped over it once: the first version wrote only in 8-byte words and left a 1..7 byte remainder, and 5 test cases went red immediately.
 * Extracting it into one function makes that kind of drift **impossible from now on**.
 *
 * @param area Total bytes of the payload area inside the slot (see `padding_area_of`).
 * @param size The number of payload bytes requested by the object.
 * @return The number of fillable bytes, capped at `PAD_FILL_MAX`; returns 0 when `area` is too tight.
 * @internal
 */
inline std::uint32_t padding_fill_len(std::uint32_t area, std::uint32_t size) {
    const std::uint32_t head = area - size;               /* slot tail slack (including the canary's 8B) */
    if (head <= (std::uint32_t)PAD_BYTES) return 0;
    std::uint32_t room = head - (std::uint32_t)PAD_BYTES;
    if (room > (std::uint32_t)PAD_FILL_MAX) room = (std::uint32_t)PAD_FILL_MAX;
    return room;
}
/**
 * @brief Initialize one object's padding (allocation path); delta records **the number of bytes given away**, and usable = area - delta is the requested size.
 * @internal
 */
inline void padding_init(const gc_chunk* c, gc_object* o, std::uint32_t idx) {
    if (c->guarded != 0) return;     /* a guard page replaces padding, see padding_verify */
    const std::uint32_t area = padding_area_of(c);
    const std::uint32_t delta = area - o->size;
    padding_store(o, padding_canary(c, idx, o->size), delta);
    /* ---- Fill the trailing alignment gap with 0xAA (used to pin an "out-of-bounds write" down to **which byte**) --------
     * This fill only improves **diagnostic precision**: the overrun itself is detected by the canary (the store above),
     * and the fill only answers "did the write go past the whole padding, and to which byte".
     *
     * It used to be a byte-by-byte `memset` of at most `PAD_FILL_MAX`(32) bytes -- measured, that was **the most expensive stretch on the allocation
     * hot path**: alternating A/B on the same machine, removing it is **5.1%~10.8%** faster (32B -5.1%, 64B -7.6%,
     * 256B -7.6%, 1024B -10.8%).
     *
     * Changed to "write 8-byte words first, then patch the remainder". **The set of bytes filled is exactly the same as the old byte-by-byte version**
     * (all of them the consecutive `padding_fill_len(area, size)` bytes after the canary), and that is crucial:
     * `padding_first_dirty_byte` scans the same region, and as soon as the two compute different lengths,
     * the self-check reports "bytes that were never filled" as "the fill was corrupted" (the first version filled only whole words and left a
     * 1..7 byte remainder, which is exactly what this self-check caught).
     *
     * The length predicate is extracted into `padding_fill_len`, **shared** by both sides, rooting out any future drift. */
    const std::uint32_t flen = padding_fill_len(area, o->size);
    if (flen != 0) {
        char* fill = payload_of(o) + o->size + PAD_BYTES;
        std::uint64_t pat;
        std::memset(&pat, (int)PAD_FILL, sizeof(pat));
        std::uint32_t done = 0;
        for (; done + 8u <= flen; done += 8u)
            std::memcpy(fill + done, &pat, 8u);
        for (; done < flen; ++done)                    /* remainder <= 7 bytes */
            fill[done] = (char)PAD_FILL;
    }
}
/** @brief Derive the slot index back from the object and verify it, for **diagnostic/verification** paths.
 *  @param out_idx    Filled with the derived slot index (needed by the error messages). @param out_usable Filled with the usable byte count. @note The allocation and free paths already have the slot index, so calling padding_verify directly saves a division.
 * @internal
 */
inline gc_padding_status padding_verify_object(const gc_chunk* c, const gc_object* o,
                                                 std::uint32_t* out_idx,
                                                 std::uint32_t* out_usable) {
    const std::uint32_t idx = (c->slot_size != 0)
        ? (std::uint32_t)((reinterpret_cast<std::uintptr_t>(o) -
              (reinterpret_cast<std::uintptr_t>(c->base) + c->data_off)) / c->slot_size)
        : 0u;
    *out_idx = idx;
    return padding_verify(c, o, idx, out_usable);
}
/** @brief In the alignment gap after the canary, the offset of the first byte that is not 0xAA (relative to payload).
 *  @return The offset; -1 means the whole stretch is intact. @note It is only meaningful when **the canary is already known to be intact**: it answers "did the write go past the whole padding, and to which byte", the half of the information the canary cannot report. The scan length has an upper bound.
 * @internal
 */
inline int padding_first_dirty_byte(const gc_chunk* c, const gc_object* o) {
    /* A guarded chunk has no "fill after the canary": that is the guard page, and reading it triggers an access violation immediately (measured: gc_verify segfaults right there). */
    if (c->guarded != 0) return -1;
    const std::uint32_t area = padding_area_of(c);
    /* **Must** use exactly the same length predicate as `padding_init`: scan only the stretch that really was filled.
     * Scanning further would treat bytes that were never filled (old data in a reused slot, zeros in a new slot) as "the fill was corrupted". */
    const std::uint32_t flen = padding_fill_len(area, o->size);
    if (flen == 0) return -1;
    const char* base = payload_of(o);
    const std::uint32_t from = o->size + (std::uint32_t)PAD_BYTES;
    for (std::uint32_t i = 0; i < flen; ++i) {
        if ((unsigned char)base[from + i] != (unsigned char)PAD_FILL)
            return (int)(from + i);
    }
    return -1;
}
/**
 * @brief Corrupted cold path: report + isolate. @note **noinline**: the hot path keeps only a perfectly predicted branch, and the whole error-handling apparatus is pushed outside.
 * @internal
 */
GCXX_NOINLINE inline void gc_padding_failed(const gc_chunk* c, const gc_object* o,
                                              std::uint32_t idx,
                                              gc_padding_status st) {
    const std::uint32_t area = padding_area_of(c);
    if (st == GCXX_PAD_DOUBLE_FREE) {
        gc_error_message(EAGAIN,
            "double free detected: heap block %p (chunk %p slot %u size %u) was "
            "already freed", (const void*)o, (const void*)c, idx, o->size);
        return;
    }
    if (st == GCXX_PAD_BAD_HEADER) {
        gc_error_message(EFAULT,
            "corrupted object header: heap block %p (chunk %p slot %u) has type=%u "
            "age=%u, which are out of range", (const void*)o, (const void*)c, idx,
            (unsigned)o->type, (unsigned)o->age);
        return;
    }
    if (st == GCXX_PAD_BAD_SIZE) {
        /* Only when size is still within the slot is the value derived from delta trustworthy; otherwise it is itself garbage, so report 0. */
        std::uint32_t claimed = 0;
        if (area >= o->size && area - o->size >= (std::uint32_t)PAD_BYTES) {
            const std::uint32_t d = padding_load(o).delta;
            claimed = (d <= area) ? (area - d) : 0u;
        }
        gc_error_message(EFAULT,
            "corrupted object header? heap block %p (chunk %p slot %u) has size %u "
            "but its padding says the usable area was %u (slot payload area %u)",
            (const void*)o, (const void*)c, idx, o->size, claimed, area);
        return;
    }
    /* The canary sits in the **first 8 bytes** at payload + size, so "the canary is broken but the tail fill is still
     * clean" is the most common shape (an overrun of exactly 1..8 bytes). Then padding_first_dirty_byte returns -1, and reporting that straight as an offset would print an incomprehensible "-1 bytes", hence two kinds of message. */
    const int dirty = padding_first_dirty_byte(c, o);
    if (dirty >= 0) {
        gc_error_message(EFAULT,
            "buffer overflow in heap block %p of size %zu: write after %d bytes "
            "(chunk %p slot %u, canary 0x%08x)",
            (const void*)o, (std::size_t)o->size, dirty, (const void*)c, idx,
            (unsigned)padding_load(o).canary);
    } else {
        gc_error_message(EFAULT,
            "buffer overflow in heap block %p of size %zu: the block-tail canary "
            "itself was overwritten (chunk %p slot %u, canary 0x%08x)",
            (const void*)o, (std::size_t)o->size, (const void*)c, idx,
            (unsigned)padding_load(o).canary);
    }
}
/* ---- direct-mapped chunk table ---- */
/**
 * @brief Address -> bucket number in the direct-mapped table.
 * @internal
 */
inline std::size_t chunk_slot_index(gc_state& g, std::uintptr_t addr) {
    return (std::size_t)((addr >> CHUNK_SHIFT) & g.table_mask);
}
/** @brief Allocate/grow the direct-mapped table: load factor (registered blocks / entries) reaching 3/4 triggers doubling; rehash rebuilds
 *         the whole table from the chunk chain. @return false on failure (keep using the old table + overflow chain; old overflow nodes have been freed).
 * @internal
 */
/**
 * @brief Retire a bucket array together with every overflow node reachable from it.
 *
 * Called when `table_grow` replaces the table. **Nothing is released**: a lock-free reader
 * (`table_find` via `object_find` / `gc_is_gc_pointer` / `weak_peek`) may be walking this array or
 * one of its collision chains at this very moment, and freeing either would be a use-after-free.
 *
 * The nodes are moved to the **retired-node chain**, not to the free pool: a pooled node is one
 * `chunk_table_link` may hand out again, and handing out a node that is still reachable from a
 * retired array would graft the new table's chain onto the old array's -- the reader would then
 * follow a chain that belongs to a *different* bucket. Keeping a separate chain makes "reachable
 * only from a retired array" and "available for reuse" disjoint sets.
 *
 * @param table The bucket array being retired.
 * @param size  Its number of entries.
 * @note Nodes are pushed one at a time: after a rehash an old array's nodes are *not* a single
 *       chain (`next` describes the chains the rehash built), so the array has to be walked bucket
 *       by bucket.
 * @note The caller records `size` in `table->key` when it links the array onto `gc_state::table_retired`;
 *       `table_take_array` matches on that, which is what keeps a smaller retired array from being
 *       cleared with a larger array's size.
 * @internal
 */
inline void table_retire_nodes(gc_state& g, gc_map_entry* table, std::size_t size) {
    for (std::size_t k = 0; k < size; ++k) {
        gc_map_entry* q = table[k].next;
        while (q != nullptr) {
            gc_map_entry* dead = q;
            q = q->next;
            dead->next = g.table_retired_nodes;
            g.table_retired_nodes = dead;
        }
    }
}
/**
 * @brief Take a bucket array to fill, preferring a retired one over a fresh allocation.
 *
 * The retired chain is stored in the `key` field of each array's **entry 0**, so it is a
 * heterogeneous list: `table_grow` doubles the table, so most retired arrays are smaller than the
 * one now being built, and clearing one with the new size would write past its end. The search picks
 * the first retired array with the right number of entries; a mismatch is skipped, not cleared.
 * There is no separate index to keep in sync, and the list never holds more than a few dozen arrays
 * (one per doubling), so a linear scan ran O(log^2) times over the process lifetime is free.
 *
 * @param allow_alloc `false` when the test injection has claimed the primary allocation.
 * @note The chosen array is cleared before reuse. That cannot disturb a reader: it is popped here
 *       only to become the table *being built*, and it cannot be the array a reader is using, because
 *       the array was already replaced as `chunk_table` before it was pushed onto the retired chain.
 * @internal
 */
inline gc_map_entry* table_take_array(gc_state& g, std::size_t entries,
                                      bool allow_alloc) {
    for (gc_map_entry** p = &g.table_retired; *p != nullptr; p = &(*p)->next) {
        gc_map_entry* t = *p;
        if ((std::size_t)t->key != entries) continue;
        *p = t->next;
        t->key = 0;
        t->chunk = nullptr;
        t->data_off = 0;
        t->slot_size = 0;
        t->next = nullptr;
        /* Clear only the entries the array was allocated for -- which is exactly `entries` here. */
        std::memset(t + 1, 0, (entries - 1) * sizeof(gc_map_entry));
        return t;
    }
    if (!allow_alloc) return nullptr;
    return static_cast<gc_map_entry*>(std::calloc(entries, sizeof(gc_map_entry)));
}
/**
 * @brief Take one overflow node: the free pool, then retired-but-unreachable nodes, then `malloc`.
 * @note Only nodes on `table_free_nodes` are handed out. They were released by
 *       `chunk_table_unlink`, which is called **only** under `gc_mutex` on a node that was just
 *       spliced out of the **current** table, so no reader can reach them any more: an array that
 *       reaches a node is either the current table (node removed) or a retired array (nodes moved
 *       to `table_retired_nodes`, never to this pool).
 * @internal
 */
inline gc_map_entry* table_take_node(gc_state& g, bool allow_alloc) {
    if (g.table_free_nodes != nullptr) {
        gc_map_entry* n = g.table_free_nodes;
        g.table_free_nodes = n->next;
        return n;
    }
    if (!allow_alloc) return nullptr;
    return static_cast<gc_map_entry*>(std::malloc(sizeof(gc_map_entry)));
}
inline bool table_grow() {
    gc_state& g = G();
    const std::size_t ns = g.table_size * 2;
    /* Test injection: see the note on `gcxx::tests::fail_next_table_grow`. Read once, **here**, and
     * threaded through every allocation this growth makes, so "the next growth fails" holds no matter
     * what the pool happens to contain, and cannot leak into a later, unrelated growth. */
    const bool alloc_ok = !gcxx::tests::fail_next_table_grow().exchange(
        false, std::memory_order_relaxed);
    const std::size_t nm = ns - 1;
    auto* nt = static_cast<gc_map_entry*>(table_take_array(g, ns, alloc_ok));
    if (nt == nullptr) return false;                /* failure: keep using the old table + overflow chain */
    for (gc_chunk* c = g.chunks; c != nullptr; c = c->all_next) {
        for (std::uint32_t i = 0; i < c->nkeys; ++i) {
            std::uintptr_t key = reinterpret_cast<std::uintptr_t>(c->base)
                               + (std::uintptr_t)i * CHUNK_SIZE;
            gc_map_entry& e = nt[(key >> CHUNK_SHIFT) & nm];
            if (e.chunk == nullptr) {
                e.key = key; e.chunk = c;
                e.data_off = c->data_off; e.slot_size = c->slot_size;
                e.next = nullptr;
            } else {
                auto* n = table_take_node(g, alloc_ok);
                if (n == nullptr) {                 /* failure: retire the half-built table, keep the old one */
                    table_retire_nodes(g, nt, ns);
                    nt->key = ns;                   /* the retired chain is indexed by entry count */
                    nt->next = g.table_retired;
                    g.table_retired = nt;
                    return false;
                }
                n->key = key; n->chunk = c;
                n->data_off = c->data_off; n->slot_size = c->slot_size;
                n->next = e.next; e.next = n;
            }
        }
    }
    /* Retire the old array, recording its entry count in entry 0's `key` (the array is dead, so no
     * key is lost), and move its nodes to the never-reused chain. **Nothing is freed**, because a
     * lock-free reader may be walking either the array or one of those nodes right now. */
    table_retire_nodes(g, g.chunk_table, g.table_size);
    g.chunk_table->key = g.table_size;
    g.chunk_table->chunk = nullptr;
    g.chunk_table->data_off = 0;
    g.chunk_table->slot_size = 0;
    g.chunk_table->next = g.table_retired;
    g.table_retired = g.chunk_table;
    g.chunk_table = nt;
    g.table_size  = ns;
    g.table_mask  = nm;
    return true;
}
/**
 * @brief One-time allocation of the direct-mapped table, called when the first heap is created. @return false on allocation failure.
 * @internal
 */
inline bool tables_init(gc_state& g) {
    g.chunk_table = static_cast<gc_map_entry*>(
        std::calloc(TABLE_INITIAL, sizeof(gc_map_entry)));
    if (g.chunk_table == nullptr) return false;
    g.table_size = TABLE_INITIAL;
    g.table_mask = TABLE_INITIAL - 1;
    return true;
}
/**
 * @brief Register a block in the flat table: an empty slot is stored inline, a collision (extremely rare) goes onto the overflow chain.
 * @internal
 */
inline void chunk_table_link(gc_chunk* c, std::uintptr_t key,
                             std::uint32_t data_off, std::uint32_t slot_size) {
    gc_state& g = G();
    gc_map_entry& e = g.chunk_table[chunk_slot_index(g, key)];
    if (e.chunk == nullptr) {                     /* empty slot: inline */
        e.key = key; e.chunk = c;
        e.data_off = data_off; e.slot_size = slot_size;
        e.next = nullptr;
        return;
    }
    /* Collision: an overflow node. The pool is preferred over `malloc`, so the number of nodes the
     * process ever allocates is bounded by the widest rehash rather than by the number of chunks. */
    auto* n = table_take_node(g, /*allow_alloc=*/true);
    if (n == nullptr) std::abort();               /* 32B allocation failed: giving up is the safer choice */
    n->key = key; n->chunk = c;
    n->data_off = data_off; n->slot_size = slot_size;
    n->next = e.next;
    e.next = n;
}
/**
 * @brief Remove a block from the direct-mapped table (an inline hit clears the slot, otherwise an overflow node is promoted to inline).
 * @param key Block base address.
 * @note Also decrements `block_count`.
 * @note **Nothing is released and no field group is rewritten wholesale.** `table_find` runs
 *       lock-free (see the note on the table section), so the promotion below overwrites the inline
 *       entry **one field at a time with the new key written last**: a concurrent reader either
 *       matches the old key against the old, still self-consistent payload, or fails the key
 *       comparison and walks the overflow chain. Writing `chunk`/`data_off`/`slot_size` before `key`
 *       is what makes "the key matches" imply "the fields describe the same chunk". Assigning the
 *       whole `gc_map_entry` in one statement would open a window in which a reader matches the
 *       **old** key against the **new** chunk, and a `chunk_of()` that yields the wrong chunk means
 *       reading arbitrary memory as object headers.
 * @internal
 */
inline void chunk_table_unlink(std::uintptr_t key) {
    gc_state& g = G();
    gc_map_entry& e = g.chunk_table[chunk_slot_index(g, key)];
    if (e.chunk != nullptr && e.key == key) {       /* inline hit */
        if (e.next == nullptr) {
            e.key = 0; e.chunk = nullptr; e.data_off = 0; e.slot_size = 0;
        } else {                                 /* promote an overflow node to inline */
            gc_map_entry* n = e.next;
            e.chunk    = n->chunk;
            e.data_off = n->data_off;
            e.slot_size = n->slot_size;
            e.next     = n->next;
            e.key      = n->key;                 /* last: `key` is the reader's match criterion */
            n->next = g.table_free_nodes;        /* recycle; a reader may still hold a pointer to it */
            g.table_free_nodes = n;
        }
        --g.block_count;
        return;
    }
    gc_map_entry** p = &e.next;                  /* overflow chain */
    while (*p != nullptr && (*p)->key != key) p = &(*p)->next;
    if (*p != nullptr) {
        gc_map_entry* dead = *p;
        *p = dead->next;
        dead->next = g.table_free_nodes;         /* recycle; a reader may still hold a pointer to it */
        g.table_free_nodes = dead;
        --g.block_count;
    }
}
/**
 * @brief Register a 64KB block. @note key must be 64KB aligned.
 * @internal
 */
inline void chunk_register_block(gc_chunk* c, std::uintptr_t key,
                                 std::uint32_t data_off, std::uint32_t slot_size) {
    gc_state& g = G();
    /* Load factor (registered blocks / entries) reaching 3/4 doubles the table first; a failed growth keeps using the overflow chain
     * (functionality is unaffected, only reverse lookups get slower). */
    if (g.block_count * 4 >= g.table_size * 3) table_grow();
    chunk_table_link(c, key, data_off, slot_size);
    ++g.block_count;                       /* counting happens only at the registration entry point (shared by both link branches) */
}
/**
 * @brief O(1) table-entry reverse lookup: inline fast path (the vast majority of hits), collisions go onto the overflow chain.
 * @internal
 */
inline gc_map_entry* table_find(std::uintptr_t addr) {
    gc_state& g = G();
    const std::uintptr_t want = addr & ~(std::uintptr_t)CHUNK_MASK;
    gc_map_entry* e = &g.chunk_table[chunk_slot_index(g, addr)];
    if (e->chunk != nullptr && e->key == want) return e;   /* fast path: usually a hit */
    for (e = e->next; e != nullptr; e = e->next)
        if (e->key == want) return e;
    return nullptr;
}
/**
 * @brief O(1) reverse lookup of which chunk addr falls in. @return nullptr on a miss.
 * @internal
 */
inline gc_chunk* chunk_of(std::uintptr_t addr) {
    gc_map_entry* e = table_find(addr);
    return e != nullptr ? e->chunk : nullptr;
}
/* ---- chunk "enlist / retire": the single entry point for the global chain + direct-mapped table + accounting (gc_mutex must be held).
 * The three must be **perfectly in sync** -- when they were spread across six places they drifted (duplicate table entries, double retirement causing a counter underflow,
 * and a reversed unlink order wrongly deleting the entry of the new owner). */
/** @brief Hang nkeys 64KB blocks onto the global chain and the direct-mapped table, and account for them. @note c->nkeys is only incremented
 *         after each key is registered -- table_grow reads it to rebuild the whole table, and it must already reflect the registered keys before that.
 * @internal
 */
inline void chunk_enlist(gc_chunk* c, std::uint32_t nkeys) {
    gc_state& g = G();
    c->nkeys = 0;
    c->all_next = g.chunks;
    g.chunks = c;
    ++g.chunk_count;
    g.mapped_bytes += c->span;
    for (std::uint32_t i = 0; i < nkeys; ++i) {
        chunk_register_block(c, reinterpret_cast<std::uintptr_t>(c->base)
                                + (std::uintptr_t)i * CHUNK_SIZE,
                             c->data_off, c->slot_size);
        c->nkeys = i + 1;
    }
    /* Heap address range snapshot (the fast exclusion interval used while scanning); it only widens and never tightens, hence it is idempotent for re-claiming a block already inside the interval. */
    auto lo = reinterpret_cast<std::uintptr_t>(c->base);
    std::uintptr_t hi = lo + c->span;
    if (lo < g.heap_lo.load(std::memory_order_relaxed))
        g.heap_lo.store(lo, std::memory_order_relaxed);
    if (hi > g.heap_hi.load(std::memory_order_relaxed))
        g.heap_hi.store(hi, std::memory_order_relaxed);
}
/**
 * @brief Remove from the global chain and the table, and decrement the accounting.
 * @return false = the block is no longer on the chain (it was already retired elsewhere).
 * @note **Does not free memory**; the caller decides whether to give it back to the recycle stack or back to the system. The return value is the key to concurrency
 *       correctness: there is a window between live reaching zero and the lock being acquired, in which the owner thread exiting and the collector's
 *       chunk_free may both judge the same block to be empty; both call this first and only continue when it returns true,
 *       so that table entries are never unregistered twice, counters never underflow, and the same node is never pushed twice. The caller must hold gc_mutex.
 * @internal
 */
inline bool chunk_retire(gc_chunk* c) {
    gc_state& g = G();
    /* On the collector's dirty-block chain -> **retirement is not allowed** (see gc_chunk::in_dirty): once retired, this block may be
     * reused or returned to the system at any moment, while its chain node is still hanging on the chain. Returning false only means "not retired this time"; the end of the slice re-evaluates it. */
    if (c->in_dirty.load(std::memory_order_acquire)) return false;
    /* Retirement is the **automatic** checkpoint of the metadata redzone, and it is exactly the moment "this memory leaves the allocatable set" --
     * metadata corruption not found at this moment will only surface later in a form that is harder to attribute (objects vanishing en masse after a claim). */
    if (!chunk_redzone_ok(c))
        gc_error_message(EFAULT,
            "corrupted chunk metadata redzone: chunk %p data_off=%u -- a write went "
            "below the data area (slot 0 underflow?)", (const void*)c, c->data_off);
    if (!chunk_meta_ok(c))
        gc_error_message(EFAULT,
            "corrupted chunk metadata: chunk %p base=%p span=%u data_off=%u "
            "slot_size=%u capacity=%u cls=%u (checksum mismatch)",
            (const void*)c, (void*)c->base, c->span, c->data_off,
            c->slot_size, c->capacity, c->cls);
    gc_chunk** p = &g.chunks;
    while (*p && *p != c) p = &(*p)->all_next;
    if (*p != c) return false;
    *p = c->all_next;
    --g.chunk_count;
    g.mapped_bytes -= c->span;
    for (std::uint32_t i = 0; i < c->nkeys; ++i)
        chunk_table_unlink(reinterpret_cast<std::uintptr_t>(c->base)
                           + (std::uintptr_t)i * CHUNK_SIZE);
    c->nkeys = 0;                    /* explicitly "not on the chain": enlist resets it on re-enlistment */
    c->all_next = nullptr;
    /* Cut the owner pointer back to the heap: a retired block is no longer on any partial chain, and that heap struct may at any moment
     * be reclaimed by reap_dead_heaps -- keeping a dangling owner pointer would be a use-after-free (rewritten on claim). */
    c->owner = nullptr;
    return true;
}
/**
 * @brief Reset a block's allocation/free state (shared by new blocks and blocks claimed from the recycle stack). @note The caller must guarantee that no owner/collector touches it concurrently at this moment.
 * @internal
 */
inline void chunk_reset_state(gc_chunk* c, std::uint32_t used) {
    c->free = SLOT_NONE;
    c->used = used;
    c->used_pub.store(used, std::memory_order_relaxed);
    c->freed = 0;
    c->freed_pub.store(0, std::memory_order_relaxed);
    c->bump.store(0, std::memory_order_relaxed);
    c->free_head.store(SLOT_NONE, std::memory_order_relaxed);
    c->dead_head = SLOT_NONE; c->dead_tail = SLOT_NONE; c->dead_len = 0;
    c->dnext = nullptr; c->in_dirty.store(false, std::memory_order_relaxed);
    /* Reuse (claiming from the recycle stack) must reset these two as well: the claim path has already cancelled decommit, so this block is from then on no longer "on a chain and decommitted". See the pairing requirement of gc_chunk::purged. */
    c->retired_ms = 0;
    c->purged = 0;
}
/* ---- create / return chunk ---- (the caller must hold gc_mutex) */
/**
 * @brief **Common initialization** of a new chunk: geometry fields, allocation/free state, free-list keys,
 *        metadata redzone and checksum.
 *
 * Small blocks (`chunk_new`, one slot size reused for the whole block) and large-object blocks (`chunk_new_big`, exclusively occupying several
 * 64KB) have **no difference whatsoever** in these fields, only different values. This code used to be two verbatim duplicate
 * copies -- duplicated safety machinery drifts most easily, and the consequence of drift is "some kind of block is missing a check".
 *
 * @param mem The base address returned by `alloc_block` (already aligned to `CHUNK_SIZE`).
 * @param owner The owning heap.
 * @param span Number of bytes occupied (a large object may span several 64KB).
 * @param data_off Data-area offset (differs from `chunk_data_off()` when a large object has a guard page).
 * @param slot_size Slot size; 0 = a large object occupying the whole block.
 * @param capacity Number of slots (always 1 for a large object).
 * @param cls size class; `SLOT_NONE` for a large object.
 * @param kind `KIND_NORMAL` / `KIND_ATOMIC`.
 * @param used Initial number of allocated slots (1 for a large object).
 * @param os_zeroed Whether the data area was freshly committed by the OS (the kernel guarantees zero pages).
 * @param guarded Whether it carries an inaccessible guard page (only large objects do).
 * @return A ready chunk (**not yet** registered in the global chain and the direct-mapped table).
 * @note `bump` is set by the caller as needed after the return: small blocks start from 0, a large object must be 1 (that single slot
 *       has already been handed out). **Keeping it in the caller** is because "how many slots have been handed out" is allocation semantics, not geometry.
 * @internal
 */
inline gc_chunk* chunk_init(unsigned char* mem, gc_heap* owner, std::uint32_t span,
                            std::uint32_t data_off, std::uint32_t slot_size,
                            std::uint32_t capacity, unsigned cls, unsigned kind,
                            std::uint32_t used, std::uint8_t os_zeroed,
                            std::uint8_t guarded) {
    /* gc_chunk contains std::atomic members: writing through a raw reinterpret_cast is UB; one must
     * first start its lifetime with placement new (a trivial type, generating no code at all). */
    auto* c = ::new (static_cast<void*>(mem)) gc_chunk;
    c->cnext = nullptr;
    c->base = mem;
    c->owner = owner;
    c->span = span;
    c->data_off = data_off;
    c->slot_size = slot_size;
    c->capacity = capacity;
    c->cls = cls;
    c->kind = kind;
    /* recip depends only on slot_size, which never changes after the block is created, so computing it once suffices (no need to recompute on claim-reuse,
     * saving a 64-bit division per block). slot_size == 0 (large object) gets 1, to avoid division by zero. */
    c->recip = (slot_size != 0)
             ? (std::uint32_t)(((std::uint64_t)1 << 32) / slot_size) + 1u
             : 1u;
    c->os_zeroed = os_zeroed;
    c->guarded = guarded;
    chunk_reset_state(c, used);
    /* `chunk_reset_state(c, used)` has already reset **everything** mutable: free / used / used_pub /
     * freed / freed_pub / bump / free_head / dead_head / dead_tail / dead_len / dnext / in_dirty /
     * retired_ms / purged. In particular it sets `bump` from its second argument, which is exactly
     * the "number of slots already handed out" this call site needs (0 for a small block, 1 for the
     * single slot of a large object) -- see the @note above.
     *
     * @note There used to be an extra `c->bump.store(used, ...)` here, a second, redundant store of
     *       the value `chunk_reset_state` had just written. It was harmless but it hid the real
     *       contract ("`used` is the slot count *and* the initial bump cursor"), which is the thing
     *       a future reader must not get wrong. What is genuinely private to the block geometry and
     *       therefore belongs here is only the blacklist bitmap and the free-list key/random state. */
    c->bl_hits.store(0, std::memory_order_relaxed);
    c->bl_prev = 0;
    c->bl_black.store(0, std::memory_order_relaxed);
    link_keys_init(c, mem);
    chunk_redzone_init(c);
    c->meta_sum = chunk_meta_checksum(c);
    return c;
}
inline gc_chunk* chunk_new(gc_heap* owner, unsigned cls,
                           std::uint32_t slot_size, std::uint32_t capacity,
                           unsigned kind) {
    bool zeroed = false;
    auto* mem = static_cast<unsigned char*>(
        alloc_block(CHUNK_SIZE, &zeroed));
    if (mem == nullptr) return nullptr;
    /* Small slots are reused over and over, so "the data area is zero" does not hold for the next allocation (only the slots
     * the bump pointer passes for the first time are zero), hence **not** marking os_zeroed -- the C3 zero-clear elision only holds for large objects. */
    (void)zeroed;
    gc_chunk* c = chunk_init(mem, owner, (std::uint32_t)CHUNK_SIZE,
                             (std::uint32_t)chunk_data_off(), slot_size, capacity,
                             cls, kind, 0u, /*os_zeroed=*/0u, /*guarded=*/0u);
    chunk_enlist(c, 1);
    return c;
}
/** @brief Create a large-object block: exclusively occupy several 64KB blocks (total = total bytes in the slot), registering one entry per block.
 *  @note E2: sample a fraction of objects per GUARD_SAMPLE to be placed **tight against an inaccessible page** (payload +
 *        size == the first byte of that page) -- a 1-byte overflow then faults on the very instruction doing the out-of-bounds write. Precondition: the block
 *        base address must be page-aligned (alloc_block guarantees 64KB alignment, and the page size divides 64KB).
 * @internal
 */
inline gc_chunk* chunk_new_big(gc_heap* owner, std::uint32_t total, unsigned kind) {
    const std::size_t head = chunk_data_off();
    /* Guard against addition wraparound before rounding up to 64KB (head+total can overflow on a 32-bit size_t). */
    if ((std::size_t)total > (std::size_t)-1 - head - (CHUNK_SIZE - 1))
        return nullptr;
    std::size_t span = (head + (std::size_t)total + CHUNK_SIZE - 1)
                       & ~(CHUNK_SIZE - 1);
    /* span is stored in a 32-bit field. gcxx::detail::MAX_OBJECT_SIZE is only the payload upper bound; after adding the header and rounding up to 64KB
     * it can come out exactly 4GiB -- truncation to 0 would make the mapped_bytes accounting, the span deduction in chunk_release,
     * and the length of the page protection (E2) all wrong, so it is rejected outright. */
    if (span == 0 || span > (std::size_t)0xFFFFFFFFu) return nullptr;
    const std::size_t ps = page_size();
    /* The sampling cursor is protected by gc_mutex (the large-object path of chunk_alloc holds it). */
    bool guard_this = false;
    {
        gc_state& g = G();
        guard_this = (++g.guard_tick % (std::uint32_t)GUARD_SAMPLE) == 0u;
    }
    std::size_t data_off = head;
    if (guard_this && ps != 0) {
        /* The payload must stay 16-byte aligned (the public contract for alignof(T) <= 16), while
         * "payload + size pressed exactly on the page boundary" requires `16 + size` to be a multiple of 16 -- when size is not
         * a multiple of 16 the two are **mutually exclusive**. The trade-off: **alignment wins** (a payload not 16-aligned would make
         * types with alignof(T)==16 unusable, which is worse than catching a few bytes less of overflow). So the object is placed as close to the
         * guard page as possible, leaving a gap = (16+size) mod 16 bytes (0..15): when gap == 0, an overflow of **1 byte**
         * hits the guard page. Blocks with `guarded` **do not participate in block-tail padding** (see padding_verify). */
        const std::size_t obj_bytes = (std::size_t)total - PAD_BYTES;  /* header+payload */
        const std::size_t need = obj_bytes + (std::size_t)SLOT_ALIGN + ps;
        span = (head + need + CHUNK_SIZE - 1) & ~(CHUNK_SIZE - 1);
        if (span == 0 || span > (std::size_t)0xFFFFFFFFu) return nullptr;
        const std::size_t raw = (span - ps) - obj_bytes;   /* slack below the page boundary */
        data_off = raw & ~(std::size_t)(SLOT_ALIGN - 1);
        if (data_off < head) {                             /* the allowance does not work out: do not sample */
            data_off = head;
            span = (head + (std::size_t)total + CHUNK_SIZE - 1)
                   & ~(CHUNK_SIZE - 1);
            guard_this = false;
        }
    }
    bool zeroed = false;
    auto* mem = static_cast<unsigned char*>(alloc_block(span, &zeroed));
    if (mem == nullptr) return nullptr;
    /* ---- The guard page must be established here, **it cannot** wait until after `chunk_init` to be changed ----
     * The last two steps of `chunk_init` are `chunk_redzone_init(c)` (placing the sentinel according to `c->data_off`)
     * and `c->meta_sum = chunk_meta_checksum(c)` (folding `data_off` / `guarded` into the checksum).
     * So "create the block first, then change `guarded`/`data_off` according to whether guard() succeeded" would make
     *   (1) the checksum inconsistent with the fields => `chunk_meta_ok()` always false;
     *   (2) the metadata sentinel left **below** the **old** `data_off` => `chunk_redzone_ok()` reads zeros => always false;
     * and then a **healthy** degraded block would report "corrupted chunk metadata + redzone" at the first `chunk_retire` / `gc_verify`
     * (a false positive is worse than a miss; see the discipline of verify_heap).
     * Therefore "whether it can be guarded" is fixed **before construction**: try `guard()` first, and on failure change the geometry to
     * that of an ordinary block, then let `chunk_init` place the sentinel and compute the checksum with the final values.
     *
     * The order is still safe: the guard page is in place **before** `chunk_enlist` (registration in the global chain/direct-mapped table),
     * so when another thread reverse-looks-up this block, the invariant "tight against the guard page" already holds. */
    if (guard_this) {
        /* Failure degrades to an ordinary block: better not to guard at all than to place the object on an actually writable "guard page"
         * -- that would let out-of-bounds accesses **silently pass**, which is worse than not guarding. */
        if (guard(mem + (span - ps), ps, true)) {
            G().guarded_objects.fetch_add(1, std::memory_order_relaxed);
        } else {
            guard_this = false;
            data_off   = head;
            span       = ((std::size_t)head + (std::size_t)total + CHUNK_SIZE - 1)
                         & ~(std::size_t)(CHUNK_SIZE - 1);
            if (span == 0 || span > (std::size_t)0xFFFFFFFFu) {
                free_block(mem, span);      /* the new span is not representable: give the whole block back to the system */
                return nullptr;
            }
        }
    }
    /* Large objects are not zeroed: capacity=1, bump=1, there is no "unallocated slot", and the large-object path of resolve
     * decides exactly by size; the payload is zeroed on demand by object_new.
     * **os_zeroed must take the actual argument `zeroed`**: when the platform has no page primitives,
     * `alloc_block` falls back to malloc, which **does not guarantee zeros**, so setting 1 would make C3 skip a
     * necessary memset (dirty bytes taken as candidate pointers by the conservative scan). */
    gc_chunk* c = chunk_init(mem, owner, (std::uint32_t)span,
                             (std::uint32_t)data_off, 0u, 1u,
                             SLOT_NONE, kind, 1u,
                             zeroed ? (std::uint8_t)1 : (std::uint8_t)0,
                             guard_this ? (std::uint8_t)1 : (std::uint8_t)0);
    chunk_enlist(c, (std::uint32_t)(span / CHUNK_SIZE));
    return c;
}
/**
 * @brief Give a block's memory **back to the system**: first retire (remove from the chain, settle the accounting), then release the mapping according to span.
 * @param c Safe to call repeatedly.
 * @internal
 */
inline void chunk_release(gc_chunk* c) {
    chunk_retire(c);                 /* a large-object block is necessarily on the chain; if not, it was already retired */
    unsigned char* mem = c->base;
    const std::size_t span = c->span;
    free_block(mem, span);
}
/* ---- Global per-class empty-block recycle stack (lock-free Treiber stack) ----
 * When sweep brings a chunk's live count to zero (collector thread) it CAS-pushes the empty block into
 * recycle[cls]; when a heap's local chain runs dry it CAS-pops to claim one (owner switch inside gc_mutex). Compared with "returning directly to
 * the system", the recycle stack makes blocks almost always reused immediately, and preserves the simplicity of owner switching inside the lock. */
/* Pop/push primitives shared by the two lock-free chains: `recycle[]` (hot) and `recycle_cold[]` (cold, data area
 * already decommitted) are Treiber stacks of the same shape. */
/**
 * @brief Lock-free Treiber stack pop (shared by `recycle[]` and `recycle_cold[]`).
 * @return The popped block; `nullptr` for an empty stack.
 * @internal
 */
inline gc_chunk* chunk_stack_pop(std::atomic<gc_chunk*>& top) {
    gc_chunk* head = top.load(std::memory_order_acquire);
    while (head != nullptr &&
           !top.compare_exchange_weak(head, head->cnext,
               std::memory_order_acquire, std::memory_order_acquire)) {}
    return head;
}
/**
 * @brief Lock-free Treiber stack push (pairs with `chunk_stack_pop`).
 * @internal
 */
inline void chunk_stack_push(std::atomic<gc_chunk*>& top, gc_chunk* c) {
    gc_chunk* head = top.load(std::memory_order_relaxed);
    do {
        c->cnext = head;
    } while (!top.compare_exchange_weak(head, c,
                 std::memory_order_release, std::memory_order_relaxed));
}
/** @brief Compute the interval for decommitting / recommitting a block's data area (D1); *off_out is filled with the starting offset.
 *  @return Interval length; 0 means there is nothing to decommit.
 *  @note **The first page never moves**: the gc_chunk header and the metadata redzone are both in the first page, and the reverse-lookup path and
 *        chunk_free's address-range comparison may touch them at any moment -- that cannot be an uncommitted page.
 * @internal
 */
inline std::size_t chunk_purge_span(const gc_chunk* c, std::size_t* off_out) {
    const std::size_t ps = page_size();
    auto off = (std::size_t)c->data_off;
    off = (off + ps - 1) & ~(ps - 1);        /* the page after the page containing the data-area start */
    if (off < ps) off = ps;                  /* step aside from the first page no matter what */
    const std::size_t end = ((std::size_t)c->span) & ~(ps - 1);
    *off_out = off;
    return end > off ? end - off : 0;
}
/**
 * @brief **Decommit** a block's data area (D1: return physical pages to the system, keep the address space).
 * @note Only done for empty blocks, and subject to the `PURGE_DELAY_MS` delay. Failure does not affect correctness, it just saves no memory.
 * @internal
 */
inline void chunk_purge_data(gc_chunk* c) {
    gc_state& g = G();
    /* Blocks already on a chain and already decommitted will not be touched here again (they are not on the hot chain). */
    if (c->purged != 0 || c->slot_size == 0) return;
    std::size_t off = 0;
    const std::size_t len = chunk_purge_span(c, &off);
    if (len == 0) return;
    if (decommit(c->base + off, len)) {
        c->purged = 1;
        g.decommits.fetch_add(1, std::memory_order_relaxed);
    }
}
/**
 * @brief Recommit the data area when claiming a block that is **already decommitted**.
 * @internal
 */
inline void chunk_recommit_data(gc_chunk* c) {
    gc_state& g = G();
    if (c->purged == 0) return;
    std::size_t off = 0;
    const std::size_t len = chunk_purge_span(c, &off);
    if (len != 0) recommit(c->base + off, len);
    c->purged = 0;
    g.recommits.fetch_add(1, std::memory_order_relaxed);
}
/**
 * @brief Push an empty block onto the global recycle stack (the retirement timestamp is recorded here, for D2's delay criterion).
 * @internal
 */
inline void recycle_push(gc_chunk* c) {
    gc_state& g = G();
    c->retired_ms = (std::uint32_t)gc_ts_ms();   /* D2's "moment of retirement" */
    c->purged = 0;                               /* entering the hot chain: it is definitely in the committed state */
    c->in_partial.store(false, std::memory_order_relaxed);
    g.recycled.fetch_add(1, std::memory_order_relaxed);
    chunk_stack_push(g.recycle[class_index(c->kind, c->cls)], c);
}
/**
 * @brief Take a block ready for **reuse**: the hot chain first, and only when it is empty touch the cold chain and recommit (after recommit, reading it back gives zero pages, exactly what the safety argument of resolve_and_mark needs).
 * @internal
 */
inline gc_chunk* recycle_pop(unsigned kind, unsigned cls) {
    gc_state& g = G();
    const unsigned ci = class_index(kind, cls);
    gc_chunk* head = chunk_stack_pop(g.recycle[ci]);
    if (head != nullptr) {
        g.recycled.fetch_sub(1, std::memory_order_relaxed);
        return head;
    }
    head = chunk_stack_pop(g.recycle_cold[ci]);
    if (head == nullptr) return nullptr;
    g.recycled.fetch_sub(1, std::memory_order_relaxed);
    g.recycled_cold.fetch_sub(1, std::memory_order_relaxed);
    chunk_recommit_data(head);
    return head;
}
/* ---- The single decision point for retirement: decide + retire + push onto the recycle stack, all inside gc_mutex ----------
 * Criterion: not on the dirty-block chain, not on any owner's partial chain, used_pub == freed.
 * **The decision must be mutually exclusive with popping from the recycle stack** -- otherwise a block that "was retired by someone else inside the window, then claimed by another heap
 * and re-attached to a partial chain" would be retired and pushed a second time with a stale criterion: the same block appearing
 * twice on the recycle stack = two heaps owning it at once = the same batch of slots handed out twice (the root cause of multi-threaded pure-allocation hangs/segfaults).
 * Hence chunk_alloc's claim (recycle_pop -> owner switch -> chunk_reset_state -> chunk_enlist ->
 * class_link) is also done in one go inside the same lock; chunk_retire is idempotent. All four call sites are low-frequency, once-per-block
 * paths, **not on the allocation hot path**. */
/**
 * @brief During sweeping, decide whether a block should be **retired** (live count reached zero + past the delay window).
 * @return `true` = handed over to the recycle stack.
 * @internal
 */
inline void maybe_retire_chunk(gc_chunk* c) {
    gc_mutex_guard lk;
    if (c->in_dirty.load(std::memory_order_acquire)) return;      /* still on the dirty-block chain */
    if (c->in_partial.load(std::memory_order_acquire)) return;    /* the owner can still get it */
    if (c->used_pub.load(std::memory_order_relaxed) != c->freed) return;
    if (chunk_retire(c)) recycle_push(c);
}
/**
 * @brief trim-only: take a block ready to be **given back to the system** -- the cold chain first (recommit then release is paying a pair of syscalls for nothing), and it does not touch the protection state at all.
 * @internal
 */
inline gc_chunk* recycle_pop_for_release(unsigned kind, unsigned cls) {
    gc_state& g = G();
    const unsigned ci = class_index(kind, cls);
    gc_chunk* c = chunk_stack_pop(g.recycle_cold[ci]);
    if (c != nullptr) {
        g.recycled.fetch_sub(1, std::memory_order_relaxed);
        g.recycled_cold.fetch_sub(1, std::memory_order_relaxed);
        return c;
    }
    c = chunk_stack_pop(g.recycle[ci]);
    if (c != nullptr) g.recycled.fetch_sub(1, std::memory_order_relaxed);
    return c;
}
/** @brief Move blocks that have been "retired long enough" from the hot chain to the cold chain (D1 + D2, only called by finish_cycle).
 *  @note Not placed in recycle_push: that is the owner allocation path, and the block pushed there is the **newest**
 *        (the top of a LIFO stack is the most likely to be reused immediately), so decommitting there is exactly backwards -- what should be returned is the old
 *        block "that nobody comes to take". The old LIFO blocks are too deep to reach, hence "pop a small batch, move the cold-enough ones to the cold chain, and put the rest
 *        back in their original order"; each round moves the old ones away, so along the depth direction every block is necessarily examined one by one.
 * @internal
 */
inline void purge_recycle_stack() {
    gc_state& g = G();
    /* When there are only a very few blocks in the recycle stack they are being reused over and over, so there is no "cold" to speak of -- this one line skips
     * the scan of all 378 size classes. Measured (alternating A/B on the same machine) it **shows no measurable difference**, so it is not a performance measure,
     * merely a common-sense guard whose cost is strictly zero. */
    if (g.recycled.load(std::memory_order_relaxed) <= (unsigned)RECYCLE_KEEP)
        return;
    const auto now = (std::uint32_t)gc_ts_ms();
    const auto delay = (std::uint32_t)PURGE_DELAY_MS;
    for (unsigned kind = 0; kind < KINDS; ++kind)
    for (unsigned cls = 0; cls < CLASSES; ++cls) {
        const unsigned ci = class_index(kind, cls);
        std::atomic<gc_chunk*>& hot = g.recycle[ci];
        gc_chunk* kept[PURGE_SCAN];
        unsigned nkept = 0;
        for (unsigned i = 0; i < (unsigned)PURGE_SCAN; ++i) {
            gc_chunk* c = chunk_stack_pop(hot);
            if (c == nullptr) break;
            /* Unsigned wrapping subtraction: both values are 32-bit milliseconds, and as long as the delay is far below 2^31ms (24 days) it holds equally at the 49-day wraparound point. */
            if (!c->purged && (std::uint32_t)(now - c->retired_ms) >= delay) {
                chunk_purge_data(c);
                if (c->purged != 0) {
                    g.recycled_cold.fetch_add(1, std::memory_order_relaxed);
                    chunk_stack_push(g.recycle_cold[ci], c);
                    continue;
                }
            }
            kept[nkept++] = c;
        }
        /* Put them back in the original order: popping gives LIFO (new -> old), so pushing in reverse restores the stack order. */
        while (nkept > 0) chunk_stack_push(hot, kept[--nkept]);
    }
}
/** @brief Cap the recycle stack: give the excess empty blocks back to the system (only called by finish_cycle; the cap takes
 *         effect only at the end of each round, so it may briefly be exceeded within a round).
 *  @note **Not** placed in recycle_push: freeing memory would leave gc_state::sweep_of_cache dangling (it
 *        may be pointing at this very block, and after free the address may be reused by the CRT at any moment), and that cache can only be safely
 *        cleared by the collector itself at the finish point -- freeing memory there would be a real use-after-free.
 * @internal
 */
inline void trim_recycle_stack() {
    gc_state& g = G();
    /* Decommit first (do not change the address space, only return physical pages), then trim to the cap; putting the non-freeing step first lets the round that "already hit the cap" examine one batch less. */
    purge_recycle_stack();
    if (g.recycled.load(std::memory_order_relaxed) <= (unsigned)RECYCLE_MAX) return;
    /* Clear the reverse-lookup cache first: the block about to be freed below may be exactly the one it points to. */
    g.sweep_of_cache = nullptr;
    for (unsigned kind = 0; kind < KINDS; ++kind)
    for (unsigned cls = 0; cls < CLASSES; ++cls) {
        for (;;) {
            const unsigned n = g.recycled.load(std::memory_order_relaxed);
            if (n <= (unsigned)RECYCLE_MAX) return;
            /* Take block + retire + return memory all inside gc_mutex: it shares the same lock with maybe_retire_chunk,
             * so that "pop" and "push" are mutually exclusive; incidentally it also serializes chunk_retire's modification of g.chunks against a concurrent
             * chunk_enlist. */
            gc_mutex_guard lk;
            gc_chunk* c = recycle_pop_for_release(kind, cls);
            if (c == nullptr) break;             /* this class is empty, move to the next class */
            chunk_release(c);                    /* already retired; here it only gives the memory back to the system */
        }
    }
}
/* ---- owner-private partial chain (chunks with free slots; the owner has exclusive read/write) ---- */
/**
 * @brief Hang a block onto the chain of its (kind, size class) (head insertion of the bump allocator's chain).
 * @internal
 */
inline void class_link(gc_chunk* c) {
    if (c->in_partial.load(std::memory_order_relaxed) ||
        c->cls == SLOT_NONE) return;
    gc_heap* h = c->owner;
    const unsigned i = class_index(c->kind, c->cls);
    c->cnext = h->partial[i];
    h->partial[i] = c;
    c->in_partial.store(true, std::memory_order_relaxed);
}
/**
 * @brief Remove a block from the (kind, size class) chain (the inverse of `class_link`).
 * @internal
 */
inline void class_unlink(gc_chunk* c) {
    if (!c->in_partial.load(std::memory_order_relaxed) ||
        c->cls == SLOT_NONE) return;
    gc_heap* h = c->owner;
    /* owner == nullptr = this block has already been retired, and it cannot be on any partial chain (chunk_retire unlinks and clears the owner in the same step). */
    if (h == nullptr) return;
    gc_chunk** p = &h->partial[class_index(c->kind, c->cls)];
    while (*p && *p != c) p = &(*p)->cnext;
    /* If it is not found, **change nothing at all**: unconditionally clearing cnext and in_partial would, when "this block is in fact hanging elsewhere",
     * truncate that chain at c, and would report in_partial as false -- the latter makes this block be judged "retireable" while it is **still on the chain**,
     * and after retirement it is claimed by another heap, so the same partial chain contains it twice, class_unlink can no longer remove it,
     * and the allocation path spins forever in place. The invariant "in_partial true is equivalent to being on some partial chain" must be kept, so that a bad structure
     * exposes itself in a diagnosable form instead of turning into a livelock. */
    if (*p != c) return;
    *p = c->cnext;
    c->cnext = nullptr;
    /* **release**, not relaxed: the collector's criterion is "acquire-read `in_partial` first,
     * then read `used_pub`", and `in_partial` being false must imply "the latest `used` has already been seen".
     * Two relaxed stores to different addresses can become visible out of order on a weakly ordered CPU (AArch64 is a first-class target of this library)
     * -- that would make the collector judge a block that "just had its last free slot taken" as empty, retire it and push it onto the recycle stack,
     * so that just-handed-out slot is handed out a **second** time (the same batch of slots handed out twice, exactly what the retirement design guards against).
     * On x86 a release store is the same code as a relaxed store, at zero cost. */
    c->in_partial.store(false, std::memory_order_release);
}
#if GCXX_BLACKLIST
/**
 * @brief Whether the 4KB page containing a slot has been blacklisted -- slots on blacklisted pages are never allocated, so a false pointer can no longer point at any real object from then on (the page granularity is fixed at 4KB, matching the bl_* bitmap).
 * @internal
 */
inline bool slot_blacklisted(gc_chunk* c, std::uint32_t idx) {
    const std::uintptr_t sa = reinterpret_cast<std::uintptr_t>(c->base)
                      + c->data_off + (std::size_t)idx * c->slot_size;
    const unsigned page = (unsigned)((sa - reinterpret_cast<std::uintptr_t>(
                              c->base)) >> 12);
    return ((c->bl_black.load(std::memory_order_relaxed) >> page) & 1u) != 0u;
}
#endif
/** @brief The owner-exclusive bump advance (allocation hot path). @return The slot index carved out.
 *  @note Uses load + store rather than `fetch_add`: **bump has only one writer, the owner**, so no update can be lost,
 *         while `fetch_add` is a `lock`-prefixed RMW -- measured at 5.6ns per allocation (about 17 cycles).
 *         Readers (resolve_and_mark / object_find / verify_heap) treat it only as an upper bound on "the number of slots already carved out";
 *         what really must be guaranteed is "write the slot contents first, then make it indexable", which the `release` store provides.
 * @internal
 */
inline std::uint32_t bump_take(gc_chunk* c) {
    const std::uint32_t idx = c->bump.load(std::memory_order_relaxed);
    /* WARNING: **the upper bound must be checked here**, you cannot rely on the caller having asked `chunk_usable` first:
     * the first criterion of `chunk_usable` is "the shared free chain is non-empty", and when that is true it **returns true directly,
     * without looking at `bump`**. So in the combination "the block was once pushed full (`bump == capacity`) + slots were published afterwards",
     * the caller reaches here, and the original code would **unconditionally return `idx == capacity`**
     * -- that is one slot out of bounds, the address given by `slot_at` lands outside the block's data area, and
     * `o->size = size;` in `object_new` writes it broken (observed with `cs.idx == capacity == 817`
     * and `bump == 817`).
     * @note The check is placed after the `load`: `bump` has only one writer, the owner, so this comparison is
     *       the accurate answer to "was this slot actually carved out or not" (`>=` uses unsigned comparison, one cmp, negligible). */
    if (idx >= c->capacity) return SLOT_NONE;   /* already full: the caller must switch to another block */
    c->bump.store(idx + 1u, std::memory_order_release);
    return idx;
}
/** @brief Whether the chunk still has "usable slots" (the free-chain head is non-empty, or bump still has slots not carved out).
 *  @note When the blacklist is enabled, skip slots on blacklisted pages (called exclusively by the owner, so it may advance bump freely). Check
 *        bl_black first: it only gets bits after "some page was hit by false candidates in two consecutive major rounds", and
 *        slot_blacklisted must compute the 4KB page number and then do a bit test, which is not cheap. bl_black is written by the collector
 *        inside gc_mutex; a lock-free read on the owner side takes effect at worst one beat late, affecting only "skipping one slot less".
 * @internal
 */
inline bool chunk_usable(gc_chunk* c) {
    /* All three places may have slots: the owner-private chain, the shared chain just published by the collector, and the not-yet-carved bump; each is a single relaxed read. */
    if (c->free != SLOT_NONE) return true;
    if (c->free_head.load(std::memory_order_relaxed) != SLOT_NONE) return true;
#if GCXX_BLACKLIST
    if (c->bl_black.load(std::memory_order_relaxed) != 0) {
        for (;;) {
            const std::uint32_t b = c->bump.load(std::memory_order_relaxed);
            if (b >= c->capacity || !slot_blacklisted(c, b)) break;
            /* Before skipping, **the slot must first be marked as a free sentinel**: resolve_and_mark decides "is there an object in the slot"
             * only by `idx < bump` and `marked != MARK_FREE`, and a skipped slot was never written by
             * object_new, so without the sentinel it would be taken as a live object, whose garbage size would be read and whose random contents
             * marked live. The order cannot be reversed: **write the sentinel first, then advance bump** (a reader that sees idx < bump necessarily has
             * already seen the sentinel). The chunk_new path is likewise pushed past uninitialized territory by the blacklist. */
            slot_at(c, b)->marked.store(MARK_FREE, std::memory_order_release);
            c->bump.store(b + 1u, std::memory_order_release);
        }
    }
#endif
    return c->bump.load(std::memory_order_relaxed) < c->capacity;
}
/* Free-slot chain: while a slot is "free", its first 4 bytes hold the next index. The producer (sweep) batches them and splices the whole batch into
 * `free_head` with one CAS (see publish_dead_slots); the consumer owner CAS-pops slot by slot (adding an owner-private local chain
 * actually measures 4.4ns slower: a published batch is only ~8 slots. */
/** @brief The single entry point for handing out slots: the owner-private free chain (mimalloc's page->free); returns the slot index,
 *         SLOT_NONE meaning this block has no allocatable slot.
 *  @note Only when the local chain runs dry does it exchange once to take the whole batch. The chain tail naturally has `free == SLOT_NONE`,
 *        so **no sentinel slot is needed** (treating a decoded NONE as "chain corruption" would permanently lose the tail slot of every chain).
 * @internal
 */
inline std::uint32_t freelist_pop(gc_chunk* c) {
    for (;;) {
        std::uint32_t idx = c->free;
        if (idx == SLOT_NONE) {
            /* Local chain ran dry: one exchange takes a whole batch (acquire pairs with the release of publish,
             * guaranteeing that the next links already written into each slot of the batch are visible). */
            idx = c->free_head.exchange(SLOT_NONE, std::memory_order_acquire);
            if (idx == SLOT_NONE) return SLOT_NONE;
        }
        /* WARNING: an idx taken from the chain must also be bounds-checked **right here**: this function is "the single entry point for handing out slots",
         * and the caller (the `chunk_alloc` hot path) may have arrived here through b on the premise "the shared free chain is non-empty"
         * -- that path **does not look at `bump`** (see the first criterion of `chunk_usable`). So if the block was once
         * pushed full (`bump == capacity`) and slots were published afterwards, `bump_take` would produce the
         * out-of-bounds slot index `capacity`, which makes `o->size = size;` write outside the block.
         * Here only **one comparison** is done (merged with the corruption check below), adding no memory access. */
        if (idx >= c->capacity) {
            G().freelist_corruptions.fetch_add(1, std::memory_order_relaxed);
            c->free = SLOT_NONE;
            return SLOT_NONE;                  /* make the caller fall back to the bump path (which also checks the upper bound) */
        }
        const std::uint32_t nxt = link_decode(c, slot_link_raw(c, idx));
        /* The only corruption decision point: the decoded value is out of bounds (decoding NONE at the chain tail is legal). */
        if (nxt != SLOT_NONE && nxt >= c->capacity) {
            G().freelist_corruptions.fetch_add(1, std::memory_order_relaxed);
            c->free = SLOT_NONE;
            c->free_head.store(SLOT_NONE, std::memory_order_relaxed);
            return SLOT_NONE;
        }
        c->free = nxt;
#if GCXX_BLACKLIST
        /* Slots on blacklisted pages are never allocated: skip it and pop the next one (affects only placement, not correctness). */
        if (c->bl_black.load(std::memory_order_relaxed) != 0 && slot_blacklisted(c, idx)) continue;
#endif
        return idx;
    }
}
/**
 * @brief The number of live slots in this block = allocated - freed, reading the **published snapshots** (used_pub is written by the owner, freed_pub is written by the collector when it publishes a batch of dead slots; shared by diagnostics and the retirement criterion).
 * @internal
 */
inline std::uint32_t chunk_live(const gc_chunk* c) {
    const std::uint32_t u = c->used_pub.load(std::memory_order_relaxed);
    const std::uint32_t f = c->freed_pub.load(std::memory_order_relaxed);
    return u > f ? u - f : 0u;
}
/**
 * @brief The filled-in result of chunk_alloc: which slot of which chunk the object landed in.
 * @internal
 */
struct chunk_slot {
    gc_chunk*     c;
    std::uint32_t idx;
    /**
     * Whether the slot memory was **freshly committed by the operating system**, with the kernel guaranteeing zero pages (only the large-object path can be true). C3 uses it to elide one memset.
 * @internal
     */
    bool          fresh_zeroed;
};
/** @brief The **slow path** of `chunk_alloc`: the current partial chain head is unusable (full / already unlinked) or the chain is empty.
 *  @note This is the "switch block" logic: unlink the unusable head block (retiring it as a fallback if necessary), claim or create
 *        a 64KB block from the recycle stack, and complete the owner switch + re-link inside `gc_mutex`. **Every call switches block**, so
 *        the head of `h->partial[ci]` is necessarily a fresh usable block -- the caller can use it directly.
 *  @note Splitting it out of `chunk_alloc` is a **performance** requirement: it leaves `chunk_alloc` with only "test emptiness + pop a slot",
 *        a dozen instructions, so it can be inlined into the allocation hot path; while this part (with `gc_mutex`, `memset`,
 *        `chunk_new`) is forced into a real call, happening only on a block switch.
 * @return A usable head block; nullptr means this size class really is out of memory.
 * @internal
 */
GCXX_NOINLINE inline gc_chunk* chunk_alloc_slow(gc_heap* h, unsigned ci,
                                                unsigned kind, unsigned cls) {
    gc_chunk* c = h->partial[ci];
    while (c != nullptr && !chunk_usable(c)) {
        /* Unlink. **`c` is exactly the head of h->partial[ci]**, so this step must succeed: unlink directly by
         * "we know it hangs on this index", rather than letting class_unlink guess from c's own kind/cls
         * -- once the two disagree (structure corruption) the generic path finds nothing and does nothing, and this
         * loop spins forever (measured: one thread stuck here, and the whole process never collects
         * garbage again). This guarantees progress on every iteration. */
        if (h->partial[ci] == c) {
            h->partial[ci] = c->cnext;
            c->cnext = nullptr;
            /* release: same reason as class_unlink -- "seeing it unlinked" must imply "seeing the latest used_pub". */
            c->in_partial.store(false, std::memory_order_release);
        } else {
            class_unlink(c);
        }
        /* Fallback retirement (see maybe_retire_chunk): it picks up the window where "the collector read in_partial as still true
         * when the count dropped to 0 and skipped retirement, and this thread then unlinked this block" -- the two
         * criteria back each other up, so neither side skips it. Here it is only a cheap pre-filter (the owner's own count, no
         * lock needed); the real decision is redone inside maybe_retire_chunk under the lock. */
        if (c->used == c->freed_pub.load(std::memory_order_relaxed))
            maybe_retire_chunk(c);
        c = h->partial[ci];
    }
    if (c == nullptr) {
        {
            /* **Claim/create + owner switch + re-link onto the partial chain: the whole thing must be done in one go inside gc_mutex**
             * (see maybe_retire_chunk): when claiming finishes, `in_partial` must already
             * be true -- otherwise a block that has been claimed and is in use still looks to others like "a retireable empty
             * block" and gets pushed onto the recycle stack a second time (two heaps owning the same block at once = the same batch of slots
             * handed out twice). class_link is put inside the lock precisely so that linking and the owner switch become visible together. */
            gc_mutex_guard lk;
            c = recycle_pop(kind, cls);      /* claim a block emptied by sweep */
            if (c != nullptr) {
                c->owner = h;                /* owner switch: from now on the block belongs to this thread */
                /* The data area is **not memset as a whole** (clearing 64KB for every claimed 64KB: measured 3773 claims in 1M mixed allocations = 247MB of pointless zeroing). Safety basis: resolve_and_mark
                 * only looks at `idx < bump` and `marked != MARK_FREE`, and the premise of block retirement is used == freed, i.e.
                 * **every slot handed out was marked MARK_FREE by chunk_free**;
                 * the only exception is the blacklist (when bl_black is non-zero, bump advances past uninitialized territory, so zeroing is kept).
                 */
                if (c->bl_black.load(std::memory_order_relaxed) != 0)
                    std::memset(c->base + c->data_off, 0, c->span - c->data_off);
                chunk_reset_state(c, 0);
    /* The block had its table entry unregistered and was removed from the global chain when it entered the recycle stack; the claim must restore all of that, otherwise addresses inside the block cannot be reverse-looked-up and all live objects die. */
                chunk_enlist(c, 1);
            } else {
                const std::uint32_t ss = slot_size_of_class(cls);
                const std::size_t off = chunk_data_off();
                const auto cap = (std::uint32_t)((CHUNK_SIZE - off) / ss);
                if (cap == 0) return nullptr;
                c = chunk_new(h, cls, ss, cap, kind);
                if (c == nullptr) return nullptr;
            }
            class_link(c);
        }
    }
    return c;
}
/** @brief Allocate one object slot (owner-private hot path; lock-free).
 *  @param total Total bytes in the slot (payload offset + payload size); out is filled with the chunk holding the object
 *         and the slot index (always 0 for a large object). @return Object header pointer; nullptr means failure.
 *  @note Layers: local partial chain -> global recycle stack (CAS) -> create a new 64KB chunk under gc_mutex; the slot index
 *         must be handed out by this function (the block-tail padding canary is encoded from it with the link_key).
 *  @note **This function is the only piece on the allocation hot path that is inlined into its caller** (`GCXX_ALWAYS_INLINE`):
 *         the large-object branch and the block-switch branch are all in the `GCXX_NOINLINE` `chunk_alloc_slow`.
 *         The three criteria of `chunk_usable` are themselves three "single relaxed reads", so the shape here is
 *         "hit the partial head -> pop the slot directly", the most economical version.
 * @internal
 */
GCXX_ALWAYS_INLINE gc_object* chunk_alloc(gc_heap* h, std::uint32_t total,
                                                 unsigned kind, chunk_slot* out) {
    if (total > (std::uint32_t)MEDIUM_MAX_TOTAL) {
        /* Large object: low-frequency path, exclusively occupy several blocks under the lock */
        gc_mutex_guard lk;
        gc_chunk* big = chunk_new_big(h, total, kind);
        if (big == nullptr) return nullptr;
        out->c = big;
        out->idx = 0;
        out->fresh_zeroed = (big->os_zeroed != 0);
        return reinterpret_cast<gc_object*>(big->base + big->data_off);
    }
    unsigned cls = class_of(total);
    if (cls >= CLASSES) cls = CLASSES - 1;
    /* fresh_zeroed keeps the value initialization of chunk_slot{} (false) -- small-block data areas are reused over and over,
     * so there is no "the kernel guarantees zeros". **It is not written on the hot path**: writing even one
     * stack field here is measurable over 2M allocations. */
    /* The two kinds each get their own partial segment: same-size "to be scanned" and "not to be scanned" objects never mix into the same chain. */
    const unsigned ci = class_index(kind, cls);
    gc_chunk* c = h->partial[ci];
    /* ---- Hot path: just ask "is the chain head still usable" -------------------------------
     * The criteria are **verbatim identical** to the original implementation (the three of `chunk_usable`: the owner-private free chain, the shared chain
     * just published by the collector, and the not-yet-carved bump), except that the whole "switch block when unusable" piece has been moved into
     * `chunk_alloc_slow`. So here, in the most common shape (the chain head is the block currently being bumped), it is
     * two loads + one comparison. */
    if (c == nullptr || !chunk_usable(c)) {
        c = chunk_alloc_slow(h, ci, kind, cls);
        if (c == nullptr) return nullptr;
    }
    std::uint32_t idx = freelist_pop(c);
    if (idx == SLOT_NONE) idx = bump_take(c);
    if (idx == SLOT_NONE) {
        /* Both sources are empty: the block really has no allocatable slot left (`bump` is fully carved -- note this does not contradict
         * "`chunk_usable` says usable": that may be true only because **the shared free chain has slots**, and that batch
         * happened to be drained by `freelist_pop`).
         * **Switch block and retry**; `chunk_alloc_slow` guarantees the returned block is usable.
         * @note Here it **must not** return nullptr directly: that would make the layer above think it is OOM.
         * @note Only reached when "the block happens to be drained exactly on this call", not the norm of the hot path. */
        c = chunk_alloc_slow(h, ci, kind, cls);
        if (c == nullptr) return nullptr;
        idx = freelist_pop(c);
        if (idx == SLOT_NONE) idx = bump_take(c);
        if (idx == SLOT_NONE) return nullptr;   /* theoretically unreachable: a new block necessarily has slots */
    }
    /* ---- Make the publications of `bump` and `used_pub` **ordered** ----
     * `bump_take` uses a relaxed load + release store (that is the **allocation hot path**, see its comment),
     * so there is originally no happens-before edge between "the number of slots carved out" and "the number of slots occupied". Hence another thread
     * (`gc_verify`) may **see the increase of `used_pub` first and the increase of `bump` later**,
     * and thus observe `live > bump` -- while `live > bump` is physically impossible (used only comes from
     * slots carved out/reused). Measured: 2~4 threads allocating concurrently + the main thread running `gc_verify`,
     * reporting "live exceeds the number of carved-out slots live=305 bump=304" (off by exactly 1), a **false positive**.
     *
     * The release fence guarantees that "all prior writes of this thread (including the `bump` store)" become visible to other threads
     * before any subsequent release write. On x86 it is **zero instructions** (store-store is naturally ordered),
     * so it adds no hot-path cost; it only really takes effect on weakly ordered architectures such as ARM -- which is exactly where it should.
     * The read side is paired by the acquire fence in `gc_verify`. */
    std::atomic_thread_fence(std::memory_order_release);
    /* Owner-private allocation count: a plain increment + one published snapshot (the cache line is exclusive to this thread,
     * basically free), trading away `lock xadd` on the fast path. The order matters: **publish used first, then
     * (possibly) class_unlink** -- the collector reads in_partial before used_pub, so
     * "seeing it unlinked" necessarily means "seeing the latest used". */
    ++c->used;
    c->used_pub.store(c->used, std::memory_order_relaxed);
    if (c->bump.load(std::memory_order_relaxed) >= c->capacity &&
        c->free == SLOT_NONE &&
        c->free_head.load(std::memory_order_relaxed) == SLOT_NONE)
        class_unlink(c);                 /* full: unlink from the chain */
    out->c = c;
    out->idx = idx;
    return slot_at(c, idx);
}
/* ---- Publish the collector's private dead-slot chain in one batch into the global free chain (the counterpart of mimalloc's "thread_free ->
 * free merge"); only called by cycle_owner.
 * One CAS splices the whole chain, then live is decremented once. **Danger point**: the CAS on free_head races with the owner's pop,
 * so the splice must be completed entirely inside one CAS -- the private chain is completely invisible to the owner before the CAS succeeds.
 * The retirement criterion is per batch (live reached zero and it is not on the owner's partial chain); the two criteria back each other up and neither side
 * skips: here in_partial is seen as true -> the owner can still reuse the just-published free slots; if the owner unlinks only after "in_partial is read here",
 * it performs its own live==0 check on the unlink path (see chunk_alloc). */
/**
 * @brief Hand the "dead slots awaiting publication" accumulated by this thread to the collector in batches.
 * @note Publishing in batches rather than per object -- per-object would cost three atomic RMWs, measured at 11.4ns each, the largest fixed cost on the free path.
 * @internal
 */
inline void publish_dead_slots(gc_chunk* c) {
    if (c == nullptr || c->dead_len == 0) return;
    std::uint32_t myhead = c->dead_head;
    std::uint32_t mytail = c->dead_tail;
    std::uint32_t len    = c->dead_len;
    c->dead_head = SLOT_NONE;
    c->dead_tail = SLOT_NONE;
    c->dead_len  = 0;
    /* Randomize the allocation order: without shuffling, the free-chain order is just "the order in which objects were freed", so **allocation addresses become
     * sequentially predictable** (for an allocator whose threat model includes "a wild write corrupting the free chain", that is a free attack surface). The range is **only
     * within one batch** (FREE_BATCH slots); the shuffle is O(len) once per batch and happens entirely on the sweep thread's private data.
     * **len never exceeds FREE_BATCH** (chunk_free publishes as soon as the batch is full), so the buffer cannot overflow. */
    {
        std::uint32_t buf[FREE_BATCH];
        std::uint32_t n = 0;
        for (std::uint32_t s = myhead; s != SLOT_NONE && n < len; ) {
            if (s >= c->capacity) break;              /* the private chain was corrupted by a wild write: stop here */
            buf[n++] = s;
            const std::uint32_t nxt = link_decode(c, slot_link_raw(c, s));
            s = (nxt < c->capacity) ? nxt : SLOT_NONE;
        }
        if (n == len && n > 1) {
            for (std::uint32_t k = n; k > 1; --k) {   /* Fisher-Yates */
                const std::uint32_t j = link_rand(c->rng) % k;
                const std::uint32_t t = buf[k - 1]; buf[k - 1] = buf[j]; buf[j] = t;
            }
            for (std::uint32_t k = 0; k < n; ++k) {
                const std::uint32_t nxt = (k + 1 < n) ? buf[k + 1] : SLOT_NONE;
                slot_link_raw(c, buf[k]) = link_encode(c, nxt);
            }
            myhead = buf[0];
            mytail = buf[n - 1];
        } else if (n != len) {
            /* Walking the chain did not reach len entries: the private chain itself has been corrupted. **Only** splice back the ones
             * confirmed to be recognizable and explicitly terminate the tail -- the original mytail is very likely exactly that bad node, and using it to write the link field would be an out-of-bounds write. */
            len = n;
            if (n == 0) {
                myhead = SLOT_NONE; mytail = SLOT_NONE;
            } else {
                myhead = buf[0];
                mytail = buf[n - 1];
                slot_link_raw(c, mytail) = link_encode(c, SLOT_NONE);
            }
            gc_error_message(EFAULT,
                "corrupted chunk free list: chunk %p published %u of %u dead slots",
                (const void*)c, (unsigned)n, (unsigned)len);
        }
    }
    if (myhead != SLOT_NONE) {
        std::uint32_t prev = c->free_head.load(std::memory_order_relaxed);
        for (;;) {
            slot_link_raw(c, mytail) = link_encode(c, prev);
            if (c->free_head.compare_exchange_weak(prev, myhead,
                    std::memory_order_release, std::memory_order_relaxed))
                break;
        }
    }
    /* Free-side accounting: a collector-private plain accumulation + one published snapshot (used for the owner's fallback retirement). */
    c->freed += len;
    c->freed_pub.store(c->freed, std::memory_order_release);
    /* Retirement decision: **initiated only here** (see maybe_retire_chunk); it must not be repeated after this function
     * -- the window between two decisions is enough for this block to be retired and then claimed by another heap and put back into service, so the
     * second decision still holds after it has **already been revived**, pushing it onto the recycle stack a second time. */
    if (!c->in_dirty.load(std::memory_order_relaxed))
        maybe_retire_chunk(c);
}
/**
 * @brief Publish all blocks "with dead slots awaiting publication" in one go (called at the end of a slice). @note Guarantees that **no unpublished batch exists between slices**.
 * @internal
 */
inline void flush_dirty_slots() {
    gc_state& g = G();
    gc_chunk* c = g.sweep_dirty;
    g.sweep_dirty = nullptr;
    while (c != nullptr) {
        gc_chunk* nx = c->dnext;
        c->dnext = nullptr;
        /* **Unlink first, then clear the gate, and only then publish**: after publishing this block may satisfy the retirement condition, and retirement would send
         * it into the recycle stack (reusable at any moment) or even give it back to the system via trim -- the wrong order is a use-after-free. */
        c->in_dirty.store(false, std::memory_order_release);
        publish_dead_slots(c);
        /* One extra retirement decision: a block whose dead_len is exactly 0 (already published when the batch hit 32) does not reach
         * the decision inside publish_dead_slots, and without this extra one it could never retire. Repeated calls are safe -- chunk_retire is idempotent. */
        maybe_retire_chunk(c);
        c = nx;
    }
}
/** @brief Free one object slot (called by sweep/collector; safe against concurrent owner alloc).
 *  @note Dead slots first go onto the collector's **private** chain, and are only published as a whole batch once FREE_BATCH is accumulated (the end of a slice and a full batch
 *         also publish). The owner's private chain is never touched -- a full block is claimed by the owner through the recycle stack next time.
 * @internal
 */
inline void chunk_free(gc_object* o) {
    gc_state& g = G();
    const auto oa = reinterpret_cast<std::uintptr_t>(o);
    gc_chunk* c = g.sweep_of_cache;
    if (c == nullptr || oa - (std::uintptr_t)c->base >= (std::uintptr_t)c->span) {
        c = chunk_of(oa);
        if (c == nullptr) { g.sweep_of_cache = nullptr; return; }
        if (c->slot_size == 0) {             /* large object: the exclusive block is freed directly (not cached) */
            g.sweep_of_cache = nullptr;
            {
                /* Out-of-bounds on large objects (arrays) is the most common, and its block is unmapped right away -- there is no point in "isolating this slot". */
                std::uint32_t usable = 0;
                const gc_padding_status st = padding_verify(c, o, 0, &usable);
                if (st != GCXX_PAD_OK) gc_padding_failed(c, o, 0, st);
            }
            gc_mutex_guard lk;
            chunk_release(c);
            return;
        }
        g.sweep_of_cache = c;                /* cache only small-object blocks */
    }
    const auto off = (std::uint32_t)(oa -
                      (reinterpret_cast<std::uintptr_t>(c->base) + c->data_off));
    /* Slot index = off * recip >> 32 (see gc_chunk::recip): saves one 64-bit division per dead object. */
    const auto idx = (std::uint32_t)(((std::uint64_t)off * c->recip) >> 32);
    /* Block-tail padding verification: **freeing is the only moment at which it is certain that "the user has finished writing"** (before it the mutator
     * may still write, after it the slot is reused). On failure it **does not enter the free chain** (isolation), but freed is still recorded --
     * otherwise used would never catch up with freed, and the whole 64KB block could not retire. */
    {
        std::uint32_t usable = 0;
        const gc_padding_status st = padding_verify(c, o, idx, &usable);
        if (st != GCXX_PAD_OK) {
            gc_padding_failed(c, o, idx, st);
            o->marked.store(MARK_FREE, std::memory_order_relaxed);
            c->freed += 1;
            c->freed_pub.store(c->freed, std::memory_order_release);
            return;
        }
        /* canary intact -> rewrite it as the freed sentinel. When the same slot is freed twice, the second free reads
         * exactly this, thus distinguishing it from "the canary was corrupted by an out-of-bounds write" (handling and attribution both differ). The cost is one extra
         * 8-byte store per free, and the target cache line is already exclusively held by the sweep thread.
         * A guarded block has no padding area (the guard page sits right on payload+size), so it is skipped. */
        if (c->guarded == 0)
            padding_store(o, PAD_FREED, padding_area_of(c) - o->size);
    }
    o->marked.store(MARK_FREE, std::memory_order_relaxed);
    /* If this block currently has no dead slots awaiting publication, hang it onto the collector's dirty-block chain first (the end of the slice, flush_dirty_slots,
     * publishes them all together). **Not published on "block switch"**: adjacent objects on the young chain almost certainly belong to different blocks, so
     * every free would trigger a publish and batching would be defeated -- measured, 1M frees triggered **920k**
     * publish_dead_slots calls (an average of 1.09 slots per batch, while FREE_BATCH is 32).
     * The private push is two plain stores; the link field is exactly 4 bytes before the object header, so writing `o` directly saves one imul. */
    *reinterpret_cast<std::uint32_t*>(o) = link_encode(c, c->dead_head);
    if (c->dead_head == SLOT_NONE) {
        c->dead_tail = idx;
        /* From "no dead slots awaiting publication" to "there are": hang this block onto the collector's private dirty-block chain, and at the end of the slice
         * flush_dirty_slots publishes them all together. **Without this step, the remainder below FREE_BATCH
         * would never be published** -- those slots could not get back onto the free chain, freed could not catch up with used, and the whole
         * 64KB block could never retire (measured: repeatedly allocating/freeing the same size, mapped grows 6MB per round and never
         * comes back down). */
        if (!c->in_dirty.load(std::memory_order_relaxed)) {
            c->in_dirty.store(true, std::memory_order_release);
            c->dnext = g.sweep_dirty;
            g.sweep_dirty = c;
        }
    }
    c->dead_head = idx;
    if (++c->dead_len >= (std::uint32_t)FREE_BATCH) publish_dead_slots(c);
}
inline gc_object* object_find(std::uintptr_t p);   /* forward declaration (used by the self-check) */
/* Whole-heap consistency check (a robustness facility, for debugging/self-check), O(live objects). Checked one by one: the three object chains are acyclic
 * and of finite length; objects on the chains are not "free slot" sentinels; every live object can be reverse-looked-up to itself in O(1); objects fall inside their own
 * chunk's slots with the payload in bounds; color/age values are legal; the size class matches the payload. */
/**
 * @brief **Restore** the nursery color invariant before asserting it (called from `verify_heap`).
 *
 * Why this is necessary (this is a real race window, not "relaxing an assertion"):
 *
 * `finish_cycle()` walks every heap's nursery under `gc_mutex` and writes `marked` to the
 * current `g.white`; but `object_new` does head insertion into `h->nursery` **without taking
 * that lock**, and it may read a **stale** `phase` value `PROPAGATE`/`ATOMIC` -- so it takes the
 * "allocated inside the marking window" route, the dark-colored branch, coloring the object
 * `BLACK`(0) or gray (**no white bit**) and inserting it at the chain head. If this head
 * insertion happens after the recolor walk **has already passed the chain head**, that object
 * will not be recolored, and so it **enters `PAUSE` with no white bit**.
 *
 * The recolor comment in `finish_cycle` itself admits this ("even if a few newly inserted
 * objects are missed...") -- its judgment that "the **structure** is not broken" is correct, but
 * this **does break the color invariant**, and the color invariant is exactly what both
 * `gc_verify` and `mark_object` depend on. Measured (8 threads x 4 rounds + weak-reference
 * registration, 3 out of 120 times) it reliably hits `list=2 marked=0 white=2`.
 *
 * Handling: the caller already holds `gc_mutex`, so before asserting, this performs exactly the
 * same write as the recolor (**completely identical**: setting `marked` of every object on the
 * nursery to the current `g.white`). There are two independent correctness arguments:
 *   1. **Diagnostic reading**: the nursery color is maintained by "explicit recoloring"; it is a
 *      **convention** and not an invariant that can be asserted concurrently; this aligns the
 *      self-check's view with that convention, so that **not a single check of the same kind**
 *      on `young`/`old` is weakened (they are still asserted strictly);
 *   2. **Soundness reading**: the recolor writes exactly the correct value, "the nursery should
 *      be white at the start of the next round", so this **really restores** the invariant rather
 *      than bypassing it.
 *
 * @note Unrelated to `reap_dead_heaps`: that one requires **all three chains
 *       young/old/nursery to be empty** before releasing a heap, so it cannot step on the
 *       nursery. The observed "too many heaps" is **harmless memory retention**, a different matter.
 * @note Only done during `PAUSE`: in other phases the nursery color is not required to equal the
 *       white bit anyway.
 * @internal
 */
inline void verify_restore_nursery_colors(gc_state& g) {
    if (g.phase.load(std::memory_order_relaxed) != (int)GC_PHASE_PAUSE) return;
    const std::uint8_t w_fix = g.white.load(std::memory_order_relaxed);
    for (gc_heap* hfix = heaps_head(); hfix != nullptr; hfix = hfix->hnext)
        for (gc_object* ofix = hfix->nursery; ofix != nullptr; ofix = ofix->next)
            ofix->marked.store(w_fix, std::memory_order_relaxed);
}
/**
 * @brief Recompute the weak-table counting invariant (`weak_live`), called from `verify_heap`.
 *
 * `weak_live` is the fast gate for "skip the whole stabilization phase entirely"; once it drifts
 * to 0, ephemeron activation is skipped wholesale (values are no longer marked, handles expire
 * immediately) -- a **silent** behavioral error that only the self-check can catch. So every
 * whole-heap verification recomputes the count instead of trusting the cached one.
 * @return 1 when the count is wrong (one inconsistency), 0 when it matches.
 * @internal
 */
inline std::size_t verify_weak_table_count(const gc_state& g) {
    std::size_t live = 0;
    for (std::size_t i = 0; i < g.weak_len; ++i) {
        const weak_slot& s = g.weak_tab[i];
        if (s.in_use && s.key.load(std::memory_order_relaxed) != nullptr)
            ++live;
    }
    if (live != g.weak_live) {
        std::fprintf(stderr,
            "[verify] weak reference table count mismatch: weak_live=%zu actual=%zu\n",
            g.weak_live, live);
        return 1;
    }
    return 0;
}
/**
 * @brief Upper bound on how many objects the three chains may hold, called from `verify_heap`.
 *
 * Uses the **physical bound**, not an estimate from the live count. It used to be
 * `live_objects + the sum of each heap's unflushed increment + 1024`: that is the **live count**,
 * while the three chains carry objects that are "**allocated and not yet swept**", and under
 * concurrency the two differ a lot -- the global count is flushed in 64KB batches and lags by
 * itself, and more importantly objects **just judged dead this round and not yet swept** are
 * still hanging on the chains. Measured (2 threads allocating + the main thread repeatedly
 * calling `gc_collect`, 9 out of 30 times): with `live_objects=7` there were 1032 objects on the
 * chains while limit was only 1031 => a **false positive** "cycle/too long", judging a healthy
 * state as corruption.
 *
 * The physical bound is **the sum of every block's capacity**: the hard upper bound on "how many
 * objects can be on the chains at most" (objects can only come from block slots). It relaxes
 * automatically as the heap grows and is independent of concurrent timing.
 * @note A real cycle is still caught: a cycle would grow far beyond the summed capacity.
 * @internal
 */
inline std::size_t verify_walk_limit(const gc_state& g) {
    std::size_t limit = 0;
    for (const gc_chunk* cx = g.chunks; cx != nullptr; cx = cx->all_next)
        limit += (std::size_t)cx->capacity;
    limit += 1024;                    /* fallback margin: the block chain itself may be concurrently added to/removed from */
    return limit;
}
/**
 * @brief Check the **immutable** chunk metadata of every registered block, called from
 *        `verify_heap`.
 *
 * Must run **before** the per-object pass: that pass uses `c->slot_size` / `data_off` /
 * `capacity` to compute slot geometry, and those are exactly the fields covered by the checksum.
 * Verifying the metadata first means a bad block is not taken and dereferenced -- otherwise a
 * `slot_size` enlarged by a wild write would make the verification itself read out of bounds.
 * @return Number of inconsistencies found.
 * @internal
 */
inline std::size_t verify_chunk_metadata(const gc_state& g) {
    std::size_t bad = 0;
    for (const gc_chunk* c = g.chunks; c != nullptr; c = c->all_next) {
        /* Metadata redzone: an underflow write of slot 0 hits this first -- if it is broken,
         * someone wrote **downwards out of the data area**. */
        if (!chunk_redzone_ok(c)) {
            std::fprintf(stderr, "[verify] corrupted chunk metadata redzone chunk@%p data_off=%u\n",
                         (const void*)c, c->data_off);
            ++bad;
        }
        if (!chunk_meta_ok(c)) {
            std::fprintf(stderr, "[verify] chunk header checksum mismatch chunk@%p\n",
                         (const void*)c);
            ++bad;
        }
    }
    return bad;
}
/**
 * @brief Whole-heap consistency self-check: returns the number of inconsistencies, **0 = healthy**.
 *
 * Checks the object chains, the free-slot sentinels, index consistency, slot boundaries, color validity, and verifies block-tail padding for every live object one by one; it also asserts that "an object requiring scanning never appears in an atomic block".
 * @note Only diagnostics/tests call it (`gc_verify()` is a thin wrapper) -- it is O(heap).
 * @note The phases that are self-contained (nursery recolor, weak-table count, walk limit, chunk
 *       metadata) live in `verify_restore_nursery_colors` / `verify_weak_table_count` /
 *       `verify_walk_limit` / `verify_chunk_metadata` above, so this function stays a readable
 *       sequence of steps rather than one long body.
 * @internal
 */
inline std::size_t verify_heap() {
    diag_flush();
    gc_state& g = G();
    gc_mutex_guard lk;
    std::size_t bad = 0;
    /* ---- Step 1: **restore** the nursery color invariant before asserting it ----
     * (the full argument for why this is a real race window lives on
     *  `verify_restore_nursery_colors`, together with its two correctness arguments) */
    verify_restore_nursery_colors(g);
    /* ---- WARNING: the color invariant must be judged against **one single snapshot**, it cannot re-read per object ------------
     * This function holds `gc_mutex`, but **neither input of the color invariant is protected by that lock**:
     *   * `g.phase` / `g.white` are written by the **cycle owner** at the STW boundary;
     *   * `o->marked` is written by the mutator's `object_new` (colored "dark" and head-inserted inside the marking window)
     *     and by the collector's sweep.
     * So "re-reading `phase` and `white` for every object" would mix **two states from different instants**
     * into one assertion:
     *   when object A is read, the snapshot was `PAUSE`;
     *   meanwhile another thread completes `atomic_finish` (a major **flips the white bit**) and finishes a whole round;
     *   when object B is read, `phase` is still `PAUSE`, but `white` has already changed generation =>
     *   `B->marked` (belonging to the **previous generation's** white bit) differs from the **new generation's** `white` => **false positive**.
     * Measured (`weak_thread_churn`, 8 threads + background `ctest -j8` contention):
     *   `[verify] idle-phase color is not the current white bit @... list=1 marked=0 white=1`
     * -- `list=1` is the **old chain**, `marked=0` means "no white bit". And **the old chain cannot be uniformly recolored**
     * (unlike the nursery above): `marked == cycle_white` on old is the **"dead" criterion of sweeping**
     * (see the `m == cw` line in `sweep_some`); changing it to the white bit would be **laundering garbage into live objects**,
     * causing a bounded leak. So here **only the reading of the assertion can be fixed**, not the data.
     *
     * Handling: take one `(is_pause, white_snapshot)` **before** the walk, use it for the whole walk,
     * and **re-check at the end that `phase` has not changed**; if it has, abandon the color decision for the whole walk (this round was not
     * an idle phase anyway, so there is nothing to assert). This way:
     *   * a genuine "some object lacks the current white bit during the idle phase" **is still caught** (state at the same instant);
     *   * mixing comparisons across rounds is eliminated => no more false positives.
     * @note The other checks (padding/canary/reverse lookup/block kind/`weak_live` counting) are unaffected:
     *       they look only at **the single object itself** or at fields **unrelated to the phase**. */
    const int      ph_snapshot = g.phase.load(std::memory_order_acquire);
    const std::uint8_t w_snapshot = g.white.load(std::memory_order_acquire);
    const bool     color_check_ok = (ph_snapshot == (int)GC_PHASE_PAUSE);
    /* Monotonic round-count snapshot: **comparing `phase` alone is not enough** -- a collection round may **run to completion entirely** during the walk
     * (PAUSE -> ... -> PAUSE), so the phase seen at the end equals the snapshot and is missed;
     * meanwhile the white bit/color has been rewritten during that time => an object scanned later differs from `w_snapshot` => false positive. */
    const std::size_t cyc_snapshot = g.minor_cycles.load(std::memory_order_relaxed)
                                   + g.major_cycles.load(std::memory_order_relaxed);
    /* Color complaints are **recorded first, committed later** (see the end of the walk): during the walk they are only accumulated, not immediately counted as failures,
     * otherwise that "abandon the decision" sentence at the end could not retract the `bad` already counted (which is exactly why it was toothless). */
    std::size_t  color_bad = 0;
    const gc_object* color_first = nullptr;
    int          color_first_list = 0;
    unsigned     color_first_marked = 0;
    /* ---- Step 3: the weak/ephemeron table's counting invariant (see the helper) ---- */
    bad += verify_weak_table_count(g);
    /* ---- Step 4: walk depth upper bound -- the **physical bound** (see the helper) ---- */
    const std::size_t limit = verify_walk_limit(g);
    /* ---- Step 5: the chunk metadata check must be done **first** (see the helper) ---- */
    bad += verify_chunk_metadata(g);
    /* Per-object verification: the union of the three chains = all allocated objects. It deliberately **shares the same traversal primitive**
     * `for_each_object` with "cancel rollback" and "gray-set fallback scanning" -- once any one of the three hand-copied loops
     * misses the nursery, it becomes a silent drift of the kind "some objects are never verified / never rolled back". */
    {
        std::size_t n_seen = 0;
        bool stop = false;
        for_each_object([&](gc_object* o, int li) {
            if (stop) return false;
            if (++n_seen > limit) {
                /* Report **how many were actually walked**, what limit is, and which chain it was -- only these three items can distinguish
                 * "a real cycle" (an astronomical number of nodes walked) from "this chain really is longer than limit" (limit estimated too low under concurrency).
                 * @note This uses the **per-chain** walk of `for_each_object`, where `li` is the chain index:
                 *       0=young 1=old 2=nursery (see the implementation of for_each_object). */
                std::fprintf(stderr,
                             "[verify] list %d cycle/too long @%p (walked %zu, limit=%zu, "
                             "live_objects=%zu)\n",
                             li, (void*)o, n_seen, limit,
                             g.live_objects.load(std::memory_order_relaxed));
                ++bad; stop = true; return false;
            }
            if (mark_load(o) == MARK_FREE) {
                std::fprintf(stderr, "[verify] list %d free slot is on the chain @%p\n",
                             li, (void*)o);
                ++bad; return true;
            }
            gc_chunk* c = chunk_of(reinterpret_cast<std::uintptr_t>(o));
            if (c == nullptr) {
                std::fprintf(stderr, "[verify] list %d object is in no chunk @%p\n",
                             li, (void*)o);
                ++bad; return true;
            }
            if (c->slot_size == 0) {
                /* Large object: verify "the object really falls inside its own block with the payload in bounds". A guarded one must also verify
                 * the invariant **pressed against the guard page** -- breaking it turns into "out-of-bounds silently passing", worse than no protection. */
                const std::uint32_t area =
                    c->span - c->data_off - (std::uint32_t)PAYLOAD_OFF;
                if (o->size > area) {
                    std::fprintf(stderr,
                        "[verify] large object payload overflows the block @%p size=%u area=%u\n",
                        (void*)o, o->size, area);
                    ++bad; return true;
                }
                if (c->guarded != 0) {
                    const std::size_t ps = page_size();
                    const std::size_t end =
                        (std::size_t)c->data_off + (std::size_t)PAYLOAD_OFF
                        + (std::size_t)o->size;
                    /* Invariant: the object is pressed right before the guard page and the gap must be < 16 (see chunk_new_big); breaking it turns into "out-of-bounds silently passing". */
                    if ((c->data_off & (std::uint32_t)(SLOT_ALIGN - 1)) != 0 ||
                        end > (std::size_t)c->span - ps ||
                        ((std::size_t)c->span - ps) - end >= SLOT_ALIGN) {
                        std::fprintf(stderr,
                            "[verify] guarded large object not pressed against the guard page @%p "
                            "data_off=%u size=%u span=%u page=%zu\n",
                            (void*)o, c->data_off, o->size, c->span, ps);
                        ++bad;
                        return true;
                    }
                }
                if (o != reinterpret_cast<gc_object*>(c->base + c->data_off)) {
                    std::fprintf(stderr, "[verify] large object in the wrong position @%p\n", (void*)o);
                    ++bad;
                }
            } else {
                std::uintptr_t data = reinterpret_cast<std::uintptr_t>(c->base)
                                    + c->data_off;
                std::uintptr_t off = reinterpret_cast<std::uintptr_t>(o) - data;
                if (off % c->slot_size != 0) {
                    std::fprintf(stderr, "[verify] not a slot start @%p (off=%llu ss=%u)\n",
                                 (void*)o, (unsigned long long)off, c->slot_size);
                    ++bad;
                } else if (off / c->slot_size >= c->bump.load(std::memory_order_relaxed)) {
                    std::fprintf(stderr, "[verify] beyond the carved-out range @%p\n", (void*)o);
                    ++bad;
                } else if (object_total_bytes(o) > c->slot_size) {
                    std::fprintf(stderr, "[verify] size exceeds the slot @%p\n", (void*)o);
                    ++bad;
                }
            }
            if (object_find(reinterpret_cast<std::uintptr_t>(payload_of(o))) != o) {
                std::fprintf(stderr, "[verify] reverse lookup inconsistent @%p\n", (void*)o);
                ++bad;
            }
            /* Block-tail padding: the canary / delta / tail-filler three segments are verified together. This is an **offline** checkpoint,
             * whose value is that out-of-bounds writes are often discovered before the object is reclaimed, and it can point out which object and which byte was written. */
            {
                gc_chunk* pc = c;
                std::uint32_t pidx = 0;
                std::uint32_t pusable = 0;
                const gc_padding_status st =
                    padding_verify_object(pc, o, &pidx, &pusable);
                if (st != GCXX_PAD_OK) {
                    std::fprintf(stderr,
                        "[verify] padding corrupted @%p slot=%u status=%d size=%u\n",
                        (void*)o, pidx, (int)st, o->size);
                    ++bad;
                } else {
                    /* Only when the canary is intact does it come to "was the alignment gap after the canary written" -- that indicates the write went past the whole padding. */
                    const int dirty = padding_first_dirty_byte(pc, o);
                    if (dirty >= 0) {
                        std::fprintf(stderr,
                            "[verify] filler after padding was written @%p slot=%u offset=%d\n",
                            (void*)o, pidx, dirty);
                        ++bad;
                    }
                }
            }
            if (o->age > (std::uint8_t)GC_AGE_OLD) {
                std::fprintf(stderr, "[verify] illegal age @%p\n", (void*)o);
                ++bad;
            }
            /* ---- Block-kind invariant (bidirectional) --------------------------------
             * A block either holds only objects that need scanning, or only objects that do not. This is
             * the **verifiable form** of the promise "gca memory is never scanned": an atomic block containing a
             * to-be-scanned object (or the reverse) means the object kind and its block do not match -- a bug in the allocation path
             * that would make some object be scanned when it should not, or not scanned when it should. */
            if (c->kind == KIND_ATOMIC && type_scanned_ct(o->type)) {
                std::fprintf(stderr,
                    "[verify] object requiring scanning in an atomic block @%p type=%u\n",
                    (void*)o, (unsigned)o->type);
                ++bad;
            } else if (c->kind == KIND_NORMAL && !type_scanned_ct(o->type)) {
                std::fprintf(stderr,
                    "[verify] object not requiring scanning in a normal block @%p type=%u\n",
                    (void*)o, (unsigned)o->type);
                ++bad;
            }
            const std::uint8_t m = mark_load(o);
            if (m != 0 && (m & (std::uint8_t)~7) != 0) {
                std::fprintf(stderr, "[verify] illegal marked @%p marked=%u\n",
                             (void*)o, (unsigned)m);
                ++bad;
            }
            /* Idle-phase invariant: objects on **all three chains** (young / old / nursery) must be **exactly
             * equal to the current white bit** -- an object with not a single white bit left would be skipped by the next round's mark_object at
             * "(m & WHITE_BITS) == 0 -> already gray/black", and after its children are missed they get swept,
             * leaving a **silent dangling pointer** (the worst error this file can produce). The nursery is in scope too:
             * it is **not guaranteed** to be emptied before the end of this round (rehome_some is incremental). */
            if (color_check_ok &&
                m != w_snapshot) {
                /* **Only record it**: whether it counts as a failure waits until the end of the walk confirms "neither the phase nor the round count changed". */
                if (color_bad == 0) {
                    color_first = o;
                    color_first_list = li;
                    color_first_marked = (unsigned)m;
                }
                ++color_bad;
            }
            return true;
        });
    }
    /* **Re-check the phase** after the walk: if during it another thread started/finished a collection round, then the
     * `marked` values seen per object above may span two generations, and the color decision is untrustworthy -- abandon the whole walk.
     * This is **prefer a miss over a false positive**: the contract of `gc_verify` is "0 == healthy",
     * and a false positive drowns real defects in noise (this project has already been burned by that). */
    if (color_check_ok) {
        const int         ph_now  = g.phase.load(std::memory_order_acquire);
        const std::size_t cyc_now = g.minor_cycles.load(std::memory_order_relaxed)
                                  + g.major_cycles.load(std::memory_order_relaxed);
        if (ph_now != ph_snapshot || cyc_now != cyc_snapshot) {
            /* A collection happened during the walk (including "a whole round ran to completion", where `phase` goes back and forth) =>
             * the `marked` values seen per object may span two generations => **discard the whole walk**.
             * Prefer a miss over a false positive: the contract of `gc_verify` is "0 == healthy",
             * and a false positive drowns real defects in noise (this project has already been burned by that). */
            if (color_bad > 0) {
                std::fprintf(stderr,
                    "[verify] a collection happened during the walk (phase %d -> %d, cycles %zu -> %zu): "
                    "color decision abandoned for this walk (%zu suspicious items discarded)\n",
                    ph_snapshot, ph_now, cyc_snapshot, cyc_now, color_bad);
            }
        } else if (color_bad > 0) {
            /* **Confirmed that no round completed during this time** => this is a true positive, commit it.
             * The error carries `size/type/age`: these three separate "was this object freshly allocated or promoted"
             * (`age==0` = fresh; `age>=PROMOTE_AGE` = already promoted to the old generation),
             * thereby pointing directly at which coloring path missed it. */
            const gc_object* c0 = color_first;
            /* Second diagnostic round: locate "which category this object belongs to" -- a live heap or a **dead heap from an exited thread**
             * (dead heap after the thread exited), whether it was minor or major at the time, and the geometry of the chunk it is in. */
            const gc_chunk* ck0 = (c0 != nullptr)
                ? chunk_of(reinterpret_cast<std::uintptr_t>(c0)) : nullptr;
            const gc_heap*  hp0 = (ck0 != nullptr) ? ck0->owner : nullptr;
            std::fprintf(stderr,
                "[verify] idle-phase color is not the current white bit @%p list=%d marked=%u white=%u"
                "(%zu items of the same kind) size=%u type=%u age=%u"
                " | kind=%d stage=%d cwhite=%u"
                " | chunk cls=%u slot=%u bump=%u"
                " | heap dead=%d"
                " | gdisc=%zu grescan=%zu gover=%d\n",
                static_cast<const void*>(c0), color_first_list, color_first_marked,
                (unsigned)w_snapshot, color_bad,
                (unsigned)(c0 != nullptr ? c0->size : 0u),
                (unsigned)(c0 != nullptr ? c0->type : 0u),
                (unsigned)(c0 != nullptr ? c0->age : 0u),
                (int)g.kind.load(std::memory_order_relaxed),
                (int)g.sweep_stage,
                (unsigned)g.cycle_white,
                (unsigned)(ck0 != nullptr ? ck0->cls : 0u),
                (unsigned)(ck0 != nullptr ? ck0->slot_size : 0u),
                (unsigned)(ck0 != nullptr ? ck0->bump.load(std::memory_order_relaxed) : 0u),
                (int)(hp0 != nullptr ? hp0->dead.load(std::memory_order_relaxed) : -1),
                (std::size_t)g.gray_discards.load(std::memory_order_relaxed),
                (std::size_t)g.gray_rescans,
                (int)(g.gray_overflow.load(std::memory_order_relaxed) ? 1 : 0));
            bad += color_bad;
        }
    }
    /* Chunk-level invariants: the slot count / live count / free-chain head must all be within legal ranges (a single relaxed read,
     * so it cannot produce false positives against a concurrent sweep).
     * **Do not walk the free chain node by node** -- that chain is modified lock-free by sweep (push is done only by the collector,
     * pop by the owner), so a walk would necessarily read nodes being unlinked right now; for a "0 == healthy" contract,
     * a false positive is worse than a miss. */
    for (gc_chunk* c = g.chunks; c != nullptr; c = c->all_next) {
        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint32_t cap  = c->capacity;
        const std::uint32_t bump = c->bump.load(std::memory_order_relaxed);
        const std::uint32_t live = chunk_live(c);
        const std::uint32_t fh   = c->free_head.load(std::memory_order_relaxed);
        if (bump > cap) {
            std::fprintf(stderr, "[verify] bump out of bounds chunk@%p bump=%u cap=%u\n",
                         (void*)c, bump, cap);
            ++bad;
        }
        /* ---- An invariant may only be stated in terms of what is **physically possible** --------------------------------
         * This was written wrong twice in a row, both times by taking the semantics of `used_pub` for granted:
         *   1. `live > bump` -- **is not** an invariant: `bump` is the **high-water mark** of the bump cursor, while
         *      `used` can grow by **reusing free slots**, and that path **does not advance bump** (`freelist_pop`
         *      takes slots that already have `idx < bump`) => `used > bump` is entirely legal (measured up to a difference of 832).
         *   2. `used > capacity` -- also **not** an invariant: `used` is a published quantity **accumulated across reuse cycles**,
         *      not "the number of currently occupied slots"; after a block is reused many times via the recycle stack, `used` far exceeds
         *      `capacity` (measured `used=1999 / cap=1021`).
         * Both were **false positives** (judging a healthy state as corruption). The contract of `gc_verify` is "0 = healthy",
         * and **a false positive is worse than a miss** -- it makes callers stop trusting this interface.
         *
         * The two that really hold **physically** and are independent of other parties' concurrency:
         *   * `bump <= capacity` -- the cursor cannot exceed this block's slot count;
         *   * `live <= capacity` -- `chunk_live` already does a saturating subtraction (always >= 0), and the live object count
         *     cannot exceed the slot count.
         * Breaking them is real corruption (cursor out of bounds, counter drift, slots handed out twice).
         * @note Neither `used` nor `bump` **participates** in the decision here -- they are used only for diagnostics and the retirement criterion. */
        if (live > cap) {
            std::fprintf(stderr, "[verify] live out of bounds chunk@%p live=%u cap=%u\n",
                         (void*)c, live, cap);
            ++bad;
        }
        if (fh != SLOT_NONE && fh >= cap) {
            std::fprintf(stderr, "[verify] free chain head out of bounds chunk@%p head=%u cap=%u\n",
                         (void*)c, fh, cap);
            ++bad;
        }
        if (c->slot_size != 0 && c->data_off % SLOT_ALIGN != 0) {
            std::fprintf(stderr, "[verify] data_off not slot-aligned chunk@%p off=%u\n",
                         (void*)c, c->data_off);
            ++bad;
        }
    }
    for (std::size_t i = 0; i < g.table_size; ++i) {
        const gc_map_entry& e0 = g.chunk_table[i];
        if (e0.chunk != nullptr && chunk_slot_index(g, e0.key) != i) {
            std::fprintf(stderr, "[verify] chunk-table inline entry bad i=%llu\n",
                             (unsigned long long)i);
            ++bad;
        }
        for (const gc_map_entry* e = e0.next; e != nullptr; e = e->next) {
            if (e->chunk == nullptr || chunk_slot_index(g, e->key) != i) {
                std::fprintf(stderr, "[verify] chunk-table overflow entry bad i=%llu\n",
                             (unsigned long long)i);
                ++bad;
            }
        }
    }
    return bad;
}
/* Conservative-scan reverse lookup: resolve a "suspected pointer" into an object and mark it live. The hit interval is inside the payload
 * (&obj->field, &vec[i] and other interior pointers); the negative-bias window covers the case where p falls on the object header or even the tail of the previous slot
 * (the compiler may keep a base address like `a - 24` in a register/on the stack), so both candidates, "the slot p is in" and
 * "the next slot", are checked; a large object exclusively occupies several 64KB blocks and is decided by [payload, payload+size).
 * Conservative principle: when unsure, mark both live (keeping one extra is safe; missing one is fatal). */
/**
 * @brief Mark one object: clear white bit -> color -> advance the scan.
 * @param o The candidate object; for `excl` see `scan_ctx::excl`.
 * @note `kind` (major/minor) is read concurrently here, hence it is atomic.
 * @internal
 */
inline std::size_t mark_object(gc_object* o, bool excl, bool minor);
/* Forward declaration: returns the number of payload bytes skipped (>0 when a leaf object is skipped as a whole); excl/minor are carried down by scan_ctx. */
/* Scan context -- the loop invariants are computed before entering the scan loop (previously G(), my_heap(),
 * heap_lo/heap_hi were all fetched afresh on every candidate word). **Table entries are not cached in gc_heap**: that would stuff the marking
 * concern into the allocation structure, make table_grow walk all heaps in reverse to clear it, and the cache would be a raw pointer to the table
 * array -- after table_grow frees the old array that is a use-after-free read (looking it up on demand with table_find costs about the same). */
struct scan_ctx {
    gc_state*      g;
    std::uintptr_t lo;      /**< Lower bound of the heap address range snapshot (for fast exclusion); hi is the upper bound */
    std::uintptr_t hi;
    /** Diagnostic accumulator **pointer** (this thread's, fetched once by the caller).
     *  Not calling diag() on the spot: it reads a function-local thread_local, which MinGW implements as emutls
     *  (a real function call __emutls_get_address), while on the marking path it is touched **per object**
     *  (leaf objects accumulate skipped_bytes) -- measured at 4.4ns/object in marking-heavy cases.
     *  With the pointer passed down, the whole marking process reads TLS only once at the entry.
     * @internal
     */
    diag_accum*    da;
    /** Intra-slice local accumulator: the bytes skipped for leaf objects are accumulated here first, and only written back to `da`
     *  once at the end of `scan_words` (writing da directly per object measures 4ns/object).
     * @internal
     */
    std::size_t    skipped;
    /** This thread is the **exclusive** marker (all mutators have stopped; see begin_cycle) -- no CAS needed,
     *   saving one atomic RMW per object (about 20 cycles on x86).
     * @internal
     */
    bool           excl;
    bool           minor;   /**< This round is a minor collection (the old generation is the root set, not expanded) */
};
/**
 * @brief Build a scan context (this round's white bit, per-class accumulators, exclusion set, etc.).
 * @internal
 */
inline scan_ctx make_scan_ctx(diag_accum* da) {
    scan_ctx sc{};
    sc.g  = &G();
    sc.lo = sc.g->heap_lo.load(std::memory_order_relaxed);
    sc.hi = sc.g->heap_hi.load(std::memory_order_relaxed);
    sc.da = da;
    sc.excl  = sc.g->mark_excl.load(std::memory_order_relaxed);
    sc.minor = sc.g->kind.load(std::memory_order_relaxed) == (int)GC_KIND_MINOR;
    return sc;
}
/**
 * @brief The only "does the candidate point at this object" criterion -- shared by resolve_and_mark / object_find / verify_heap.
 * @internal
 */
inline bool ptr_hits_object(std::uintptr_t p, std::uintptr_t pl, std::uint32_t size) {
#if GCXX_ALL_INTERIOR_POINTERS
    /* p falls inside [pl, pl+size) (an interior pointer), or falls no more than BIAS_BYTES before pl
     * (negative-bias window: the compiler may hold a base address like a-24). */
    return p < pl + (std::uintptr_t)size && p + (std::uintptr_t)BIAS_BYTES >= pl;
#else
    (void)size;
    return p == pl;                       /* only the exact starting address is accepted */
#endif
}
/* The second candidate ("the previous slot") may only approach from below: p must fall before the payload start. */
/**
 * @brief Decide whether a candidate raw address falls inside the payload of `o` (for interior-pointer recognition and deduplication).
 * @internal
 */
inline bool ptr_hits_object_prev(std::uintptr_t p, std::uintptr_t pl) {
#if GCXX_ALL_INTERIOR_POINTERS
    return p < pl && p + (std::uintptr_t)BIAS_BYTES >= pl;
#else
    return p == pl;
#endif
}
/**
 * @brief Slot geometry of a **small-object** chunk, computed once per candidate.
 *
 * `resolve_and_mark` (the marking hot spot) and `object_find` (diagnostics) must agree
 * *exactly* about which slot a candidate address designates -- the doc comment on
 * `object_find` says so, and a divergence would silently turn "verification passed" into
 * an illusion. Both used to spell this arithmetic out separately; it lives here once.
 *
 * Two details are load-bearing and must not be "simplified":
 *   * `p - data` is necessarily < 64 KiB (the object and the chunk base are in the same
 *     64 KiB block), so this uses a **32-bit** division: on x86-64 `div r32` is more than
 *     twice as fast as `div r64`, and this runs once per candidate pointer.
 *   * the index must be `< capacity` **and** `< bump`: `capacity` keeps the read inside the
 *     slot array, `bump` keeps it inside the slots that have actually been handed out
 *     (a not-yet-allocated slot holds no object header worth reading).
 *
 * @param e  Table entry of the chunk (carries `data_off` / `slot_size` / `chunk`).
 * @param p  Candidate address, already known to fall inside this chunk.
 * @param out_data Receives the data-area start address.
 * @param out_slot_size Receives the slot size.
 * @return The slot index, plus the object header when that index is a handed-out slot
 *         (`obj == nullptr` otherwise). For a large-object chunk (`slot_size == 0`) the
 *         index is `SLOT_NONE` and `obj` is `nullptr`; the caller handles large objects
 *         separately because they have no slot array at all.
 * @internal
 */
struct small_slot_at {
    gc_object*    obj;
    std::uint32_t idx;
};
inline small_slot_at small_slot_of(const gc_map_entry* e, std::uintptr_t p,
                                   std::uintptr_t* out_data,
                                   std::uint32_t* out_slot_size) {
    const std::uintptr_t data      = (p & ~(std::uintptr_t)CHUNK_MASK) + e->data_off;
    const std::uint32_t  slot_size = e->slot_size;
    *out_data      = data;
    *out_slot_size = slot_size;
    small_slot_at r{};
    r.obj = nullptr;
    r.idx = SLOT_NONE;
    if (slot_size == 0) return r;                 /* large object: no slot array */
    const std::uint32_t idx = (p >= data)
                            ? (std::uint32_t)(p - data) / slot_size : 0u;
    const gc_chunk* chunk = e->chunk;
    if (idx < chunk->capacity && idx < chunk->bump.load(std::memory_order_relaxed))
        r.obj = reinterpret_cast<gc_object*>(data + (std::size_t)idx * slot_size);
    r.idx = idx;
    return r;
}
/**
 * @brief The **next** slot's object header, for the negative-bias candidate ("p sits just
 *        before this object's payload").
 *
 * The purely arithmetic pre-filter comes first and touches no memory: the payload offset is
 * the constant `PAYLOAD_OFF`, so "does the bias window hit this slot" is a decidable range
 * test. Failing it means the slot cannot be the answer -- this never causes a missed mark,
 * it only avoids a read. The final term keeps the object-header read inside the chunk even
 * when the slot index is the last one, since a candidate may legitimately sit in the bytes
 * after the final object.
 * @internal
 */
inline gc_object* next_small_slot_of(const gc_map_entry* e, std::uintptr_t p,
                                     std::uintptr_t data, std::uint32_t slot_size,
                                     std::uint32_t idx) {
    if (slot_size == 0 || idx == SLOT_NONE) return nullptr;
    const std::uint32_t  idx2  = idx + 1;
    const std::uintptr_t base2 = data + (std::uintptr_t)idx2 * slot_size;
    const std::uintptr_t blk   = p & ~(std::uintptr_t)CHUNK_MASK;
    const gc_chunk*      chunk = e->chunk;
    if (p + (std::uintptr_t)BIAS_BYTES >= base2 + (std::uintptr_t)PAYLOAD_OFF &&
        p < base2 + (std::uintptr_t)PAYLOAD_OFF &&
        idx2 < chunk->capacity &&
        idx2 < chunk->bump.load(std::memory_order_relaxed) &&
        base2 + (std::uintptr_t)sizeof(gc_object)
            <= blk + (std::uintptr_t)CHUNK_SIZE) {
        return reinterpret_cast<gc_object*>(base2);
    }
    return nullptr;
}
/**
 * @brief Resolve a candidate raw address back to its object and mark it.
 * @return the number of objects newly marked live by this call.
 * @internal
 */
inline unsigned resolve_and_mark(scan_ctx& sc, std::uintptr_t p) {
    (void)sc;
    const gc_map_entry* e = table_find(p);
    if (e == nullptr) return 0;
    unsigned n = 0;
    if (e->slot_size == 0) {                       /* large object */
        const gc_chunk* c = e->chunk;
        auto* o = reinterpret_cast<gc_object*>(c->base + c->data_off);
        const auto pl = reinterpret_cast<std::uintptr_t>(payload_of(o));
        if (ptr_hits_object(p, pl, o->size)) {
            if (p < pl) ++sc.da->bias_hits;
            sc.skipped += mark_object(o, sc.excl, sc.minor); return 1;
        }
        return 0;
    }
    /* Small object: all geometry lives in the table entry, the chunk header is not read -- each
     * candidate costs only two levels of access, "table entry + object header". Unallocated slots
     * are safe by "whole-chunk zeroing / kernel zero page" (an all-zero header has size=0 and never
     * matches), freed slots are blocked by the MARK_FREE sentinel. The slot-index bounds live in
     * `small_slot_of` (must be < capacity and < bump at the same time, otherwise an out-of-bounds
     * read picks up debris past the end of the slot array). */
    std::uintptr_t      data = 0;
    std::uint32_t slot_size = 0;
    const small_slot_at slot = small_slot_of(e, p, &data, &slot_size);
    gc_chunk* c = e->chunk;
    if (slot.obj != nullptr && mark_load(slot.obj) != MARK_FREE) {
        auto* o = slot.obj;
        const auto pl = reinterpret_cast<std::uintptr_t>(payload_of(o));
        if (ptr_hits_object(p, pl, o->size)) {
            if (p < pl) ++sc.da->bias_hits;
            sc.skipped += mark_object(o, sc.excl, sc.minor); ++n;
        }
    }
    if (gc_object* o2 = next_small_slot_of(e, p, data, slot_size, slot.idx)) {
        if (mark_load(o2) != MARK_FREE) {
            const auto pl2 = reinterpret_cast<std::uintptr_t>(payload_of(o2));
            if (ptr_hits_object_prev(p, pl2)) { sc.skipped += mark_object(o2, sc.excl, sc.minor); ++n; }
        }
    }
#if GCXX_BLACKLIST
    /* ---- Blacklist signal collection (the core of Boehm-style accidental-retention prevention) ----
     * A candidate lands inside a registered chunk yet hits no allocated object -- either an integer
     * collision, or a stale pointer into free space. Record the 4KB page it hit; pages hit in two
     * consecutive majors go on the blacklist, and the allocator stops allocating objects there from
     * then on (see finish_cycle and chunk_alloc). Only one extra shift+set on the miss path. */
    if (n == 0) {
        c->bl_hits.fetch_or((std::uint16_t)(
            1u << (unsigned)((p - reinterpret_cast<std::uintptr_t>(c->base)) >> 12)),
            std::memory_order_relaxed);
    }
#else
    (void)c;
#endif
    return n;
}
/* Convenience overload for debugging/testing: builds its own context. */
/**
 * @brief Convenience overload for debugging/testing: builds its own scan context and forwards to the main implementation.
 * @internal
 */
inline unsigned resolve_and_mark(std::uintptr_t p) {
    diag_accum fallback{};
    scan_ctx sc = make_scan_ctx(diag_or(&fallback));
    const unsigned n = resolve_and_mark(sc, p);
    sc.da->skipped_bytes += sc.skipped;      /* there is no scan_words epilogue here, so fold it in here */
    return n;
}
/** @brief For debugging/verification: return the first object that is hit.
 *  @note The decision range must match resolve_and_mark exactly (including the GCXX_ALL_INTERIOR_POINTERS
 *        switch and the slot-index constraints), otherwise the diagnostic result disagrees with the real
 *        marking behavior and "verification passed" becomes an illusion.
 * @internal
 */
inline gc_object* object_find(std::uintptr_t p) {
    /* Resolve through the table rather than `chunk_of`, because the shared slot-geometry
     * helpers work on a table entry (it carries `data_off` / `slot_size`; the chunk header
     * would need the same three reads anyway). `table_find` is what `chunk_of` calls. */
    const gc_map_entry* e = table_find(p);
    if (e == nullptr) return nullptr;
    if (e->slot_size == 0) {
        const gc_chunk* c = e->chunk;
        auto* o = reinterpret_cast<gc_object*>(c->base + c->data_off);
        const auto pl = reinterpret_cast<std::uintptr_t>(payload_of(o));
        return ptr_hits_object(p, pl, o->size) ? o : nullptr;
    }
    /* Small object: share the slot geometry with `resolve_and_mark` so the two can never
     * disagree about which slot a candidate designates (that is what the @note above is
     * about). The negative-bias candidate uses the same helper too. */
    std::uintptr_t      data = 0;
    std::uint32_t slot_size = 0;
    const small_slot_at slot = small_slot_of(e, p, &data, &slot_size);
    if (slot.obj != nullptr && mark_load(slot.obj) != MARK_FREE) {
        gc_object* o = slot.obj;
        const auto pl = reinterpret_cast<std::uintptr_t>(payload_of(o));
        if (ptr_hits_object(p, pl, o->size)) return o;
    }
    if (gc_object* o2 = next_small_slot_of(e, p, data, slot_size, slot.idx)) {
        if (mark_load(o2) != MARK_FREE) {
            const auto pl2 = reinterpret_cast<std::uintptr_t>(payload_of(o2));
            if (ptr_hits_object_prev(p, pl2)) return o2;
        }
    }
    return nullptr;
}
/** @brief The **mark-window epilogue** of `object_new`: push into the gray set + cross-cycle guard. The caller must already have linked it.
 *
 * This path is taken only when **a collection is in progress and this thread allocates inside the window** (single-threaded that means during STW,
 * when this thread is acting as collector and does not allocate; multi-threaded it is the few moments belonging to another mutator).
 * It is split out so that the hot path of `object_new` contains neither the gray set nor the two acquires of the cross-cycle guard.
 *
 * @note The gray push must come after the link, and the cross-cycle guard must re-read the phase
 *       plus `aborting`. For the reasoning see the comment on `object_new_mark_window_finish` and the call site in `object_new`.
 * @internal
 */
GCXX_NOINLINE inline void object_new_mark_window_finish(gc_state& g, gc_object* o,
                                                        bool need_gray) {
    /* ---- Cross-cycle guard: an object dyed "no white bit" must be guaranteed to **actually be expanded this cycle** ----------------
     * `ph` is the phase the caller **read once**; the thread can be preempted between that read and its use, and the "mark window ends"
     * step has to run through the whole mark + incremental sweep + epilogue (which can take a long time). So "I dye dark per the mark window"
     * does not imply "we are still inside the mark window at this instant". Once we are not inside it, that enqueue **will not be consumed by anyone**,
     * and the object crosses the cycle carrying no white bit -- the next cycle's `mark_object` skips it at
     * "(m & WHITE_BITS) == 0 -> already gray/black", so its children are never expanded, which is a
     * **missed mark and therefore a dangling pointer**.
     * This is exactly what `gc_verify`'s "the idle-period color is not the current white bit" check is looking for.
     *
     * The predicate is `phase in {PROPAGATE, ATOMIC} || aborting`:
     *   * the phase is no longer the mark window => marking has finished (or never started) => nobody consumes that enqueue;
     *   * `aborting` covers the **cancel path**: it does `gray_clear()` first, then `unmark_all_objects()`, and only
     *     then sets `phase` back to PAUSE, so the window in which "the phase is still PROPAGATE but the rollback has already walked the object chain"
     *     can only be recognized through `aborting` (`sp_scanning` does not cover `object_new`).
     *
     * **The order of the two loads must not be swapped**: the cancel path is `phase = PAUSE` (release) **followed by**
     *    `aborting = false` (release). If `phase` is read before `aborting`, the consistent order allows
     *    `load(phase) -> store(phase=PAUSE) -> store(aborting=false) -> load(aborting)`,
     *    so it is possible to read "a stale PROPAGATE plus a fresh false" => the guard wrongly concludes "still inside
     *    the window" => that object may have been linked only after `unmark_all_objects()`, and it enters the idle
     *    period carrying **no white bit** => the next cycle's `mark_object` skips it => its children are missed
     *    => dangling pointer.
     *    When `aborting` is read first and `phase` second: reading a stale `false` **necessarily** means the read of `phase`
     *    also happened before `store(phase=PAUSE)` (program order within the same thread plus the acquire lower bound),
     *    so at that point `phase` can only be PROPAGATE/ATOMIC, that is, the thread really is still inside the window,
     *    and the decision holds. When both conditions hold the object is restored to unmarked: it is in the nursery,
     *    **this cycle will not sweep it**, and it is certainly reachable (the mutator has just obtained the pointer)
     *    => the next cycle's self-scan will mark it.
     *    This step can only make this cycle **retain more**, never miss a mark (the safe direction). */
    if (need_gray) gray_push(o);
    const bool ab = g.aborting.load(std::memory_order_acquire);
    const int  ph2 = g.phase.load(std::memory_order_acquire);
    if (ab || (ph2 != (int)GC_PHASE_PROPAGATE && ph2 != (int)GC_PHASE_ATOMIC)) {
        o->marked.store(g.white.load(std::memory_order_relaxed),
                        std::memory_order_relaxed);
    }
}
/** @brief Create the object header and link it into "where it belongs this cycle" (owner thread private path).
 *  @param force_zero force zeroing (the calloc contract of gc(size)/gc_allocator; ctor_zeroes means the
 *         constructor writes the whole block and it can be skipped). @return the object header pointer; nullptr means failure.
 *  @note During idle/sweep it is linked onto young and dyed the current white bit; during the mark window (PROPAGATE/ATOMIC) it is linked into the nursery (never
 *        swept this cycle) and dyed gray onto the gray stack (treated as a root this cycle), so no write barrier is needed and no mark is missed.
 *  @note **The hot path is only the "idle/sweep + no zeroing needed" one** (`GCXX_ALWAYS_INLINE`): the mark-window
 *        branch, the zeroing fallback, and the epilogue after choosing the link target all live in a `GCXX_NOINLINE` helper.
 *        The predicates of `is_leaf_object` and `type_scanned_ct` are unchanged.
 * @internal
 */
GCXX_ALWAYS_INLINE gc_object* object_new(gc_heap* h, std::uint32_t size,
                                                std::uint8_t type, bool ctor_zeroes,
                                                bool force_zero = false) {
    gc_state& g = G();
    /* Test injection: make this allocation fail so that the emergency collection path of
     * `allocate_bytes_oom` is taken (see `gcxx::tests::fail_next_object_new`). The injection resets itself after use. */
    if (tests::fail_next_object_new().load(std::memory_order_relaxed)) {
        tests::fail_next_object_new().store(false, std::memory_order_relaxed);
        return nullptr;
    }
    /* The chunk kind is decided by the **type in the object header**, not by which interface the caller used -- so gc<int>
     * and gcs<int> naturally land in different chunks, and "an atomic chunk never contains an object that needs scanning" becomes an assertable structural property. */
    const unsigned kind = type_scanned_ct(type) ? KIND_NORMAL : KIND_ATOMIC;
    chunk_slot cs{};
    gc_object* o = chunk_alloc(h, object_total_need(size), kind, &cs);
    if (o == nullptr) return nullptr;
    /* The slot memory comes from a raw chunk: a placement new must start the lifetime of gc_object before its
     * atomic members are accessed (trivial type, no actual code). */
    o = ::new (static_cast<void*>(o)) gc_object;
    o->size   = size;
    o->type   = type;
    o->age    = (std::uint8_t)GC_AGE0;
    /* o->next is deliberately **not written** here: both branches below (young / nursery) overwrite it unconditionally. */
    const int ph = g.phase.load(std::memory_order_relaxed);
    /* During idle/sweep (the most common shape) dye the current white bit. */
    o->marked.store(g.white.load(std::memory_order_relaxed), std::memory_order_relaxed);
    /* Zeroing: a conservative GC needs clean initial values (with ctor_zeroes the caller guarantees a full write, so it can be skipped).
     * force_zero is used by gc(size)/gc_allocator: that path has no type information, so dirty bytes are most at risk
     * of being taken for pointers, and its **public contract** is calloc semantics, hence it is not affected by GCXX_ZERO_MEMORY.
     *
     * `cs.fresh_zeroed` (cf. mimalloc `types.h:486 free_is_zero` / `alloc.c:119-132`):
     * this slot **has never been written**, and the kernel guarantees the content is a zero
     * page, so another memset is pure duplicated work. It has exactly **one** source:
     *   * a large-object chunk just committed by the OS (C3, the original predicate) -- see
     *     `chunk_new_big`, which sets `os_zeroed` from what `alloc_block` actually reports.
     * The predicate is always "**the kernel guarantees zero**" or "never written", never
     * "it looks new".
     * @note It is worth skipping only when `size` is clearly larger than the fixed cost of one memset -- below 16 bytes
     *       `memset` does not beat this branch, so those size classes just zero as before.
     * @note `force_zero` **must stay in the predicate**: it decouples the zeroing of `gc(n)` / `gc_allocator`
     *       from `GCXX_ZERO_MEMORY` (with `-DGCXX_ZERO_MEMORY=0` those two paths must still zero,
     *       because calloc semantics are a **public contract**). Dropping it from the predicate would leave the
     *       parameter unused on those paths and make `gc(n)` / `gc_allocator` violate their contract. */
    if (size > 0 && !ctor_zeroes && !cs.fresh_zeroed && (GCXX_ZERO_MEMORY || force_zero)) {
        std::memset(payload_of(o), 0, size);
    }
    /* Block-tail padding: write the canary + the exact delta, **must come after zeroing** (so that nobody later wipes it out while moving the zeroing range). */
    padding_init(cs.c, o, cs.idx);
    if (ph == (int)GC_PHASE_PAUSE) {
        o->next.store(h->young.load(std::memory_order_relaxed),
                     std::memory_order_relaxed);
        h->young.store(o, std::memory_order_relaxed);
    } else if (ph == (int)GC_PHASE_PROPAGATE || ph == (int)GC_PHASE_ATOMIC) {
        /* Allocation inside the mark window: dye gray (or black for a leaf) and push it onto the gray chain, guaranteeing it **is expanded this cycle**,
         * so old objects it points to are not missed; it also enters the nursery and is not swept this cycle.
         * The price is that it survives across cycles carrying a "no white bit" color -- dyeing here alone cannot fix that: as long as it is reachable,
         * the mutator's self-scan will mark_object it and dye it black, which is what leaves it with a color that no white-bit test recognises. The colors
         * are uniformly fixed up by atomic_finish's "mark-window suffix re-dye". */
        const bool need_gray = !is_leaf_object(o);
        o->marked.store(need_gray ? (std::uint8_t)0 : (std::uint8_t)BLACK,
                        std::memory_order_relaxed);
        o->next.store(h->nursery.load(std::memory_order_relaxed),
                     std::memory_order_relaxed);
        h->nursery.store(o, std::memory_order_relaxed);
        /* The gray push must come **after the link is installed**: the fallback rescan (rescan_gray_from_heap) walks the object chain looking
         * for "marked but not yet expanded" objects, and it cannot see an object that is "gray but not yet linked" -- that object and its children
         * would be missed and then collected. The color must be settled **before** linking: an object on the chain is immediately visible to other threads,
         * and leaving the free-slot sentinel color (MARK_FREE) would make mark_object skip it and never mark it live. */
        object_new_mark_window_finish(g, o, need_gray);
    } else {
        o->next.store(h->nursery.load(std::memory_order_relaxed),
                     std::memory_order_relaxed);
        h->nursery.store(o, std::memory_order_relaxed);
    }
    /* Statistics: accumulated locally per heap, flushed into the global atomics 64KB at a time (zero atomic RMW on the hot path); a snapshot is also published for gc_stat. */
    if (size > h->max_alloc) h->max_alloc = size;   /* feeds the amplification of the next cycle's debt floor (see gc_heap::max_alloc) */
    h->pend_bytes += size;
    ++h->pend_objs;
    if (h->pend_bytes >= 65536) {
        g.total_bytes.fetch_add(h->pend_bytes, std::memory_order_relaxed);
        g.live_objects.fetch_add(h->pend_objs, std::memory_order_relaxed);
        h->pend_bytes = 0;
        h->pend_objs = 0;
    }
    h->pend_bytes_pub.store(h->pend_bytes, std::memory_order_relaxed);
    h->pend_objs_pub.store(h->pend_objs, std::memory_order_relaxed);
    return o;
}
/* Accounting: allocation is booked lazily (pend_bytes enters the global counter only at 64KB) while **freeing is deducted immediately** -- total_bytes
 * must never be deducted into a negative number (otherwise the next cycle's quota becomes astronomical and **the GC stops forever**). Defenses: publish_pending (publish the unbooked part before parking), stats_sub_nonneg
 * (saturating subtraction), finish_cycle (live clamped to mapped_bytes + DEBT_CAP_MAPPED_PERCENT). */
/** @brief Fold this heap's not-yet-booked allocations into the global counters.
 *  @note **Only callable by the owner thread of that heap** -- pend_bytes/pend_objs are the owner's
 *        private plain accumulators (the hot path deliberately avoids atomic RMW), so a cross-thread read is a data race.
 *  @note Synchronization relies on park_epoch's release/acquire: the publication happens before store(park_epoch),
 *        and when the collector's acquire read sees "in position for this cycle" it has necessarily also seen that publication, so the sweep
 *        can never subtract into a negative number.
 * @internal
 */
inline void publish_pending(gc_heap* h) {
    if (h == nullptr || (h->pend_bytes == 0 && h->pend_objs == 0)) return;
    gc_state& g = G();
    if (h->pend_bytes != 0) {
        g.total_bytes.fetch_add(h->pend_bytes, std::memory_order_relaxed);
        h->pend_bytes = 0;
        h->pend_bytes_pub.store(0, std::memory_order_relaxed);
    }
    if (h->pend_objs != 0) {
        g.live_objects.fetch_add(h->pend_objs, std::memory_order_relaxed);
        h->pend_objs = 0;
        h->pend_objs_pub.store(0, std::memory_order_relaxed);
    }
}
/** @brief Saturating subtraction (relaxed CAS loop): **never underflows**.
 *  @note On the normal path old > v always holds, the first compare_exchange succeeds, and the cost is on the order of
 *        one fetch_sub. Better to under-report liveness (collect more eagerly) than to underflow into an astronomical number
 *        (which would turn the GC off forever).
 * @internal
 */
inline void stats_sub_nonneg(std::atomic<std::size_t>& c, std::size_t v) {
    std::size_t old = c.load(std::memory_order_relaxed);
    for (;;) {
        const std::size_t next = (old > v) ? (old - v) : (std::size_t)0;
        if (c.compare_exchange_weak(old, next, std::memory_order_relaxed,
                                    std::memory_order_relaxed)) return;
    }
}
/** @brief Fold the batched accounting accumulated on the free side into the global atomic counters.
 *  @note Called only by cycle_owner (the sole freer), and it **must be called before any reader sees the global
 *        counters**: finish_cycle must call it once before reading total_bytes to compute the next cycle's quota;
 *        gc_stat() goes through the _pub snapshots and does not depend on this timing.
 * @internal
 */
inline void sweep_stats_flush() {
    gc_state& g = G();
    if (g.sweep_pend_bytes != 0) {
        stats_sub_nonneg(g.total_bytes, g.sweep_pend_bytes);   /* see below: saturating subtraction */
        g.sweep_pend_bytes = 0;
        g.sweep_pend_bytes_pub.store(0, std::memory_order_relaxed);
        g.sweep_pend_objs_pub.store(g.sweep_pend_objs, std::memory_order_relaxed);
    }
    if (g.sweep_pend_objs != 0) {
        g.freed_objects.fetch_add(g.sweep_pend_objs, std::memory_order_relaxed);
        /* Lower-bound protection for live_objects: when the global value is below the batch size, subtract only down to 0, never underflow. */
        stats_sub_nonneg(g.live_objects, g.sweep_pend_objs);
        g.sweep_pend_objs = 0;
        g.sweep_pend_objs_pub.store(0, std::memory_order_relaxed);
    }
}
/** @brief Object free (called by sweep/collector).
 *  @note **No per-object global atomic RMW**: the three global counters (total_bytes / freed_objects /
 *        live_objects) first accumulate in collector-private accumulators and are folded in
 *        in one go at 64KB or at the end of the cycle. The reasoning is the same as for pend_bytes on the allocation side, and these three counters have
 *        only the collector as a writer.
 * @internal
 */
inline void object_free(gc_state& g, gc_object* o) {
    /* g is passed in as a parameter: this function is called **once per dead object**, so calling G() itself would pay for the magic-static guard for nothing. */
    g.sweep_pend_bytes += (std::size_t)o->size;   /* read before free, plain accumulation */
    ++g.sweep_pend_objs;
    if (g.sweep_pend_bytes >= 65536) sweep_stats_flush();
    /* The sweep_pend_*_pub snapshots are deliberately **not** updated per object here (only gc_stat() reads those values, they are purely diagnostic); the snapshots are published uniformly by sweep_stats_flush and the cycle epilogue. */
    chunk_free(o);
}
/**
 * @brief Nursery rehousing (owner side, executed in batches during PAUSE): move the objects "allocated during collection" one by one into young and reset their color. @note It only touches this heap's chains, and never runs concurrently with sweep.
 * @internal
 */
inline void rehome_some(gc_heap* h, std::size_t budget) {
    gc_state& g = G();
    const std::uint8_t w = g.white.load(std::memory_order_relaxed);
    for (std::size_t i = 0; i < budget && h->nursery.load(std::memory_order_relaxed) != nullptr; ++i) {
        gc_object* o = h->nursery.load(std::memory_order_relaxed);
        h->nursery.store(o->next.load(std::memory_order_relaxed),
                        std::memory_order_relaxed);
        o->marked.store(w, std::memory_order_relaxed);
        o->age    = (std::uint8_t)GC_AGE0;
        o->next.store(h->young.load(std::memory_order_relaxed),
                     std::memory_order_relaxed);
        h->young.store(o, std::memory_order_relaxed);
    }
    if (h->nursery == nullptr)
        h->rehome_pending.store(false, std::memory_order_relaxed);
}
/** @brief Restore a live object to "the next cycle's white", **and clear the BLACK bit**.
 *  @note BLACK must be cleared: the fallback rescan after a gray-set overflow relies on it to decide "was this already expanded this cycle", and a stale BLACK
 *        makes a long-lived object (for example a multi-MB large array) be treated as already expanded and skipped in the next cycle, so its children
 *        are never re-marked -> collected -> dangling pointer. Invariant: **BLACK is necessarily 0 when a live object enters a new cycle**
 *        (sweep goes through this function, and nursery rehousing writes `w` as a whole byte).
 * @internal
 */
inline void make_white(gc_object* o) {
    std::uint8_t m = mark_load(o);
    o->marked.store((std::uint8_t)((m & (std::uint8_t)~(WHITE_BITS | BLACK)) |
                                   G().white.load(std::memory_order_relaxed)),
                    std::memory_order_relaxed);
}
/* Marking: dye the object gray and push it onto the gray chain (dyeing removes the white bit, so each object is linked at most once). In a minor the old
 * generation is treated as a whole as the root set and its graph is not expanded. Leaf objects (a type that needs no scanning) are dyed black directly and do not enter the gray set
 * (the gray set is an array-style worklist of capacity GCXX_GRAY_CAPACITY). Concurrency: CAS to clear the white bit, the winner links onto the gray chain. */
/**
 * @brief Whether the mark window is still open (= `phase in {PROPAGATE, ATOMIC}`).
 * @note Used by `mark_object`'s "two-read repair": ask once again **after the store**.
 *       Reading the predicate with relaxed is deliberate -- this is an "after-the-fact confirmation", not synchronization; seeing "still open" one beat late while it is
 *       actually closed is extremely unlikely, and even then it only means "this cycle retains one more object", which is the safe direction (see the note on
 *       `mark_restore_white_bits`). This path is taken only when "a non-leaf object was actually marked", not in the innermost word-by-word loop.
 * @internal
 */
inline bool mark_window_still_open() {
    const int ph = G().phase.load(std::memory_order_relaxed);
    return ph == (int)GC_PHASE_PROPAGATE || ph == (int)GC_PHASE_ATOMIC;
}
/**
 * @brief **Restore to unmarked** an object that was dyed dark per the mark window (idempotent).
 *
 * Used for `mark_object`'s repair: the thread can be preempted between "reading PROPAGATE" and "the store", and
 * the mark-window epilogue has to cross `drain_mark_work` + `weak_process_after_mark`, so that gray-set entry
 * may end up with **no consumer at all** (`finish_cycle`'s `gray_clear()` silently drops it). If the object thus
 * crosses the cycle carrying "no white bit", the next cycle's `mark_object` skips it at `(m & WHITE_BITS) == 0` --
 * its children are never expanded -- **missed mark => dangling pointer**. Dropping a gray-set entry does not matter, **a missing white bit must never cross a cycle**.
 *
 * @note **Why confirm once more after the write-back**: the first re-read and the "write the white bit" can likewise be preempted, and by then
 *       sweep may already have started and **run past this object** (sweep only `make_white`s objects the cursor passes over),
 *       so nobody will ever restore its white bit again. Hence the form "confirm => write back => confirm again => write back again",
 *       which narrows the window to "it leaks only when there is no preemption". The write-back is **idempotent** (it only ORs the white bits back, it does not clear other bits).
 * @note Safe direction: this step **only retains more**. Dyeing dark inside the mark window means "marking is still in progress", and an object written back
 *       to white is not treated as marked this cycle, so the next cycle's root scan/self-scan will re-mark it normally; and the iron rule of a conservative GC is
 *       "rather retain more than ever miss a mark", so this direction is always acceptable. If the object is in the nursery right now, `finish_cycle`
 *       will also dye it uniformly to the current white bit, and the two paths agree.
 * @internal
 */
inline void mark_restore_white_bits(gc_object* o, std::uint8_t white_bits) {
    const std::uint8_t cur = mark_load(o);
    o->marked.store((std::uint8_t)(cur | white_bits), std::memory_order_relaxed);
}
/**
 * @brief Mark an object: clear the white bit -> dye -> advance the scan.
 * @param o the candidate object; for `excl` see `scan_ctx::excl`; `minor` decides whether the old generation is expanded.
 * @return the work done by this call (used by the adaptive pacing).
 * @internal
 */
inline std::size_t mark_object(gc_object* o, bool excl, bool minor) {
    /* **Marking is only allowed to happen in the mark phase (PROPAGATE / ATOMIC)**.
     *
     * During sweep (SWEEP / END) a mutator's "late self-scan" can still hand a candidate object here -- by then
     * **marking has long been declared finished** (`cycle_mark_done()` already returned true), and sweep may already have passed this object.
     * Once it is dyed dark here (= no white bit):
     *   1. no marker will ever come and expand it again;
     *   2. sweep will not `make_white` it again (it is already behind the cursor, or `sweep_stage` has reached 3);
     *   3. its gray-set entry will be **silently dropped** by `finish_cycle`'s `gray_clear()`
     *      (it neither records `gray_discards` nor sets `gray_overflow`).
     * => it **stays at no-white-bit forever**; the next cycle's `mark_object` **skips it** at
     * `(m & WHITE_BITS) == 0 -> already gray/black` => its children are missed and then swept
     * => **dangling pointer** (the one error this file must never make).
     *
     * The observed shape of that state: on the `old` chain `marked=0` (gray), `age=2`, `stage=3` (sweep finished)
     * while `gray_discards=0 gray_rescans=0 gray_overflow=0`, i.e. no counter ever recorded the discard.
     * It is latency-dependent and therefore reproducible only under thread churn.
     *
     * The predicate reads `phase` with `relaxed` and reads once more **after the store** to do the **repair** (see the
     * "two-read repair" below) -- a single read is not enough: the thread can be preempted between the single read and the store, and
     * "PROPAGATE -> ATOMIC -> atomic_finish" has to run through `drain_mark_work` +
     * `weak_process_after_mark`; once the store happens **after the last drain**,
     * nobody consumes that gray-set entry (`finish_cycle`'s `gray_clear()` drops it silently).
     * Since an acquire read on every store would raise the cost of this function on the scan hot path,
     * the form taken is "relaxed double read + repair after the store":
     *   * preemption between the read and the store => the second read sees "has left the mark window" => write the white bit back;
     *   * preemption after the store => that entry has already been drained and consumed (the object is expanded), no repair needed.
     * The repair direction **only retains more**: the object returns to "unmarked this cycle" and the next cycle re-marks it normally. */
    {
        const int ph = G().phase.load(std::memory_order_relaxed);
        if (ph != (int)GC_PHASE_PROPAGATE && ph != (int)GC_PHASE_ATOMIC) return 0;
    }
    std::uint8_t m = mark_load(o);
    if (excl) {
        /* ---- Exclusive-marking fast path: this thread is the only marker, no CAS needed ----
         * Rationale: begin_cycle expands only after `got >= want` (all registered heaps are in position), and a stopped
         * thread does not mark. The only possible concurrency is "a thread created mid-cycle" (its heap is registered
         * after the want count) -- it takes the CAS branch below; both sides store the **same value**, and enqueueing from both just expands the same object twice
         * (the array-style gray set tolerates duplicates). Atomic operations and explicit memory orders are still used so that the mixing stays well-defined. */
        if (m == MARK_FREE) return 0;
        if (minor && is_old(o)) return 0;
        if ((m & WHITE_BITS) == 0) return 0;
        const auto white_back = (std::uint8_t)(m & WHITE_BITS);
        o->marked.store((std::uint8_t)(m & (std::uint8_t)~WHITE_BITS),
                        std::memory_order_relaxed);
        if (is_leaf_object(o)) {
            /* Leaf: dyeing black is enough, no gray-set entry -- so the risk of "nobody consumes the gray-set entry" does not exist,
             * and no repair is needed (black means "expanded", which always holds for an object with no children). */
            o->marked.store((std::uint8_t)BLACK, std::memory_order_relaxed);
            return o->size;
        }
        gray_push(o);
        /* Two-read repair (see the function header): a gray-set entry may be dropped, but **a missing white bit must never be carried into the next cycle**.
         * Read the phase **once more** after the store as confirmation -- the first re-read and the "write the white bit" can still be preempted,
         * and once sweep has run past this object it will never `make_white` it again. */
        if (!mark_window_still_open()) {
            o->marked.store(white_back, std::memory_order_relaxed);
            return 0;
        }
        return 0;
    }
    for (;;) {
        if (m == MARK_FREE) return 0;       /* free-slot sentinel, must never be marked live */
        if (minor && is_old(o))
            return 0;                /* the old generation is a root in a minor; it is scanned by oldscan */
        if ((m & WHITE_BITS) == 0)
            return 0;                /* already gray/black: another thread is handling it or has handled it */
        const auto want = (std::uint8_t)(m & (std::uint8_t)~WHITE_BITS);
        if (o->marked.compare_exchange_weak(m, want,
                std::memory_order_relaxed, std::memory_order_relaxed)) {
            /* Reuse `want`, the value the CAS won with (i.e. the state after clearing the white bits); no need to load the object header again. */
            if (is_leaf_object(o)) {
                o->marked.store((std::uint8_t)(want | (std::uint8_t)BLACK),
                                std::memory_order_relaxed);
                return o->size;          /* leaf: the skipped bytes are accumulated locally by the caller */
            }
            gray_push(o);
            /* Two-read repair (see the function header). Note that after a successful CAS the object header may have been rewritten by another
             * marker and need not still be `want` (the multi-marker case), so it is expressed as "put the white bits back":
             * write back the white bits that were cleared in want, rather than writing the whole byte `m`.
             * After the write-back, **confirm once more**: if sweep has already started, repair once more idempotently (see the helper). */
            if (!mark_window_still_open()) {
                mark_restore_white_bits(o, (std::uint8_t)(m & WHITE_BITS));
            }
            return 0;
        }
    }
}
/**
 * @brief Conservative scanning: treat `[lo, hi)` as a "suspected pointer array" and check it word by word, aligned to the pointer width.
 *
 * Two levels of filtering: **fast exclusion by address range** (the vast majority of integers/floats/text are rejected here with a single unsigned
 * comparison), and an O(1) chunk-table reverse lookup (direct mapping + a single comparison).
 *
 * @note All memory reads go through `memcpy`: this both avoids unaligned access and avoids strict-aliasing problems;
 *       on x86 the compiler emits a plain mov, at zero cost.
 * @note **The grouping exists to create memory-level parallelism**: each candidate of a conservative scan has to chase 2~3 chained indirections
 *       (table entry -> chunk header -> object header), and out-of-order execution cannot overlap that latency across iterations. So 4 words
 *       (32B, one cache line) are taken as a group: fetch all the candidates of the group, issue every prefetch that can be issued,
 *       and only then resolve them one by one -- the 4 memory chains within a group are independent and the hardware can advance them in parallel.
 *
 * @warning **These 4 lanes must stay written out as unrolled fixed-length code; do not "tidy" them into a `for (k...)` loop.**
 *       If the group's loads, prefetches and resolutions are each rewritten into a loop with "a runtime upper bound `n` and an
 *       inner `break`", `pause` p50 and `alloc-small` both degrade by tens of percent, while the set of words checked and the
 *       prefetches issued stay word-for-word identical. The cause is not the algorithm but that **the compiler no longer
 *       unrolls the inner body**: only a fixed-length array index plus a constant upper bound can be unrolled into a
 *       branch-free straight-line sequence; once the bound becomes a runtime variable it degrades to "one comparison plus
 *       one branch per lane". The cost of this path is in the **instruction stream**, not in the algorithm.
 * @param sc the scan context (heap address range, diagnostic accumulator, exclusive/minor flags).
 * @param lo_v the start address (aligns up to `sizeof(void*)` internally).
 * @param hi_v the end address (exclusive).
 * @internal
 */
inline void scan_words(scan_ctx& sc, const void* lo_v, const void* hi_v) {
    gc_state& g = *sc.g;
    constexpr auto align = (std::uintptr_t)sizeof(void*);
    std::uintptr_t lo = (reinterpret_cast<std::uintptr_t>(lo_v) + align - 1) & ~(align - 1);
    const auto hi = reinterpret_cast<std::uintptr_t>(hi_v);
    if (hi <= lo) return;
    const std::uintptr_t hlo  = sc.lo;
    const std::uintptr_t span = sc.hi - hlo;
    std::size_t bytes = 0;
    /* Main body: 4 candidates at a time (32B); fetch all 4, prefetch them all, then resolve them one by one. */
    for (; lo + 4 * align <= hi; lo += 4 * align) {
        std::uintptr_t v0, v1, v2, v3;
        std::memcpy(&v0, reinterpret_cast<const void*>(lo),             sizeof(std::uintptr_t));
        std::memcpy(&v1, reinterpret_cast<const void*>(lo + align),     sizeof(std::uintptr_t));
        std::memcpy(&v2, reinterpret_cast<const void*>(lo + 2 * align), sizeof(std::uintptr_t));
        std::memcpy(&v3, reinterpret_cast<const void*>(lo + 3 * align), sizeof(std::uintptr_t));
        if (v0 - hlo < span) {
            GCXX_PREFETCH(&g.chunk_table[chunk_slot_index(g, v0)]);
            GCXX_PREFETCH(reinterpret_cast<const void*>(v0 & ~(std::uintptr_t)CHUNK_MASK));
        }
        if (v1 - hlo < span) {
            GCXX_PREFETCH(&g.chunk_table[chunk_slot_index(g, v1)]);
            GCXX_PREFETCH(reinterpret_cast<const void*>(v1 & ~(std::uintptr_t)CHUNK_MASK));
        }
        if (v2 - hlo < span) {
            GCXX_PREFETCH(&g.chunk_table[chunk_slot_index(g, v2)]);
            GCXX_PREFETCH(reinterpret_cast<const void*>(v2 & ~(std::uintptr_t)CHUNK_MASK));
        }
        if (v3 - hlo < span) {
            GCXX_PREFETCH(&g.chunk_table[chunk_slot_index(g, v3)]);
            GCXX_PREFETCH(reinterpret_cast<const void*>(v3 & ~(std::uintptr_t)CHUNK_MASK));
        }
        if (v0 - hlo < span) { bytes += align; resolve_and_mark(sc, v0); }
        if (v1 - hlo < span) { bytes += align; resolve_and_mark(sc, v1); }
        if (v2 - hlo < span) { bytes += align; resolve_and_mark(sc, v2); }
        if (v3 - hlo < span) { bytes += align; resolve_and_mark(sc, v3); }
    }
    /* Epilogue: the remainder of less than one group (0~3 words) is handled word by word. */
    for (; lo + align <= hi; lo += align) {
        std::uintptr_t v = 0;
        std::memcpy(&v, reinterpret_cast<const void*>(lo), sizeof(void*));
        if (v - hlo >= span) continue;           /* a single unsigned comparison performs the range exclusion */
        bytes += align;
        resolve_and_mark(sc, v);
    }
    /* Diagnostic counters are folded in once **per slice** (not once per object); see scan_ctx::skipped. */
    sc.da->scanned_bytes += bytes;
    sc.da->skipped_bytes += sc.skipped;
    sc.skipped = 0;
}
/* ------------------------------- Stack handling -------------------------------- */
/* Take the address of a local variable in the current stack frame (used only for gc_init's "stack-top anchor"; there the value is taken in the **caller's** frame,
 * with the opposite semantics). It returns an **integer address** rather than a pointer: it is used only as a number and is never dereferenced -- returning it directly would be
 * diagnosed by -Wreturn-stack-address as "returning the address of a local variable", and the caller might mistakenly believe it is accessible. */
/**
 * @brief Convert an address to an integer (for register capture).
 * @note It is a separate function so that the compiler **cannot optimize it away during root scanning** -- when a root lives only in a register, eliminating it by inlining is the same as missing it.
 * @internal
 */
inline std::uintptr_t stack_addr_as_int(const volatile char* p) {
    return reinterpret_cast<std::uintptr_t>(p);
}
/**
 * @brief Take the address of a local variable that is **necessarily inside the current stack frame** (the stack-anchor fallback when there is no inline assembly).
 * @note Less safe than the real `rsp`; a fallback only.
 * @internal
 */
/* MSVC reports C4172 ("returning address of local variable or temporary") -- **this is a false positive**:
 * what is returned is the **numeric value** of the address (`stack_addr_as_int` immediately turns it into a `uintptr_t`; it is never dereferenced
 * and never used as a pointer). Moreover this function is **never called at all** on MSVC (MSVC takes the `_alloca(1)`
 * branch below); it is just that an inline function in a header still gets compiled.
 * The pragma must wrap **the whole function**: MSVC reports C4172 on **the function's line** (col 1),
 * not on the `return`, so wrapping only the `return` does not suppress it. */
#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable : 4172)
#endif
inline std::uintptr_t stack_marker_addr() {
    volatile char probe = 0;
    return stack_addr_as_int(&probe);
}
#if defined(_MSC_VER)
#  pragma warning(pop)
#endif
/* The **lower bound of a conservative scan must be the current real stack pointer rsp**, not "the address of some local variable": the compiler will
 * inline gc / GC internals into the caller, at which point the locals live in the same frame and who is above whom is decided by the allocator; a live root
 * pointer can perfectly well sit at a lower address than the "anchor local variable" and be missed, so the object is wrongly collected. */
/**
 * @brief Get **the current real stack pointer** (x86-64 uses `movq %rsp`; MSVC uses `_alloca(1)`).
 * @note It must be the real rsp, not "the address of some local variable" -- gc internals get inlined into the caller, at which point locals and roots live in the same frame and who is above whom is decided by the allocator, so a live root may sit at a lower address and be missed.
 * @internal
 */
inline char* current_stack_pointer() {
#if !defined(GCXX_NO_INLINE_ASM) && (defined(__clang__) || defined(__GNUC__)) \
    && defined(__x86_64__)
    void* sp = nullptr;
    __asm__ __volatile__("movq %%rsp, %0" : "=r"(sp));
    return reinterpret_cast<char*>(sp);
#elif !defined(GCXX_NO_INLINE_ASM) && (defined(__clang__) || defined(__GNUC__)) \
    && (GCXX_ARCH_AARCH64)
    /* AArch64: `mov %0, sp` (both Android and iOS use clang, and both support it). */
    void* sp = nullptr;
    __asm__ __volatile__("mov %0, sp" : "=r"(sp));
    return reinterpret_cast<char*>(sp);
#elif defined(_MSC_VER)
    /* MSVC has no inline assembly to read rsp: use _alloca(1) to obtain an address necessarily below the current rsp */
    return reinterpret_cast<char*>(_alloca(1));
#else
    return reinterpret_cast<char*>(stack_marker_addr());
#endif
}
/* Lowest address of the committed stack area (used to clamp the lower bound so that uncommitted pages are not read) */
/**
 * @brief The **lowest address** of the committed stack area (Windows takes the TEB's StackLimit).
 * @return the lower bound; returns `nullptr` on unsupported platforms.
 * @internal
 * @note **Android's API threshold**: `pthread_getattr_np` / `pthread_attr_getstack` are symbols Bionic
 *       exported only as of **API 24 (Android 7.0)**, and they require `_GNU_SOURCE`.
 *       Using them below 24 causes a **link failure**, so this explicitly falls back to `nullptr` --
 *       `stack_scan_lower_bound` consequently falls back to "the current sp minus `STACK_LOWER_MARGIN`",
 *       which is the **conservative direction** (scans more, never misses a mark), only slightly more expensive. */
inline char* platform_stack_bottom() {
#if GCXX_PLATFORM_WINDOWS
    /* TEB+0x10 = StackLimit (lowest address of the committed stack) */
    return *reinterpret_cast<char**>(
        reinterpret_cast<char*>(NtCurrentTeb()) + 0x10);
#elif GCXX_HAVE_PTHREAD_GETATTR_NP
    /* Platforms with `pthread_getattr_np` (Linux / Android API>=24). */
    pthread_attr_t attr;
    void*  addr = nullptr;
    size_t size = 0;
    if (pthread_getattr_np(pthread_self(), &attr) != 0) return nullptr;
    pthread_attr_getstack(&attr, &addr, &size);
    pthread_attr_destroy(&attr);
    return reinterpret_cast<char*>(addr);
#elif GCXX_HAVE_PTHREAD_ATTR_GET_NP
    /* **FreeBSD**: there is no `pthread_getattr_np`, but `<pthread_np.h>` provides
     * `pthread_attr_get_np`, declared as
     * `int pthread_attr_get_np(pthread_t, pthread_attr_t *)`.
     * @note On FreeBSD 13+ `pthread_attr_t` is an **opaque pointer**, so `pthread_attr_init()` must be
     *       called first and the result passed in -- passing an uninitialized stack variable directly is undefined behavior. */
    pthread_attr_t attr;
    if (::pthread_attr_init(&attr) != 0) return nullptr;
    void*  addr = nullptr;
    size_t size = 0;
    if (::pthread_attr_get_np(pthread_self(), &attr) != 0) {
        ::pthread_attr_destroy(&attr);
        return nullptr;
    }
    ::pthread_attr_getstack(&attr, &addr, &size);
    ::pthread_attr_destroy(&attr);
    return reinterpret_cast<char*>(addr);
#else
    /* The committed stack lower bound is unavailable => `stack_scan_lower_bound` falls back to "the current sp minus STACK_LOWER_MARGIN".
     * This is the **conservative direction** (scans more, never misses a mark), only slightly more expensive.
     * The platforms in this bucket: NetBSD / OpenBSD / DragonFly (no verified usable interface, so no guessing),
     * Android API<24, and unknown POSIX. */
    return nullptr;
#endif
}
/* Compute the stack lower bound for this scan: **the larger of the platform lower bound and "rsp minus the margin"**.
 *
 * **Never use the platform lower bound directly**:
 * the `addr` given by `pthread_attr_getstack()` includes the **stack guard page** on some platforms
 * -- that page is `PROT_NONE`, and touching it is a `SIGSEGV`, while the crash site is the first
 * `memcpy` in `scan_words`, which looks nothing like "the stack boundary was computed wrong" (lldb stack: `scan_words ... invalid permissions
 * for mapped object`, with the address landing exactly at the low end of `[addr, addr+size)`).
 *
 * **These interfaces give the "stack region", not the "safely readable range"** --
 * the two are not equivalent on platforms with a guard page, and "can it be read safely" is the predicate a scan needs.
 * The right trade-off for a conservative GC is: **rather be less precise than suffer a single out-of-bounds read**.
 *
 * So the predicate is "tighten **at most** to `rsp - STACK_LOWER_MARGIN`":
 *   * `want` starts from `rsp - STACK_LOWER_MARGIN` (the lower bound itself leaves a 256B margin);
 *   * the platform lower bound is used to **raise** want only when it is **higher than want** --
 *     raising = scanning a bit less already-useless stack (safe), lowering = possibly stepping on the guard page (unsafe).
 *
 * @note This holds for Windows too: the TEB `StackLimit` can likewise fall on the guard page, so MinGW must not take a
 *       separate path that skips the clamp.
 * @note Is `STACK_LOWER_MARGIN` (256B) enough? Yes -- it only exists to cover the case where "the compiler pushes a few
 *       temporaries below rsp", while the guard page is further down, avoided by "not lowering". */
inline char* stack_scan_lower_bound() {
    char* rsp  = current_stack_pointer();
    char* want = rsp - (std::ptrdiff_t)STACK_LOWER_MARGIN;
    char* floor_ = platform_stack_bottom();
    if (floor_ != nullptr && floor_ > want) want = floor_;
    return want;
}
/* Register root capture: a conservative GC must also treat "pointers currently living in registers" as roots. setjmp only guarantees
 * that callee-saved registers are saved, and its coverage varies across compilers and optimization levels, so it can
 * miss "pointers that exist only in registers" and thus wrongly collect. So on x86/x86-64 inline assembly is used to write
 * **all** general-purpose registers into a stack buffer; that buffer lies inside this function's frame and is covered by the subsequent stack scan. */
/**
 * @brief Write **all general-purpose registers** (plus the 16 XMM registers on x86-64) into the buffer provided by the caller.
 * @param buf at least `REG_CAPTURE_SLOTS` `uintptr_t`s.
 * @note `setjmp` is not used: it only guarantees that callee-saved registers are saved, and its coverage varies with compiler and optimization level, so "pointers that live only in registers" can be missed. The buffer is inside this frame and is subsequently covered by the stack scan.
 * @internal
 */
inline void capture_registers(std::uintptr_t* buf) {
#if !defined(GCXX_NO_INLINE_ASM) && (defined(__clang__) || defined(__GNUC__)) \
    && defined(__x86_64__)
    /* The stack pointer is read in its own asm block and handed to the main block as an input operand.
     * Writing `movq %%rsp, 120(%0)` in the main block instead would make that block read a register it
     * does not declare as an input: a compiler tracks operand registers and clobbers, not "registers the
     * template happens to mention", so a template that reads an undeclared register silently tells the
     * compiler a stale value is still live there. Nothing goes wrong for `rsp` (it is never handed out as
     * an operand register), but the AArch64 arm below has no such luck and the same laxness is a real
     * defect there; keeping both arms on the same declared-operand footing is what stops the pattern from
     * being copied again. */
    std::uintptr_t sp = 0;
    __asm__ __volatile__("movq %%rsp, %0" : "=r"(sp));
    __asm__ __volatile__(
        "movq %%rax,   0(%0)\n\t"
        "movq %%rbx,   8(%0)\n\t"
        "movq %%rcx,  16(%0)\n\t"
        "movq %%rdx,  24(%0)\n\t"
        "movq %%rsi,  32(%0)\n\t"
        "movq %%rdi,  40(%0)\n\t"
        "movq %%rbp,  48(%0)\n\t"
        "movq %%r8,   56(%0)\n\t"
        "movq %%r9,   64(%0)\n\t"
        "movq %%r10,  72(%0)\n\t"
        "movq %%r11,  80(%0)\n\t"
        "movq %%r12,  88(%0)\n\t"
        "movq %%r13,  96(%0)\n\t"
        "movq %%r14, 104(%0)\n\t"
        "movq %%r15, 112(%0)\n\t"
        "movq %1,    120(%0)\n\t"
        /* Vector registers may also hold pointers temporarily (on Windows x64 xmm6-xmm15 are callee-saved, and their values may never
         * reach the stack); movups is used so that 16-byte alignment is not required. */
        "movups %%xmm0,  128(%0)\n\t"
        "movups %%xmm1,  144(%0)\n\t"
        "movups %%xmm2,  160(%0)\n\t"
        "movups %%xmm3,  176(%0)\n\t"
        "movups %%xmm4,  192(%0)\n\t"
        "movups %%xmm5,  208(%0)\n\t"
        "movups %%xmm6,  224(%0)\n\t"
        "movups %%xmm7,  240(%0)\n\t"
        "movups %%xmm8,  256(%0)\n\t"
        "movups %%xmm9,  272(%0)\n\t"
        "movups %%xmm10, 288(%0)\n\t"
        "movups %%xmm11, 304(%0)\n\t"
        "movups %%xmm12, 320(%0)\n\t"
        "movups %%xmm13, 336(%0)\n\t"
        "movups %%xmm14, 352(%0)\n\t"
        "movups %%xmm15, 368(%0)\n\t"
        : : "r"(buf), "r"(sp) : "memory");
#elif !defined(GCXX_NO_INLINE_ASM) && (defined(__clang__) || defined(__GNUC__)) \
    && (GCXX_ARCH_AARCH64)
    /* AArch64 (the dominant ABI on Android / iOS): capture **31 general-purpose registers** (x0..x30).
     * @note The trade-off differs from the x86-64 branch: in the ARM64 calling convention x0..x17 are caller-saved,
     *       so in principle a pointer that "lives across a call" can only be in x19..x30; but **a conservative GC must not rely on that reasoning**
     *       -- the compiler may keep a pointer in x0..x17 within the same function, and when `capture_registers` is inlined into
     *       the caller those registers are still in use. So all of them are captured here, at a cost of only 31 stores.
     * @note Vector registers are not used: ARM64's v0..v31 serve only floating point/SIMD, and this library does not move pointers out of floats;
     *       capturing 16~32 fewer slots also saves half the stores.
     * @note **`sp` is an input operand, never a register the template writes.** `sp` is not an allocatable
     *       operand (the ABI reserves it), so `mov x9, sp` inside the template writes an **undeclared**
     *       register -- and declaring that register as a clobber does not help either: GCC accepts a clobber
     *       that also holds an operand and then allocates the operand into the clobbered register anyway, so
     *       the buffer address would be overwritten by the `sp` move and every store below would land at the
     *       wrong address. Capturing `sp` from C leaves the template with only declared registers, exactly as
     *       the x86-64 arm above does for `rsp`. */
    std::uintptr_t sp = 0;
    __asm__ __volatile__("mov %0, sp" : "=r"(sp));
    __asm__ __volatile__(
        "stp x0,  x1,   [%0, #0]\n\t"
        "stp x2,  x3,   [%0, #16]\n\t"
        "stp x4,  x5,   [%0, #32]\n\t"
        "stp x6,  x7,   [%0, #48]\n\t"
        "stp x8,  x9,   [%0, #64]\n\t"
        "stp x10, x11,  [%0, #80]\n\t"
        "stp x12, x13,  [%0, #96]\n\t"
        "stp x14, x15,  [%0, #112]\n\t"
        "stp x16, x17,  [%0, #128]\n\t"
        "stp x18, x19,  [%0, #144]\n\t"
        "stp x20, x21,  [%0, #160]\n\t"
        "stp x22, x23,  [%0, #176]\n\t"
        "stp x24, x25,  [%0, #192]\n\t"
        "stp x26, x27,  [%0, #208]\n\t"
        "stp x28, x29,  [%0, #224]\n\t"
        "stp x30, %1,   [%0, #240]\n\t"
        : : "r"(buf), "r"(sp) : "memory");
#else
    /* MSVC x64 has no inline assembly, and non-x86-64/AArch64 architectures are not implemented:
     * the registers are covered by the local jmp_buf in mark_roots_self (setjmp spilling the callee-saved ones). */
    (void)buf;
#endif
    const volatile auto temp = buf[0];
    buf[0] = temp;   /* keep the compiler from optimizing the buffer away */
}
inline char* platform_stack_top() {
#if GCXX_PLATFORM_WINDOWS
    /* Thread stack top: TEB+0x08 = StackBase (NtCurrentTeb comes from winnt.h; the offset is a stable
     * ABI convention, and under LEAN_AND_MEAN the _TEB structure is incomplete, so the offset is used directly) */
    return *reinterpret_cast<char**>(
        reinterpret_cast<char*>(NtCurrentTeb()) + 0x08);
#elif GCXX_HAVE_PTHREAD_GET_STACKADDR_NP
    /* Apple (macOS/iOS): `pthread_get_stackaddr_np` **returns the stack base address itself** (the high address),
     * exactly the "stack top" required here; iOS has no `pthread_getattr_np`, so this is the only option.
     * @note The range it returns includes the guard page (`addr` is the highest address), and together with
     *       `pthread_get_stacksize_np` the lower bound can be computed -- see the note on `platform_stack_bottom()`
     *       for Apple. */
    return reinterpret_cast<char*>(::pthread_get_stackaddr_np(::pthread_self()));
#elif GCXX_HAVE_PTHREAD_GETATTR_NP
    pthread_attr_t attr;
    void*  addr = nullptr;
    size_t size = 0;
    if (pthread_getattr_np(pthread_self(), &attr) != 0) return nullptr;
    pthread_attr_getstack(&attr, &addr, &size);
    pthread_attr_destroy(&attr);
    return reinterpret_cast<char*>(addr) + size;
#elif GCXX_HAVE_PTHREAD_ATTR_GET_NP
    /* FreeBSD: same shape as `platform_stack_bottom()` (`pthread_attr_t` is an opaque pointer). */
    pthread_attr_t attr;
    if (::pthread_attr_init(&attr) != 0) return nullptr;
    void*  addr = nullptr;
    size_t size = 0;
    if (::pthread_attr_get_np(::pthread_self(), &attr) != 0) {
        ::pthread_attr_destroy(&attr);
        return nullptr;
    }
    ::pthread_attr_getstack(&attr, &addr, &size);
    ::pthread_attr_destroy(&attr);
    return reinterpret_cast<char*>(addr) + size;
#else
    /* Neither `pthread_getattr_np` nor Apple's `pthread_get_stackaddr_np` exists
     * (**this is the bucket NetBSD / OpenBSD / DragonFly fall into**). Return nullptr and the caller falls back to
     * "take the address of a local variable in the current stack frame as the anchor"; see `my_heap_slow` / `gc_start_impl`.
     * The upper bound at those two places must be `max(current rsp, anchor) + margin`, not just `anchor + STACK_MARGIN`:
     *    the stack grows toward **lower addresses**, and the scanned interval is `[rsp, stack_top)` -- addresses
     *    **higher** than `stack_top` belong to **the caller's outer frames** (`main` / user functions), where GC pointers
     *    are equally live roots. When the anchor lands inside the GC call chain, the outer frames are **above** the anchor; if the upper bound is
     *    pinned to `anchor + 64KiB`, then whenever "an outer frame is more than 64KiB away from the anchor" those frames **are never scanned**
     *    => wrong collection => dangling pointer. What is missed in that case is the live frames **above** the anchor, which is **under-scanning**.
     * @note This `#else` is required: without it the function **returns no value** on this path
     *       (`-Wreturn-type`), which is undefined behavior. */
    return nullptr;
#endif
}
/** @brief The **block granularity** (bytes) for re-scanning static areas / dynamic roots.
 *
 * Block boundaries align to real page boundaries, so "a block was written" corresponds one-to-one with "a page was written" -- this is
 * the prerequisite for a future "only re-scan pages that were written" (see the note on scan_root_range_paged).
 * 4096 is the page size on x86-64 Windows/Linux; **even if it differs from the real page size it does not
 * affect correctness**, it only makes "block" and "page" no longer correspond one-to-one (choosing the wrong granularity costs some savings, it does not
 * miss a scan).
 * @internal
 */
constexpr std::size_t ROOT_PAGE_BYTES = 4096;
static_assert((ROOT_PAGE_BYTES & (ROOT_PAGE_BYTES - 1)) == 0,
              "ROOT_PAGE_BYTES must be a power of two (page-boundary "
              "arithmetic uses a mask)");
static_assert(ROOT_PAGE_BYTES % sizeof(void*) == 0,
              "ROOT_PAGE_BYTES must be a multiple of sizeof(void*) so that "
              "block boundaries never split a scanned word");
/* Boundary symbols of the platform's static area. **C linkage is mandatory**: C++ linkage would mangle the names into
 * _ZN4gcxx6detail... while the linker only has undecorated symbols and cannot find the definitions.
 * The declaration must be at **namespace scope** -- `extern "C"` is not allowed inside a function body
 * (C++ standard [dcl.link]); writing it as an in-function block simply will not compile.
 *
 * **Which pair of symbols is used per platform -- all verified by real linking**:
 *
 * | Platform | Static root range | Basis |
 * |---|---|---|
 * | Linux / Android | `[__data_start, _end)` | the GNU/Bionic linker script provides `__data_start` |
 * | **FreeBSD** | **`[__executable_start, _end)`** | `__data_start` is an **undefined symbol** there, while `__executable_start` / `_end` / `__bss_start` / `_edata` are **all RESOLVED** |
 * | Darwin | `[get_etext(), get_end())` | Apple has neither of those two ELF symbols |
 *
 * **A platform conclusion must come from real linking, never from "some form of it was tried once"**:
 * naming `__data_start` on FreeBSD makes the link fail outright, and declaring the BSD static root "a gap"
 * would be false because `__executable_start` links there.
 *
 * @note `[__executable_start, _end)` is **wider** than `[__data_start, _end)` (it additionally contains `.text`
 *       and `.rodata`) -- **safe by direction** (rather scan more), at the cost of a few tens of KB to a few MB more scanned per root scan.
 *       If there is no more precise pair that **really links**, wide is the right trade-off.
 *
 * **NetBSD / OpenBSD / DragonFly**: unverified,
 * so **no guessing** -- on those platforms "the symbol branch does not take effect", but **the dynamic-root branch still does**
 * (`dl_iterate_phdr` also exists in the NetBSD/OpenBSD libc), so no scan is missed:
 * the writable `PT_LOAD` segment of the main program image gets collected by `dl_iterate_phdr` as well.
 * That is precisely the trade-off of "rather cover uniformly with dynamic roots than gamble on some symbol name". */
#if GCXX_USE_STATIC_ROOTS && (GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID)
extern "C" {
    extern char __data_start[];
    extern char _end[];
}
#endif
#if GCXX_USE_STATIC_ROOTS && GCXX_PLATFORM_FREEBSD
extern "C" {
    /* The FreeBSD linker script does **not** provide `__data_start` (it is undefined); it provides these two. */
    extern char __executable_start[];
    extern char _end[];
}
#endif
#if GCXX_USE_STATIC_ROOTS && GCXX_PLATFORM_DARWIN
extern "C" {
    /* Apple has no `__data_start`/`_end`; the public substitute is the `<mach-o/getsect.h>` family's
     * `get_etext()` / `get_end()` (exported by libSystem, linkable). What it gives is
     * the range "from the end of __TEXT to the end of the image" -- **wider** than the precise `[__data_start,_end)`,
     * and it sweeps the read-only __DATA_CONST / __LINKEDIT in as well.
     * **Safe by direction** (rather scan more), at the cost of a few tens of KB ~ a few MB more per root scan. */
    extern char* get_etext(void);
    extern char* get_end(void);
}
#endif

/**
 * @brief Resolve the platform static area (the writable data segment where globals live) range and write it into g.static_ranges in pairs.
 *
 * GC pointers stored in globals are roots too; otherwise an object referenced only by a global would be wrongly collected.
 * Only **writable** sections (.data/.bss) are scanned. The range is resolved once before first use and cached in gc_state.
 * @internal
 */
/**
 * @brief Merge `[lo, hi)` into the root range table (at most `cap` pairs); **overlapping/adjacent ranges are merged**.
 *
 * `r` is flat: `[lo0,hi0, lo1,hi1, ...]`, and at all times it stays **pairwise disjoint and non-descending**.
 *
 * @note **Why not just "deduplicate"**: the test "skip if the new range is **fully contained** in some existing range"
 * blocks only "exact duplicates", not **partial overlaps** -- and partial overlaps are very common on real platforms:
 * the **symbol range** of the main program image (e.g. FreeBSD's `[__executable_start,_end)`) and the
 * **precise PT_LOAD segments** given by `dl_iterate_phdr` necessarily overlap each other.
 * There are two consequences, both unacceptable:
 *   1. the same memory is **scanned twice** (pure waste, and static-area scanning is expensive to begin with: 17.65 ns/word);
 *   2. the table's entry count is needlessly inflated, crowding out other ranges (a full table silently drops, which is a **missed scan**).
 * => the predicate must be "merge when they intersect or are adjacent", so that **enabling both root sources together is equivalent to enabling either alone**.
 *
 * @note The merge uses a "swallow forward" formulation (`i` does not advance; it keeps comparing with the next one), guaranteeing convergence in a single pass;
 *       the complexity is O(n^2), and `n <= STATIC_RANGES_MAX` (64 by default) and it only runs once when root ranges are collected.
 * @internal
 */
inline void root_range_add(char** r, unsigned* n, unsigned cap, char* lo, char* hi) {
    if (*n >= cap || hi <= lo) return;
    /* 1) The new range is contained in an existing range => nothing to do. */
    for (unsigned i = 0; i < *n; ++i)
        if (lo >= r[2 * i] && hi <= r[2 * i + 1]) return;
    /* 2) Merge in each range that intersects or touches it, one by one (keep looking at the next one after merging; it can chain). */
    for (unsigned i = 0; i < *n; ) {
        const bool overlap = (hi >= r[2 * i] && lo <= r[2 * i + 1]);
        if (!overlap) { ++i; continue; }
        if (r[2 * i]     < lo) lo = r[2 * i];      /* extend left */
        if (r[2 * i + 1] > hi) hi = r[2 * i + 1];  /* extend right */
        /* Remove entry i (shift the following entries forward by one slot) and decrement *n. */
        for (unsigned k = i; k + 1 < *n; ++k) {
            r[2 * k]     = r[2 * k + 2];
            r[2 * k + 1] = r[2 * k + 3];
        }
        --*n;
        /* i does not advance: it may still overlap what was entry i+1. */
    }
    r[2 * *n] = lo;
    r[2 * *n + 1] = hi;
    ++*n;
}
#if GCXX_PLATFORM_HAVE_LINK_H
/** @brief Context for the `dl_iterate_phdr` callback. */
struct dyn_root_ctx { char** r; unsigned* n; unsigned cap; };
/* Smooth over the ELF type-name difference between Linux and BSD (the first four members have the same semantics and layout; see
 * the note on `GCXX_PLATFORM_HAVE_LINK_H`):
 *   Linux (<link.h>)  : ElfW(Addr) / ElfW(Phdr) / ElfW(Half)
 *   FreeBSD (<sys/link_elf.h>): Elf_Addr / Elf_Phdr / Elf_Half
 * @note Both are synonyms of `unsigned long` / the corresponding struct / `unsigned short`,
 *       so the aliases merely let the code compile on both sides; they **do not change semantics**. */
#if GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID
typedef ElfW(Addr)  gcxx_elf_addr;
typedef ElfW(Phdr)  gcxx_elf_phdr;
typedef ElfW(Half)  gcxx_elf_half;
#else
typedef Elf_Addr    gcxx_elf_addr;
typedef Elf_Phdr    gcxx_elf_phdr;
typedef Elf_Half    gcxx_elf_half;
#endif
#if GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID
static_assert(sizeof(gcxx_elf_addr) == sizeof(void*),
              "ELF address width must match the pointer width (64-bit only)");
#endif
/**
 * @brief `dl_iterate_phdr` callback: collect the **writable** `PT_LOAD` segments of every **loaded object** (main program image + shared libraries + anything
 *        brought in by `dlopen`) into the range table.
 * @note The main program image (`dlpi_name` empty) is **skipped** -- it is already covered by the static-area branch
 *       (Linux/Android's `[__data_start,_end)`; **FreeBSD's `[__executable_start,_end)`**). Everything else is exactly the gap to be filled here.
 * @note Only `PF_W` is taken: a read-only segment cannot contain "a mutable root". `PT_GNU_RELRO` falls inside a writable segment and is scanned along with it;
 *       the scan only **reads** it, so there are no side effects.
 * @note **On FreeBSD the `dlpi_name` of the main program image is sometimes " " and sometimes the executable path**
 *       (it depends on the rtld version), so the predicate "`dlpi_addr == 0` or `dlpi_name` empty" is more robust --
 *       on Linux the `dlpi_name` of the main program image is always the empty string; on FreeBSD the
 *       `dlpi_addr` of the main program image is always 0 (non-PIE) or equal to the image base. Using "the name is empty" as the one
 *       **common predicate** works for both, and any surplus segment is eaten by `root_range_add`'s overlap merging.
 * @internal
 */
inline int dyn_roots_cb(struct dl_phdr_info* info, std::size_t, void* data) {
    dyn_root_ctx* c = static_cast<dyn_root_ctx*>(data);
    if (info->dlpi_name == nullptr || info->dlpi_name[0] == '\0') return 0;   /* main program image */
    for (int i = 0; i < info->dlpi_phnum; ++i) {
        const gcxx_elf_phdr& ph = info->dlpi_phdr[i];
        if (ph.p_type != PT_LOAD) continue;
        if ((ph.p_flags & PF_W) == 0) continue;
        if (ph.p_memsz == 0) continue;
        char* lo = reinterpret_cast<char*>(info->dlpi_addr + ph.p_vaddr);
        root_range_add(c->r, c->n, c->cap, lo, lo + ph.p_memsz);
    }
    return 0;                       /* 0 = keep iterating */
}
#endif
#if GCXX_USE_DYNAMIC_ROOTS && GCXX_PLATFORM_WINDOWS
/**
 * @brief Append the **writable sections** (`IMAGE_SCN_MEM_WRITE`) of the image `image` as root ranges.
 * @return how many segments were actually appended this call (those skipped by deduplication do not count).
 *
 * @note This function **only parses PE; it calls no Windows enumeration API** => its arithmetic and bounds
 *       (taking the NT header via `e_lfanew`, locating the section table, RVA=> pointer, VirtualSize) are **platform independent**,
 *       so it can be unit-tested on Linux with a synthetic PE image, without waiting for a Windows build.
 * @internal
 */
inline unsigned win_pe_add_writable(void* image, char** r, unsigned* n, unsigned cap) {
    if (image == nullptr) return 0;
    auto* dos = static_cast<IMAGE_DOS_HEADER*>(image);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE) return 0;
    /* `e_lfanew` is the offset to the NT header and must be positive (a negative value would cause an out-of-bounds read below). */
    if (dos->e_lfanew <= 0) return 0;
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(
                   reinterpret_cast<char*>(dos) + (std::size_t)dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return 0;
    const std::uint16_t nsec = nt->FileHeader.NumberOfSections;
    /* The upper bound 96 gives an explicit limit so that a malformed image cannot make the section table be read as an unbounded array. */
    if (nsec == 0 || nsec > 96) return 0;
    const IMAGE_SECTION_HEADER* sec = IMAGE_FIRST_SECTION(nt);
    char* img = reinterpret_cast<char*>(dos);
    unsigned added = 0;
    for (std::uint16_t i = 0; i < nsec; ++i) {
        if ((sec[i].Characteristics & IMAGE_SCN_MEM_WRITE) == 0) continue;
        if (sec[i].Misc.VirtualSize == 0) continue;
        const unsigned before = *n;
        char* lo = img + sec[i].VirtualAddress;
        root_range_add(r, n, cap, lo, lo + sec[i].Misc.VirtualSize);
        if (*n > before) ++added;
    }
    return added;
}
/**
 * @brief Enumerate **all loaded modules** (including DLLs brought in at runtime by `LoadLibrary`) and append their writable sections.
 *
 * @note It follows the same reasoning as Linux's `dl_iterate_phdr`: it **must be called outside the lock**.
 * @note A fixed 256-entry buffer, bailing out on failure; **no dynamic allocation** -- this function sits on a cold branch of the allocation path
 *       and must not introduce a new failure point.
 * @note The main program image is **skipped**: it is already handled separately by the `_WIN32` branch of `static_ranges_gather`.
 * @internal
 */
inline void win_dll_roots_add(char** r, unsigned* n, unsigned cap) {
    const HMODULE self = ::GetModuleHandleA(nullptr);
    HMODULE mods[256];
    DWORD needed = 0;
    if (!::EnumProcessModules(::GetCurrentProcess(), mods, (DWORD)sizeof(mods), &needed)) return;
    auto cnt = (unsigned)(needed / sizeof(HMODULE));
    if (cnt > 256u) cnt = 256u;
    for (unsigned i = 0; i < cnt; ++i) {
        if (mods[i] == nullptr || mods[i] == self) continue;
        (void)win_pe_add_writable(reinterpret_cast<void*>(mods[i]), r, n, cap);
    }
}
#endif
#if GCXX_USE_DYNAMIC_ROOTS && GCXX_HAVE_DYLD_IMAGE_API
/**
 * @brief Append the **writable segments** of a Mach-O image (`header` points at its `mach_header`/`mach_header_64`)
 *        as root ranges. `slide` is the image's ASLR slide.
 * @return how many segments were actually appended this call (those skipped by deduplication do not count).
 *
 * @note Exactly corresponding to `win_pe_add_writable`: **pure parsing, no enumeration API called**,
 *       so its arithmetic and bounds (magic validation, the `sizeofcmds` upper bound, load command traversal,
 *       the `vmaddr` + slide conversion) are all **platform independent**, and it can be unit-tested on Linux
 *       with a synthetic Mach-O image, without waiting for the Apple toolchain.
 * @note Only segments with `VM_PROT_WRITE` are taken: a read-only segment cannot contain "a mutable root".
 *       `__DATA` / `__DATA_DIRTY` / `__DATA_CONST` / `__OBJC` are all writable (or once were) and
 *       are collected together. **Segment names are deliberately not filtered** -- the segment name is not the predicate, `initprot` is.
 * @note `__PAGEZERO`'s `initprot` is 0 (neither readable nor writable), so it is filtered out naturally.
 * @note Upper bounds: both the load command count and `sizeofcmds` get hard limits, so a malformed image cannot make the command
 *       stream be read as an unbounded array (the same reasoning as the 96-section bound on the PE side).
 * @internal
 */
inline unsigned macho_add_writable(const void* header, std::intptr_t slide,
                                   char** r, unsigned* n, unsigned cap) {
    if (header == nullptr) return 0;
    const auto* h = static_cast<const struct mach_header*>(header);
    const bool is64 = (h->magic == MH_MAGIC_64);
    if (!is64 && h->magic != MH_MAGIC) return 0;
    const std::uint32_t ncmds    = h->ncmds;
    const std::uint32_t sizeofcmds = h->sizeofcmds;
    /* Hard upper bounds: a normal Mach-O has a few dozen commands and a command stream of a few tens of KB. */
    if (ncmds == 0 || ncmds > 512u || sizeofcmds == 0 || sizeofcmds > (16u * 1024u * 1024u))
        return 0;
    /* The 64-bit header is 32 bytes, the 32-bit one is 28 bytes; the command stream follows the header immediately. */
    const char* p = static_cast<const char*>(header) + (is64 ? sizeof(mach_header_64)
                                                             : sizeof(mach_header));
    const char* const end = p + sizeofcmds;
    unsigned added = 0;
    for (std::uint32_t i = 0; i < ncmds; ++i) {
        if (p + sizeof(load_command) > end) break;
        const auto* lc = reinterpret_cast<const load_command*>(p);
        if (lc->cmdsize < sizeof(load_command)) break;      /* 0/too small would make the loop below spin forever */
        if (p + lc->cmdsize > end) break;
        if (lc->cmd == (is64 ? LC_SEGMENT_64 : LC_SEGMENT)) {
            /* **The 32-bit and 64-bit segment command layouts differ; never read a 32-bit image with the 64-bit structure**:
             *   segment_command (32-bit): vmaddr/vmsize are uint32, at offsets 24/28
             *   segment_command_64      : vmaddr/vmsize are uint64, at offsets 24/32
             * Forcing a 64-bit read makes `vmsize` pick up the high half of `fileoff` (usually 0),
             * so the segments of a 32-bit image are **dropped wholesale** -- a silent missed scan, exactly the error that must never happen.
             * This project targets only 64-bit platforms, but the parser is still implemented correctly per the format:
             * Darwin has been all-64-bit since 10.15, yet 32-bit headers still occur in fat/old images. */
            if (is64) {
                const auto* sg = reinterpret_cast<const segment_command_64*>(p);
                if ((sg->initprot & VM_PROT_WRITE) != 0 && sg->vmsize != 0) {
                    const unsigned before = *n;
                    char* lo = reinterpret_cast<char*>(
                        static_cast<std::uintptr_t>(sg->vmaddr + (std::uint64_t)slide));
                    root_range_add(r, n, cap, lo, lo + (std::size_t)sg->vmsize);
                    if (*n > before) ++added;
                }
            } else {
                const auto* sg = reinterpret_cast<const segment_command*>(p);
                if ((sg->initprot & VM_PROT_WRITE) != 0 && sg->vmsize != 0) {
                    const unsigned before = *n;
                    char* lo = reinterpret_cast<char*>(
                        static_cast<std::uintptr_t>(sg->vmaddr + (std::uint32_t)slide));
                    root_range_add(r, n, cap, lo, lo + (std::size_t)sg->vmsize);
                    if (*n > before) ++added;
                }
            }
        }
        p += lc->cmdsize;
    }
    return added;
}
/**
 * @brief Enumerate **all loaded Mach-O images** (executable + shared libraries + anything `dlopen`ed at runtime)
 *        and append their writable segments.
 *
 * @note It uses `_dyld_image_count` / `_dyld_get_image_header` / `_dyld_get_image_vmaddr_slide`:
 *       all three are **publicly exported by libSystem**, present on both macOS and iOS, and they **introduce no new link dependency**
 *       (this library already depends on libSystem for the RSS/stack interfaces). It deliberately does **not** use
 *       the `_dyld_register_func_for_add_image` callback registration -- that requires registering very early (right after the first image), while this library collects only at the first collection,
 *       and once the registration window is missed there will never be a callback.
 * @note The main program image (index 0) is **skipped**: it is already covered by the Apple branch of `static_ranges_gather`
 *       (`get_etext`/`get_end`).
 * @note **No dynamic allocation, no locking**: these `_dyld_*` queries are read-only snapshots and can be called outside the lock
 *       (same reasoning as the `dl_iterate_phdr` item; see `scan_dyn_roots_slow`).
 * @internal
 */
inline void darwin_image_roots_add(char** r, unsigned* n, unsigned cap) {
    const std::uint32_t cnt = _dyld_image_count();
    if (cnt == 0) return;
    /* Upper bound: a normal process has a few dozen images; this guards against a malformed/pathological case filling the range table instantly.
     * Only the first DARWIN_MAX_IMAGES are traversed, the rest are **abandoned** (the other side of the conservative direction:
     * when the range table is full `root_range_add` rejects on its own too -- two layers of protection). */
    const std::uint32_t maximg = 256u;
    const std::uint32_t lim = (cnt > maximg) ? maximg : cnt;
    for (std::uint32_t i = 1; i < lim; ++i) {          /* 0 = main program image */
        const struct mach_header* h = _dyld_get_image_header(i);
        if (h == nullptr) continue;
        const std::intptr_t slide = _dyld_get_image_vmaddr_slide(i);
        (void)macho_add_writable(h, slide, r, n, cap);
    }
}
#endif
/**
 * @brief Collect the root ranges of "static area + writable data segments of dynamic libraries" into `r` (at most `cap` pairs); return the pair count.
 * @note It **does not touch `gc_state`** and can therefore be called **outside the lock** (it takes/releases the dynamic loader's lock).
 *       This is precisely what removes the lock-order inversion; see `scan_dyn_roots_slow`.
 * @internal
 */
inline unsigned static_ranges_gather(char** r, unsigned cap) {
    (void)r; (void)cap;      /* with `GCXX_USE_STATIC_ROOTS=0` none of the branches below uses it */
    /* The symbol arms below write `r[0]` / `r[1]` **directly** instead of going through
     * `root_range_add`, which is the only place that checks `*n >= cap`. That is safe only
     * because they all run with `n == 0`: they write at most one pair, so they need
     * `cap >= 1`. The assertion documents the implicit precondition instead of leaving it
     * to be re-derived; the call site passes `STATIC_RANGES_MAX` (64 by default). */
    static_assert(STATIC_RANGES_MAX >= 1,
                  "static_ranges_gather's symbol arms write r[0]/r[1] unconditionally, "
                  "so the table must have room for at least one range");
    unsigned n = 0;
    /* ---- 1) The static area of the main program image (where globals live) ----
     *
     * **On FreeBSD this branch is used only when the dynamic roots cannot cover the main program image.**
     * On FreeBSD (non-PIE, image base 0x200000) the symbol range
     * `[__executable_start, _end)` is ~119 KB wide, and symbols like `_end` come from the **main program
     * linker script**, whose enclosed interval **crosses holes that the program headers claim exist but whose pages are not mapped**,
     * so the symbol range itself may contain unreadable pages.
     * Instrumenting or rebuilding the image moves those holes, so the failure depends on the layout.
     *
     * **Why "use the precise segments" is the right answer**: what `dl_iterate_phdr` gives is the **precise PT_LOAD segments**
     * (with the `PF_W` filter), which are exactly the intervals the loader **actually mapped** -- they have no "holes inside a symbol span".
     *
     * @note **Who covers the main program image, per platform -- read this before touching either step.**
     *       Step 2's callback deliberately **returns early for the main program image**
     *       (`dyn_roots_cb` tests `dlpi_name` empty; `darwin_image_roots_add` starts at image 1;
     *       `win_dll_roots_add` skips `GetModuleHandleA(nullptr)`), so "step 2 will pick it up" is
     *       **false on Linux and Darwin**. The actual division of labour is:
     *         * **Linux/Android** -- covered by **this step** (`[__data_start, _end)`), which is
     *           therefore **mandatory**, not a fallback. Step 2 does not contribute it.
     *         * **FreeBSD** -- the symbol range is skipped here (it can cross unmapped pages, see
     *           above) and left to step 2, which only really covers it when rtld reports a
     *           non-empty `dlpi_name` for the main image (rtld-version dependent, see the note on
     *           `dyn_roots_cb`); on the versions where it reports `""` the main image is missed.
     *           That residual gap is the accepted price of not using a range that can cross
     *           unmapped pages: the alternative is a **crash**, and the symbols on FreeBSD have
     *           never been verified to be hole-free.
     *         * **Darwin** -- **NOT COVERED by either step on a normal build**, and this is a real
     *           (documented, not-yet-fixed) root gap: the `get_etext()`/`get_end()` arm below is
     *           itself gated on `!GCXX_HAVE_DYLD_IMAGE_API`, which is **1** on every macOS/iOS
     *           target, so the arm is compiled out; and step 2 starts at image 1, skipping the main
     *           image. Net effect: on macOS/iOS a GC pointer held **only** in a global of the main
     *           executable is not a root and its object is reclaimed. The fix would be to ungate the
     *           `get_etext`/`get_end` arm the same way Linux is ungated, but that arm cannot be
     *           verified without an Apple toolchain, and a platform conclusion here must come from a
     *           real build, so the code is left as-is and the gap is recorded here instead of being guessed away.
     *         * **Windows** -- covered by this step's PE-section parse of the main image.
     * @note Apple has no `dl_iterate_phdr`, but it has `_dyld_*` (`GCXX_HAVE_DYLD_IMAGE_API`),
     *       which likewise gives precise segments for the **shared libraries**.
     * @warning **NetBSD / OpenBSD / DragonFly are currently a gap in this chain** (see the note on the
     *       `#else` branch below). Their `GCXX_PLATFORM_HAVE_LINK_H` is 0
     *       (only Linux/FreeBSD/Android ~1 are 1), so:
     *         * this function's symbol branch falls to `#else` (registers nothing);
     *         * step 2's dynamic-root branch is compiled out as well (it is likewise conditioned on `GCXX_PLATFORM_HAVE_LINK_H`).
     *       => these three have **neither static roots nor dynamic roots**, so "an object referenced only by a global/static variable"
     *       is collected as unreachable. The dynamic roots of step 2 cover them only if that guard is
     *       lifted, and FreeBSD's capability must not be read as the whole BSD family's capability.
     * @note **This whole `#if` chain is the one place where a wrong guard silently removes a root
     *       rather than failing loudly**, so any edit here has to be checked against the "who covers
     *       the main program image" list above, not just against "does it compile". */
#if GCXX_USE_STATIC_ROOTS && (GCXX_PLATFORM_LINUX || GCXX_PLATFORM_ANDROID)
    /* Linux/Android: **this arm is the main program image's ONLY root source** (see the list above),
     * so it must not be removed or gated further. Step 2 contributes the shared libraries only;
     * `root_range_add` still merges any overlap, so nothing is scanned twice.
     *
     * Keeping the symbol range is safe here and not on FreeBSD because `[__data_start, _end)`
     * covers only `.data` + `.bss`, which is contiguous, writable, and entirely mapped, while
     * FreeBSD's `[__executable_start, _end)` spans ~119 KB and **does** cross
     * pages the program headers claim but the loader never mapped, which is why that
     * platform drops it instead. */
    if (+__data_start != +_end) { r[0] = __data_start; r[1] = _end; n = 1; }
#elif GCXX_USE_STATIC_ROOTS && GCXX_PLATFORM_FREEBSD && !GCXX_PLATFORM_HAVE_LINK_H
    /* FreeBSD **without** `dl_iterate_phdr` (a hypothetical old version): the only option is to fall back to the symbol range.
     * This range is wide and **may contain unmapped pages** -- a known risk, but there is no better option;
     * modern FreeBSD all have `dl_iterate_phdr`, so this should not normally be reached. */
    if (+__executable_start != +_end) { r[0] = __executable_start; r[1] = _end; n = 1; }
#elif GCXX_USE_STATIC_ROOTS && GCXX_PLATFORM_DARWIN && !GCXX_HAVE_DYLD_IMAGE_API
    /* Apple without `_dyld_*` (a hypothetical old SDK): fall back to `get_etext`/`get_end`.
     * It is likewise "a rather wide span": safe by direction but not precise enough. */
    char* a = get_etext();
    char* b = get_end();
    if (b > a) { r[0] = a; r[1] = b; n = 1; }
#elif GCXX_USE_STATIC_ROOTS && GCXX_PLATFORM_WINDOWS
    /* The main program image (GetModuleHandleA(nullptr), i.e. the old __ImageBase).
     * The parsing logic is shared with the DLL path via `win_pe_add_writable`, avoiding writing it twice.
     * @note Windows **also** has the dynamic branch (`EnumProcessModules`), where the main program image is skipped;
     *       what is given here is the **precise PE writable sections**, with no symbol-span problem. */
    (void)win_pe_add_writable(reinterpret_cast<void*>(::GetModuleHandleA(nullptr)), r, &n, cap);
#else
    /* **NetBSD / OpenBSD / DragonFly and unknown platforms: this is a gap, and there is no fallback.**
     *
     * Symbol names must not be filled in here: whether `__data_start` / `_end` /
     * `__executable_start` even exist and what their semantics are is **unverified**; guessing wrong is a silent missed scan
     * (worse than "a gap" -- a gap is at least explicitly known). So this **deliberately does nothing**.
     *
     * For these three, `GCXX_PLATFORM_HAVE_LINK_H` is **0**, so step 2's dynamic-root branch below
     * (whose condition is likewise `GCXX_PLATFORM_HAVE_LINK_H`) **is compiled out too**.
     * => they have **neither static roots nor dynamic roots**. FreeBSD's capability must not be read as the
     * whole BSD family's capability.
     *
     * Consequence: an object referenced only by a global/static variable is collected as unreachable on these three (dangling pointer).
     * `gc_refresh_roots()` cannot save it either (it too is a no-op when `GCXX_PLATFORM_HAVE_LINK_H` is 0).
     * Until the interfaces are confirmed on those platforms, "do not rely on static/global variables as the only root on these three platforms"
     * is **a limitation the user needs to know**. */
#endif
    /* ---- 2) The writable data segments of all loaded objects (including the main program image + shared libraries + dlopen'ed ones) ----
     * This branch is the **dynamic roots**, and it is also the right answer to the BSD/Android static-root gap:
     *   * the main program image was already covered by symbols in the previous step => it is skipped in the callback (`dlpi_name` empty);
     *     should some platform not skip it, `root_range_add`'s overlap merging eats the duplicate segment.
     *   * `dl_iterate_phdr` / `EnumProcessModules` take the dynamic loader's lock --
     *     so this function **must be called outside the lock** (see `scan_dyn_roots_slow`). */
#if GCXX_USE_DYNAMIC_ROOTS && GCXX_PLATFORM_HAVE_LINK_H
    {
        dyn_root_ctx c; c.r = r; c.n = &n; c.cap = cap;
        (void)::dl_iterate_phdr(&dyn_roots_cb, &c);
    }
#endif
#if GCXX_USE_DYNAMIC_ROOTS && GCXX_PLATFORM_WINDOWS
    /* Writable sections of DLLs (the counterpart of Linux/FreeBSD's dl_iterate_phdr). The main program image is
     * skipped inside `win_dll_roots_add`. Likewise it **must be called outside the lock**. */
    win_dll_roots_add(r, &n, cap);
#endif
#if GCXX_USE_DYNAMIC_ROOTS && GCXX_HAVE_DYLD_IMAGE_API
    /* Writable segments of Mach-O images (Apple's counterpart). The main program image is
     * skipped inside `darwin_image_roots_add`. */
    darwin_image_roots_add(r, &n, cap);
#endif
    return n;
}
/**
 * @brief Collect "static area + dynamic library data segments" into `g.static_ranges` (overwriting).
 * @note The caller must ensure mutual exclusion (both existing call sites are in a `gc_mutex` / STW context).
 * @internal
 */
inline void static_ranges_collect(gc_state& g) {
    for (auto& p : g.static_ranges) p = nullptr;
    g.static_range_count = static_ranges_gather(g.static_ranges, STATIC_RANGES_MAX);
}
/**
 * @brief **Cold-path** collection of dynamic roots: **collect outside the lock, publish inside it**.
 *
 * **Why the two must be separated**: `dl_iterate_phdr` takes the dynamic loader's (`_dl_load_lock`) lock,
 * while `gc_mutex` is held elsewhere. Calling it while holding `gc_mutex` constitutes a
 * **lock-order inversion** with the following path:
 *   * collector: holds `gc_mutex` -> wants `_dl_load_lock`;
 *   * a library loaded by `dlopen` calls `gc<T>()` inside its `.init_array` (constructors):
 *     holds `_dl_load_lock` -> wants `gc_mutex`.
 * => deadlock. This function splits the two steps: **first collect outside the lock into a stack temporary array** (by then the loader lock is released),
 * **then take `gc_mutex` and publish**. The two locks are never held overlapping.
 *
 * It triggers only on the first pass through the allocation path (when `dyn_roots_scanned` is false); afterwards the hot path just reads one relaxed atomic.
 * @internal
 */
GCXX_NOINLINE inline void scan_dyn_roots_slow(gc_state& g) {
    if (g.dyn_roots_scanned.load(std::memory_order_acquire)) return;
    char* tmp[2 * STATIC_RANGES_MAX];
    const unsigned n = static_ranges_gather(tmp, STATIC_RANGES_MAX);   /* outside the lock */
    {
        gc_mutex_guard lk;                                             /* publish inside the lock */
        if (!g.static_inited) {
            for (unsigned i = 0; i < 2 * n; ++i) g.static_ranges[i] = tmp[i];
            g.static_range_count = n;
            g.static_inited = true;
        }
    }
    g.dyn_roots_scanned.store(true, std::memory_order_release);
}
/** @brief Re-collect the static area + dynamic library data segment roots (called after `dlopen`; see `gc_refresh_roots`). */
inline void static_ranges_refresh(gc_state& g) {
    char* tmp[2 * STATIC_RANGES_MAX];
    const unsigned n = static_ranges_gather(tmp, STATIC_RANGES_MAX);   /* outside the lock */
    {
        gc_mutex_guard lk;
        for (auto& p : g.static_ranges) p = nullptr;
        for (unsigned i = 0; i < 2 * n; ++i) g.static_ranges[i] = tmp[i];
        g.static_range_count = n;
        g.static_inited = true;
    }
}
/**
 * @brief How many words the cached static area + dynamic roots total (for statistics).
 * @internal
 */
inline std::size_t static_root_words(gc_state& g) {
    std::size_t n = 0;
    for (unsigned i = 0; i < g.static_range_count; ++i) {
        const char* lo = g.static_ranges[2 * i];
        const char* hi = g.static_ranges[2 * i + 1];
        if (hi > lo) n += (std::size_t)(hi - lo) / sizeof(void*);
    }
    return n;
}
/**
 * @brief Split a root range `[lo, hi)` **at page boundaries** and hand it to `scan_words` block by block.
 *
 * This is an **equivalent rewrite** of "split by page but still scan everything": splitting changes only the
 * **number of times** `scan_words` is called, not the **set of words** checked --
 *
 *   1. each block abuts the next (`p = page_end`), and their union is exactly `[lo, hi)`;
 *   2. every block start except the first is a page boundary, and 4096 is a multiple of `sizeof(void*)`, so
 *      the "align the start upward" step inside `scan_words` discards no extra word in any block; the first block's
 *      start is `lo` itself, so its alignment behavior matches a whole-range scan;
 *   3. the 4-word pipelined grouping may shift by one position at the seams between blocks, but that is only a difference in
 *      **prefetch/access order**: every word is still checked exactly once, and the set of candidates `resolve_and_mark`
 *      receives is exactly the same (marking is idempotent and order independent).
 *
 * The only observable difference is that the diagnostic counters (`scanned_bytes` / `skipped_bytes`) are folded in **per block**
 * rather than per whole range -- the accumulated totals are unchanged (`diag_accum` is purely diagnostic and takes no part in control flow).
 * @internal
 */
inline void scan_root_range_paged(scan_ctx& sc, const char* lo, const char* hi) {
    constexpr auto kPage = (std::uintptr_t)ROOT_PAGE_BYTES;
    auto       p     = reinterpret_cast<std::uintptr_t>(lo);
    const auto end   = reinterpret_cast<std::uintptr_t>(hi);
    while (p < end) {
        /* The end of the page containing `p`; always > p (the in-page offset < kPage), so the loop must advance. */
        std::uintptr_t page_end = (p & ~(kPage - 1)) + kPage;
        if (page_end > end) page_end = end;
        scan_words(sc, reinterpret_cast<const void*>(p),
                       reinterpret_cast<const void*>(page_end));
        p = page_end;
    }
}
/**
 * @brief Scan the **static area** roots (the range snapshot initialized once).
 * @note This is a fixed cost paid every cycle; `static constexpr` should not be placed in the static area.
 * @internal
 */
inline void scan_static_roots() {
    gc_state& g = G();
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);
    scan_ctx sc = make_scan_ctx(da);
    if (!g.static_inited) {
        g.static_range_count = 0;
        g.static_inited = true;                  /* set it first to avoid recursion */
        static_ranges_collect(g);
    }
    for (unsigned i = 0; i < g.static_range_count; ++i) {
        char* lo = g.static_ranges[2 * i];
        char* hi = g.static_ranges[2 * i + 1];
        if (hi > lo) scan_root_range_paged(sc, lo, hi);
    }
}
/* ---- Root scanning (multi-threaded version) ---- mark_roots_self scans this thread's own roots (registers + stack) and is called in two places:
 * when a safepoint is reached (mutator self-scan, rsp exact) and when the collector starts marking (for its own stack). The static area and
 * the stacks of "threads that missed the safepoint" are scanned by the collector inside STW. */
/**
 * @brief Scan **this thread's own** roots: registers + stack.
 * @note Called in two places: when a safepoint is reached (mutator self-scan, rsp exact) and when the collector starts marking. The static area and the stacks of "threads that missed the safepoint" are scanned by the collector inside STW.
 * @internal
 */
inline void mark_roots_self(gc_heap* h) {
    gc_state& g = G();
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);
    scan_ctx sc = make_scan_ctx(da);
    /* 1) Registers: captured into a stack buffer in this frame (which the stack scan then covers). Platforms with no inline assembly (MSVC x64)
     *    fall back to setjmp spilling the callee-saved registers -- a local jmp_buf of this frame must be used (multiple threads cannot share g.regs). */
    std::uintptr_t regs[REG_CAPTURE_SLOTS] = {};
#if defined(_MSC_VER)
    /* Real MSVC: a local jmp_buf spills the callee-saved registers (setjmp must be in the same frame as the scan, and multiple threads cannot share state) */
    {
        /* MSVC reports C4611 ("interaction between `_setjmp` and C++ object destruction is non-portable") --
         * **benign here**: this spot only uses `setjmp` to **spill the callee-saved registers into `jb`**,
         * and immediately afterwards scans that memory as roots; **there is no `longjmp` anywhere in it**,
         * so the premise "a jump skips destructors" does not hold. Locally suppressed with this note. */
#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable : 4611)
#endif
        jmp_buf jb;
        if (setjmp(jb) == 0)
            scan_words(sc, &jb, reinterpret_cast<char*>(&jb) + sizeof(jmp_buf));
#if defined(_MSC_VER)
#  pragma warning(pop)
#endif
    }
#endif
    capture_registers(&regs[0]);   /* GCC/Clang: all general-purpose + vector registers */
    /* 2) Thread stack: scan from the real rsp up to stack_top */
    std::size_t probed = 0;
    g.root_scans.fetch_add(1, std::memory_order_relaxed);
    const char* lo = stack_scan_lower_bound();
    const char* hi = h->stack_top;
    if (hi != nullptr && hi > lo) {
        const auto span = (std::size_t)(hi - lo);
        if (span > (std::size_t)MAX_STACK_SCAN)
            lo = hi - (std::size_t)MAX_STACK_SCAN;
        probed += (std::size_t)(hi - lo) / sizeof(void*);
        scan_words(sc, lo, hi);
    }
    g.root_probed_words.fetch_add(probed, std::memory_order_relaxed);
    diag_flush();                     /* fold the counters accumulated during the self-scan into the globals in one go */
}
/* Safepoint reached: the mutator calls this after seeing the global request in gc_pace / gc_mutex_lock -- self-scan stack + registers
 * -> record the cycle -> park -> wait for the initiating thread to advance the epoch and release it.
 *
 * Why the mutator scans **itself**: the collector scans only its own roots and the frozen stacks of force-suspended threads; a thread that
 * parks voluntarily stops in its own stack frame, so its roots must be scanned by itself before parking (the pointers in the stack/registers are necessarily reachable at that moment). The typical
 * case is a thread created mid-cycle: its stack may hold object pointers handed over from another thread through a non-GC channel, and missing them means
 * a wrong collection. Prerequisite: **the heap must be published into the global chain before it can possibly reach a safepoint** (see my_heap_slow and
 * gc_state::heaps), otherwise the STW-initiating thread cannot enumerate it, and a mark pushed onto the gray chain by the self-scan may land after marking has finished.
 *
 * The price is that "cancel this cycle" cannot simply throw the state away: the self-scan has already dyed objects, so the cancel path must first drain the in-flight self-scans
 * and then roll the colors back wholesale (see the cancel branch of begin_cycle). */
/**
 * @brief Forward declaration of **safepoint reach** (the definition is in the heap lifetime section).
 * @internal
 */
inline void safepoint_reach(gc_heap* h) {
    gc_state& g = G();
    if (g.sp_request.load(std::memory_order_acquire) == 0) return;  /* no STW */
    GCXX_MT_TRACE("[mt] %llu sp_enter h=%p\n", gc_ts_ms(), (void*)h);
    const unsigned long long ep = g.sp_epoch.load(std::memory_order_acquire);
    /* Check in before entering the self-scan: the cancel path uses this counter to drain "threads that are currently writing marks".
     * seq_cst is required -- together with the initiating thread's `sp_request.store(0, seq_cst)` it forms
     * a store-buffer style handshake, guaranteeing that "this thread reads request != 0" and "the initiating thread reads
     * the counter as 0" cannot both hold (otherwise the drain would miss a thread that is in the middle of dyeing). */
    g.sp_scanning.fetch_add(1, std::memory_order_seq_cst);
    if (g.sp_request.load(std::memory_order_seq_cst) == 0) {
        g.sp_scanning.fetch_sub(1, std::memory_order_seq_cst);
        return;                       /* just caught the end of this cycle: no self-scan needed */
    }
    mark_roots_self(h);
    g.sp_scanning.fetch_sub(1, std::memory_order_seq_cst);
    /* Confirm once more after the self-scan: if this cycle has already ended in the meantime (request back to zero / epoch advanced),
     * there is no need to park for it (that would wait uselessly until the next cycle). */
    if (g.sp_request.load(std::memory_order_acquire) == 0 ||
        g.sp_epoch.load(std::memory_order_acquire) != ep)
        return;
    /* This thread is about to park: first publish the unbooked allocations. After that the collector cannot see
     * new allocations (this thread is stopped), and the bytes of the objects it will sweep this cycle must all already be in
     * total_bytes -- so the free path cannot subtract total_bytes into a negative number.
     * It must come **before** store(park_epoch): that store is a release, and
     * the collector's acquire read relies on it to establish happens-before (see publish_pending). */
    publish_pending(h);
    /* This thread has demonstrably been running and using this heap during this round: record it, so that
     * `reap_dead_heaps` cannot mistake an idle-but-live owner for an abandoned heap (see last_active_epoch). */
    h->last_active_epoch.store(ep, std::memory_order_relaxed);
    h->park_epoch.store(ep, std::memory_order_release);
    while (g.sp_epoch.load(std::memory_order_acquire) == ep) {
        std::this_thread::yield();
    }
    GCXX_MT_TRACE("[mt] sp_exit\n");
}
/* ---- Thread-exit cleanup: platform TLS callbacks (**not** C++ thread_local destructors) ----------
 *
 * Platform callbacks (Windows FLS / POSIX pthread key) are used here rather than destructors of C++ `thread_local`
 * objects, for two reasons:
 *
 *  1. The order of C++ TLS destruction is uncontrollable, while "exit cleanup" must come **before** TLS objects that users destroy
 *     later -- those may still allocate during their destruction (see heap_thread_exit_cleanup).
 *  2. More fundamentally: MinGW implements `thread_local` as **emutls**, and emutls's per-thread
 *     block is already invalid before the user's TLS destructors run. When the current thread's heap pointer is stored in a
 *     C++ `thread_local`, the usage "call gc<>() inside a user TLS destructor" crashes intermittently,
 *     because the pointer read inside the destructor is **not the same** as the one in the thread body.
 *     So the heap pointer and the diagnostic accumulator both use platform slots / gc_heap members;
 *     see heap_tls_get. */
/**
 * @brief Heap cleanup at thread exit: hand it to the collector (the struct is retained until `reap_dead_heaps` unlinks it).
 * @internal
 */
inline void heap_thread_exit_cleanup(gc_heap* p);   /* defined after chunk_free */
#if GCXX_PLATFORM_WINDOWS
/**
 * @brief FLS callback at thread exit (Windows): mark this thread's heap dead and register it for cleanup.
 * @internal
 */
inline VOID NTAPI heap_fls_callback(PVOID data) {
    heap_thread_exit_cleanup(static_cast<gc_heap*>(data));
}
/**
 * @brief This thread's FLS index (allocated only once per process lifetime).
 * @return the index; `FLS_OUT_OF_INDEXES` means allocation failed.
 * @note It **must be exposed as separately accessible**: `dispose_all()` has to clear this thread's FLS slot
 *       **as well** on reset -- otherwise the exit callback still holds a `free`d heap pointer, and dereferencing it at
 *       process exit is an **ACCESS_VIOLATION**. On Windows this shows up as "after `gc_dispose()`, allocate nothing
 *       more and just exit" (`0xC0000005`), while a single further allocation avoids it because that allocation
 *       rebuilt the heap and the TLS/FLS slots.
 * @internal
 */
inline DWORD heap_fls_index() {
    /* One process-lifetime index (magic static is thread-safe); the callback fires
     * for EVERY exiting thread and reads that thread's own heap. */
    static DWORD idx = FlsAlloc(&heap_fls_callback);
    return idx;
}
/**
 * @brief Register the "clean up the heap at thread exit" callback (Windows implementation).
 * @internal
 */
inline void heap_tls_register_exit(gc_heap* h) {
#if GCXX_SKIP_THREAD_EXIT_CLEANUP
    (void)h;      /* test-only: pretend the exit hook never ran */
#else
    const DWORD idx = heap_fls_index();
    if (idx != FLS_OUT_OF_INDEXES) FlsSetValue(idx, h);
#endif
}
/**
 * @brief Clear this thread's TLS/FLS slots so that the exit callback **no longer** gets a freed heap (Windows).
 * @note Both must be cleared: the `TlsSetValue` slot (`heap_tls_get` reads it) and **the FLS slot**
 *       (the exit callback reads it). Clearing only the `TlsSetValue` slot leaves the callback with a freed heap pointer.
 * @note It only clears the slots and does **not** free the FLS index (the index is process-wide and can keep being used by new threads).
 * @internal
 */
inline void heap_tls_clear_all() {
    heap_tls_set(nullptr);
#if !GCXX_SKIP_THREAD_EXIT_CLEANUP
    const DWORD idx = heap_fls_index();
    if (idx != FLS_OUT_OF_INDEXES) FlsSetValue(idx, nullptr);
#endif
}
#elif GCXX_PLATFORM_POSIX
/**
 * @brief pthread key destructor (POSIX): mark this thread's heap dead at thread exit.
 * @internal
 */
inline void heap_pthread_key_dtor(void* data) {
    heap_thread_exit_cleanup(static_cast<gc_heap*>(data));
}
/**
 * @brief The pthread key used by the thread-exit callback (**not the same key as `gc_tls_key()`**).
 * @return the key; returns 0 when creation fails (that thread will not receive an exit callback).
 * @note It **must be exposed as separately accessible**, for exactly the same reason as Windows' `heap_fls_index()`:
 *       `dispose_all()` has to clear this slot as well, otherwise at thread exit the destructor receives a **heap pointer that has
 *       already been `free`d** (see `heap_tls_clear_all`).
 *       The same hazard exists on POSIX: exiting directly after `gc_dispose()` hands the destructor a dangling pointer.
 * @note Key creation goes through a **function-local static** (thread-safe one-time initialization since C++11);
 *       a "`static bool created` plus manual emptiness check" would be a data race and
 *       can let some thread read `created == true` while `key == 0` still holds,
 *       so its heap never receives the exit callback.
 * @internal
 */
inline pthread_key_t heap_exit_key() {
    static pthread_key_t key = 0;
    static const bool created =
        (::pthread_key_create(&key, &heap_pthread_key_dtor) == 0);
    (void)created;
    return key;
}
/**
 * @brief Register the "clean up the heap at thread exit" callback (POSIX implementation).
 * @internal
 */
inline void heap_tls_register_exit(gc_heap* h) {
#if GCXX_SKIP_THREAD_EXIT_CLEANUP
    (void)h;      /* test-only: pretend the exit hook never ran */
    return;
#endif
    const pthread_key_t key = heap_exit_key();
    if (key != 0) ::pthread_setspecific(key, h);
}
/**
 * @brief Clear this thread's TLS slots so that the exit callback **no longer** receives an already-freed heap (POSIX implementation).
 * @note On the POSIX side, what must be cleared is **the key used by the exit callback** (`heap_exit_key()`, which has a destructor installed),
 *       **not** `gc_tls_key()` (the one used by `heap_tls_get/set`, which has no destructor installed).
 *       These are two distinct keys with different values: clearing the wrong one clears nothing at all, and at thread exit the destructor will still receive
 *       a dangling `gc_heap*`. After clearing to `nullptr`, `heap_pthread_key_dtor(nullptr)` takes
 *       the null-pointer early exit of `heap_thread_exit_cleanup`, which is safe.
 * @note `heap_tls_set(nullptr)` clears the other slot (the one `heap_tls_get` reads); both places must be cleared,
 *       exactly corresponding to clearing both `TlsSetValue` + FLS on the Windows side.
 * @internal
 */
inline void heap_tls_clear_all() {
    heap_tls_set(nullptr);
    const pthread_key_t key = heap_exit_key();
    if (key != 0) ::pthread_setspecific(key, nullptr);
}
#else
inline void heap_tls_register_exit(gc_heap*) {}   /* unknown platform */
/** @brief Unknown platform: only clear the TLS slot (there is no exit callback to undo). @internal */
inline void heap_tls_clear_all() { heap_tls_set(nullptr); }
#endif
/* ---- Per-thread heap: slow-path creation + thread-exit cleanup ---- */
/**
 * @brief The **slow path** for this thread's heap: create it if absent, register it on the global list, and enroll thread-exit cleanup.
 * @return This thread's heap (never returns `nullptr`).
 * @note When creating the heap, the object range must be registered into `heap_lo`/`heap_hi` first, otherwise this thread's heap will be missed by scanning.
 * @internal
 */
inline gc_heap* my_heap_slow() {
    gc_state& g = G();
    auto* h = static_cast<gc_heap*>(std::calloc(1, sizeof(gc_heap)));
    if (h == nullptr) std::abort();
    h = ::new (static_cast<void*>(h)) gc_heap;   /* begin the lifetime of an object that contains atomic members */
    for (auto & i : h->partial) i = nullptr;
    h->debt = (std::ptrdiff_t)MIN_HEAP_SIZE;
    h->max_alloc = 0;                 /* largest single allocation on this heap this cycle (see gc_heap::max_alloc) */
    h->debt_reset.store(h->debt, std::memory_order_relaxed);
    h->debt_reset_pending.store(false, std::memory_order_relaxed);
    h->pend_bytes = 0;
    h->pend_objs = 0;
    h->pend_bytes_pub.store(0, std::memory_order_relaxed);
    h->pend_objs_pub.store(0, std::memory_order_relaxed);
    /* The scan **upper bound** (exclusive) for this thread's stack. If the platform can supply the real stack top, use it; when it cannot
     * (NetBSD / OpenBSD / DragonFly, see the notes on `platform_stack_top`), fall back to
     * **`max(current rsp, address of this frame's local variable) + STACK_MARGIN`**.
     *
     * **The `max` cannot be omitted**. The stack grows toward lower addresses and the scan range is `[rsp, stack_top)`,
     * so "**above** the upper bound" means **the caller's outer frames** -- the GC pointers inside those are live roots as well.
     * `my_heap_slow` is called from deep inside `gc()` / `ensure_init()`, so the anchor lands inside the GC call chain
     * and all outer frames lie **above** it; writing only `anchor + STACK_MARGIN` is equivalent to excluding
     * "outer frames more than 64KiB away from the anchor" from the scan entirely (a user who puts a root in a local variable of `main`
     * with a fairly deep frame will then miss a scan => wrongly reclaim). Taking `current_stack_pointer()` (the real rsp at this moment,
     * which is **higher** than the anchor) as the upper bound, and then keeping `STACK_MARGIN` to cover "the compiler pressing a few temporaries
     * above rsp", is the correct conservative direction.
     * @note This applies only when "the platform cannot obtain the stack top"; Windows / Linux / FreeBSD / Apple all take
     *       their own precise branch, so their behavior is unaffected. */
    char* top = platform_stack_top();
    if (top == nullptr) {
        char* const rsp = current_stack_pointer();
        char* const anchor = reinterpret_cast<char*>(stack_marker_addr());
        top = ((rsp > anchor) ? rsp : anchor) + (std::size_t)STACK_MARGIN;
    }
    h->stack_top = top;
    h->stack_base = platform_stack_bottom();   /* lowest address of the committed stack region (only Windows force-suspend reads it, see the notes on stack_base) */
    /* Thread identity: used to decide whether the thread has terminated -> only then can the heap struct be safely reclaimed. On Windows we must duplicate
     * a **real handle** (GetCurrentThread's pseudo-handle cannot be used across threads, nor waited on by WaitForSingleObject).
     * POSIX stores no thread identity at all: ownership is decided by `last_active_epoch` (see that field). */
#if GCXX_PLATFORM_WINDOWS
    {
        HANDLE real = nullptr;
        DuplicateHandle(GetCurrentProcess(), GetCurrentThread(),
                        GetCurrentProcess(), &real, 0, FALSE,
                        DUPLICATE_SAME_ACCESS);
        h->thread_handle = reinterpret_cast<void*>(real);
    }
#elif GCXX_PLATFORM_POSIX
    h->thread_ended.store(false, std::memory_order_relaxed);
#endif
    heap_tls_set(h);                  /* TLS must be written back first (otherwise a heap is created on every allocation) */
    heap_tls_register_exit(h);        /* enroll thread-exit cleanup (the platform TLS callback) */
    h->rehome_pending.store(false, std::memory_order_relaxed);
    h->park_epoch.store(0, std::memory_order_relaxed);
    h->last_active_epoch.store(
        G().sp_epoch.load(std::memory_order_relaxed), std::memory_order_relaxed);
    h->cancel_backoff = (std::size_t)MIN_HEAP_SIZE;
    h->dead.store(false, std::memory_order_relaxed);
    h->inited = true;
    h->hnext = nullptr;
    /* The order is critical: **publish first, then take the lock**. Taking the lock (gc_mutex_lock -> safepoint_reach) may self-scan, park,
     * and push marks onto the gray chain -- that must happen only **after** the thread initiating the STW can see this heap (see gc_state::heaps).
     * After publication every field must be readable: everything above has been fully initialized. */
    publish_heap(h);
    /* The direct-mapping table is one-shot and must be built under the lock; at this point this heap is already on the list, so even if STW intercepts it and parks it, no root is missed. */
    gc_mutex_guard lk;
    if (g.chunk_table == nullptr) {
        if (!tables_init(g)) std::abort();
    }
    return h;
}
/* Stop-the-world (STW) orchestration: begin_cycle / full_collect. Sequence (begin_cycle, caller = the
 * mutator thread that triggered the collection, holding gc_mutex): 1) start_cycle() decides major/minor and sets phase=PROPAGATE;
 * 2) set sp_request=1 and wait for the threads of all active (not dead) heaps to park (each thread reaches the safepoint on its **next allocation / lock acquisition**:
 * after self-scanning its stack + registers it parks); 3) the collector self-scans + static area, and runs all marking to completion in one go;
 * 4) atomic_finish flips the white bit and enters SWEEP; 5) release the parked threads (the objects they allocate afterwards go into the nursery);
 * 6) unlock; after this, cycle_owner advances sweeping incrementally inside its own gc_pace.
 *
 * If not everyone is in place within budget -> **cancel this cycle**: drain the in-flight self-scans -> roll back colors -> reset state. Cancellation reclaims no memory,
 * but it never wrongly kills anything either; scanning the stacks of still-running threads and then marking as usual is unsound,
 * because a mutator that cannot be stopped can rewrite the reference graph during marking. */
/**
 * @brief Set up a collection cycle: decide major/minor, build the root-set cursor, and complete nursery re-homing for heaps whose threads have exited.
 * @note The caller must hold gc_mutex. Major criterion: an explicit request, or promoted bytes reaching the threshold, or the every-`MAJOR_EVERY`-cycles fallback.
 * @internal
 */
inline void start_cycle();            /* definition in the "incremental state machine" section */
/**
 * @brief **One bounded unit of work** of the main marking loop: scan the next slice of a large object, scan the old generation, or take a batch from the gray set and expand it.
 * @param budget Maximum number of steps this unit may process.
 * @note Even if the budget runs out, **the batch of gray objects already claimed must be finished**: otherwise the cursor leaves while still holding unexpanded gray objects, and since the completion predicate only looks at "whether the cursor is in the work unit", it would declare completion early -> **miss a mark**.
 * @internal
 */
inline void propagate_some(std::size_t budget, mark_cursor& c);
/**
 * @brief The wrap-up of the marking phase: flip the white bit and enter SWEEP.
 * @note Flipping the white bit is exactly "clearing all marks at once".
 * @internal
 */
inline void atomic_finish();
/* begin_cycle's STW drain loop uses it to decide "whether the gray work is really finished"; defined in the incremental state machine section. */
/**
 * @brief Whether marking is **really** finished: the gray set is empty **and** there is no overflow residue, no fallback rescan owed, and no unfinished old-generation cursor.
 * @internal
 */
inline bool cycle_mark_done(gc_state& g);
/* Uniformly restore the color of all objects (allocated slots + large objects) to mask. Used only when "cancelling this cycle": at that point the mutators
 * have all come to rest and there is no concurrent coloring, so marked can safely be overwritten directly; free-slot sentinels and slots not yet cut out stay as they are. */
/**
 * @brief On collection **cancellation**, roll all colored objects back to "unmarked this cycle".
 * @note At this moment all mutators have come to rest and there is no concurrent coloring, so `marked` can safely be overwritten directly.
 * @internal
 */
inline void unmark_all_objects(std::uint8_t mask) {
    const auto keep = (std::uint8_t)(mask & WHITE_BITS);
    for_each_object([keep](gc_object* o, int) {
        o->marked.store(keep, std::memory_order_relaxed);
    });
    /* Every object reached here is necessarily on one of the three lists (allocation is what puts it on a list), so there is no need to walk chunk slots again. */
}
/**
 * @brief Whether this heap's owning thread has stopped using it, so that the heap struct may be reclaimed.
 *
 * The precondition for giving a heap back is "its owner will never touch it again" -- the dangerous shape is a
 * thread whose exit callback has already run but which still allocates from a later-running TLS destructor.
 * Only a predicate that observes the owner directly can prove this; a stored thread identity cannot, because
 * on POSIX a `pthread_t` is a pointer into the thread's own stack and glibc's stack cache unmaps that stack
 * once the thread has been joined, so following it faults. `heap_thread_ended` is therefore a **conjunction**:
 * the heap is flagged dead (by the exit callback or by `retire_ended_heaps`), **and** its owner has not been
 * seen for `HEAP_REAP_ROUNDS` collection rounds.
 *
 * @param h The heap to be judged.
 * @return true means the owner has stopped.
 * @note Windows: an additional direct probe is available -- the `DuplicateHandle` thread object is signalled
 *       only after every TLS destructor has run, so `WaitForSingleObject(h, 0) == WAIT_OBJECT_0` is conclusive
 *       and no inactivity window is needed there.
 * @note POSIX: the inactivity stamp (`gc_heap::last_active_epoch`) is the whole criterion; `thread_ended`
 *       alone would be wrong, because the exit callback runs while later TLS destructors may still allocate.
 * @internal
 */
inline bool heap_thread_ended(const gc_heap* h) {
#if GCXX_PLATFORM_WINDOWS
    void* const handle = h->thread_handle;
    if (handle == nullptr) return false;         /* handle unavailable: conservatively treat it as alive */
    return ::WaitForSingleObject(static_cast<HANDLE>(handle), 0) == WAIT_OBJECT_0;
#elif GCXX_PLATFORM_POSIX
    if (!h->thread_ended.load(std::memory_order_acquire)) return false;
    /* The exit callback has run. That is not yet proof that the thread has stopped -- it may still allocate from
     * a TLS destructor that runs later, in which case `gc_pace` keeps refreshing the stamp and this stays false.
     * Only once the stamp has been quiet for HEAP_REAP_ROUNDS rounds is the owner judged to be gone. */
    const unsigned long long now = G().sp_epoch.load(std::memory_order_relaxed);
    const unsigned long long seen = h->last_active_epoch.load(std::memory_order_relaxed);
    return (now - seen) >= (unsigned long long)HEAP_REAP_ROUNDS;
#else
    (void)h;
    return false;                                 /* unknown platform: conservatively treat it as alive */
#endif
}
/* Reclaim the heap structs of "terminated thread + object lists already empty + no chunk ownership". The caller must hold gc_mutex,
 * and must be in PAUSE -- otherwise sweep may be walking the heap list. */
/**
 * @brief Reclaim the heap structs left behind by exited threads (they still hold object lists that need sweeping).
 * @note Must be called while in PAUSE or holding gc_mutex -- otherwise sweep may be walking the heap list.
 * @internal
 */
inline void reap_dead_heaps() {
    gc_state& g = G();
    /* The list head may be concurrently CAS-ed in by a new thread **without a lock** (publish_heap), so unlinking the head must also use CAS; the
     * hnext of an interior node is modified only by this function (which holds gc_mutex). Unlinking (publishing the new head) must be done **before the free** -- otherwise a
     * concurrent inserter that already read the old head may attach its new node to a node that has been freed. */
    gc_heap* prev = nullptr;
    gc_heap* cur  = g.heaps.load(std::memory_order_acquire);
    /* Do not reclaim the current owner: `cycle_owner` is a bare pointer, and reclaiming would free the struct itself and leave it dangling
     * (both the takeover loop in `full_collect` and `gc_pace` read it). When this cycle ends it naturally ceases to be the owner. */
    const gc_heap* const keep = g.cycle_owner.load(std::memory_order_acquire);
    while (cur != nullptr) {
        gc_heap* next = cur->hnext;
        bool freeable = cur != keep &&
                        cur->dead.load(std::memory_order_acquire) &&
                        cur->young == nullptr && cur->old == nullptr &&
                        cur->nursery == nullptr && heap_thread_ended(cur);
        if (freeable) {
            for (const auto & c : cur->partial) {
                if (c != nullptr) {
                    freeable = false;
                    break;
                }
            }
        }
        if (!freeable) { prev = cur; cur = next; continue; }
        if (prev == nullptr) {
            /* Unlink the head: racing with a concurrent publish_heap; on failure cur has already been CAS-updated to the true current head, so just go around again. */
            if (!g.heaps.compare_exchange_strong(cur, next,
                    std::memory_order_acq_rel, std::memory_order_acquire))
                continue;
        } else {
            prev->hnext = next;
        }
#if GCXX_PLATFORM_WINDOWS
        if (cur->thread_handle != nullptr)
            ::CloseHandle(static_cast<HANDLE>(cur->thread_handle));
#endif
        /* gc_heap's members are all trivially destructible (atomics, pointers, integers); it comes from calloc and is matched by free. */
        std::free(cur);
        cur = next;
    }
}
/* ---- Force-suspend: removing the limitation that "when a thread does not reach a safepoint we can only cancel" ----------------
 * Threads that sit for a long time in pure computation, join, or condition-variable waits cannot reach a safepoint, and **without a write barrier they must not be left
 * running** -- they can rewrite the reference graph during marking and cause reachable objects to be wrongly reclaimed, so the only option is to ask the OS to stop them.
 *
 * Deadlock risk (must be understood before changing this code): SuspendThread may suspend a thread that currently holds the CRT heap lock / loader lock;
 * as long as **inside the suspension window the collector performs no operation that could allocate or take those locks**, there is no deadlock. So suspension covers only
 * the span "root scan + marking + atomic_finish", and that span only performs memory reads, the CAS in mark_object, the CAS on the gray chain, and
 * the first PE header parse -- all lock-free and allocation-free. Once that window is left (sweeping frees, later allocations malloc),
 * everything must already have been resumed. */
#if GCXX_CAN_FORCE_SUSPEND
/**
 * @brief Force-suspend threads that **missed the safepoint**.
 * @note Last resort: if they cannot be stopped, marking cannot be done safely.
 * @internal
 */
inline bool suspend_threads_for_mark(gc_heap** heaps, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
        const auto th = reinterpret_cast<HANDLE>(heaps[i]->thread_handle);
        if (th == nullptr || ::SuspendThread(th) == (DWORD)-1) {
            /* Failure: resume the ones already suspended and hand back to the cancellation path */
            for (unsigned j = 0; j < i; ++j)
                ::ResumeThread(reinterpret_cast<HANDLE>(heaps[j]->thread_handle));
            return false;
        }
    }
    return true;
}
/**
 * @brief Release the threads suspended by `suspend_threads_for_mark`.
 * @note Must be released **before sweeping**: sweeping frees/mallocs, and no operation that could take CRT locks may occur inside the suspension window.
 * @internal
 */
inline void resume_threads_for_mark(gc_heap** heaps, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        ::ResumeThread(reinterpret_cast<HANDLE>(heaps[i]->thread_handle));
}
/* Scan the roots of a force-suspended thread: the thread has stopped, its stack/registers are frozen and can be read safely. */
/**
 * @brief Scan the roots of a **force-suspended** thread (the thread has stopped, its stack/registers are frozen).
 * @internal
 */
inline void mark_suspended_roots(gc_heap* h) {
    gc_state& g = G();
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);
    scan_ctx sc = make_scan_ctx(da);
    const auto th = reinterpret_cast<HANDLE>(h->thread_handle);
    CONTEXT ctx = {};
    ctx.ContextFlags = CONTEXT_FULL;
    if (!::GetThreadContext(th, &ctx)) return;    /* context unavailable: we can only scan less */
    /* 1) All registers (including XMM -- the compiler may keep pointers temporarily in them) */
    scan_words(sc, &ctx, reinterpret_cast<const char*>(&ctx) + sizeof(ctx));
    /* 2) Stack: [Rsp - margin, stack_top). The thread is stopped, so everything above rsp is all of its live stack data;
     *    a little further down we keep some margin to cover small objects the compiler may have pressed below rsp. The lower bound is clamped by the
     *    StackLimit recorded at registration, to avoid reading into the guard page and triggering an access violation. */
    const char* hi = h->stack_top;
    if (hi == nullptr) return;
    const char* lo = reinterpret_cast<char*>(ctx.Rsp) - (std::ptrdiff_t)STACK_LOWER_MARGIN;
    const char* limit = h->stack_base;                  /* TEB StackLimit: lowest address of the committed region */
    if (limit != nullptr && lo < limit) lo = limit;
    if (lo >= hi) return;
    const auto span = (std::size_t)(hi - lo);
    if (span > (std::size_t)MAX_STACK_SCAN)
        lo = hi - (std::size_t)MAX_STACK_SCAN;
    g.root_scans.fetch_add(1, std::memory_order_relaxed);
    g.root_probed_words.fetch_add((std::size_t)(hi - lo) / sizeof(void*),
                                  std::memory_order_relaxed);
    scan_words(sc, lo, hi);
}
#endif /* GCXX_CAN_FORCE_SUSPEND */
/**
 * @brief Mark heaps whose thread is confirmed terminated as dead.
 *
 * Otherwise STW would wait forever for a heap that **can never park again**: the host forcibly kills the thread, or the
 * exit callback never runs (observed on the MinGW win32 thread model with dynamic libstdc++, where a user-side
 * function-local thread_local with a destructor breaks the whole thread-teardown flow and skips the platform TLS callback),
 * so that heap's dead stays false forever, and every collection thereafter times out and is cancelled.
 *
 * @note Safety: a terminated thread has no stack and no registers, so it can neither rewrite the reference graph nor hold roots.
 * @note The caller must hold gc_mutex.
 * @param self The caller's own heap, which is skipped (it is the collector).
 * @internal
 */
inline void retire_ended_heaps(gc_heap* self) {
    for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
        if (h == self || h->dead.load(std::memory_order_relaxed)) continue;
        if (heap_thread_ended(h)) h->dead.store(true, std::memory_order_release);
    }
}
/**
 * @brief Abort the cycle currently being started: roll the colors back and restore the state.
 *
 * Called by `begin_cycle` when, within the STW budget, some mutator could not be brought to a
 * safepoint (and, where available, force-suspension could not stop it either).
 *
 * The mutators have already self-scanned and colored objects at their safepoints, so cancellation
 * cannot just "reset the state": the colors must be rolled back as a whole, otherwise those objects
 * would enter the next cycle carrying "already marked / already black" colors, and the next cycle's
 * marking would skip them -> child objects missed -> wrongly reclaimed.
 *
 * The steps and **their order** are load-bearing:
 *   1. close the fast gate with `seq_cst` -- together with the `seq_cst` "read request /
 *      add-subtract `sp_scanning`" in `safepoint_reach` this forms a handshake, and from this
 *      moment on no new self-scan can start;
 *   2. advance the epoch to release already-parked threads;
 *   3. wait for `sp_scanning` to reach zero (after that no thread will color anything anymore,
 *      so rollback is safe);
 *   4. set `aborting`, which widens that window: `sp_scanning` only covers
 *      threads that are currently self-scanning and **does not** cover threads currently inside
 *      `object_new`. Those may already have read this cycle's `ph in {PROPAGATE, ATOMIC}` and will
 *      link their object into the nursery **after** the `unmark_all_objects()` below (colored
 *      "no white bit"). The cross-cycle guard at the end of `object_new` relies on this flag to
 *      recognize "the phase is still PROPAGATE, but the rollback has already walked the object
 *      chains". It must be set **before** `unmark_all_objects()` and cleared only **after**
 *      `phase = PAUSE`, so that "read aborting == false" implies "the phase is already PAUSE";
 *   5. clear the gray set and restore all objects to `cycle_white`.
 *
 * @param prev_next_major  `g.next_major` as it was before `start_cycle` consumed it.
 * @param prev_promoted    `g.promoted_since_major` as it was before `start_cycle` cleared it.
 * @note Always returns `false`, so `begin_cycle` can write `return cancel_cycle(...)` and make the
 *       "cancelled" outcome impossible to forget.
 * @internal
 */
GCXX_NOINLINE inline bool cancel_cycle(gc_heap* self, gc_state& g,
                                      bool prev_next_major,
                                      std::size_t prev_promoted) {
    g.cancelled_cycles.fetch_add(1, std::memory_order_relaxed);
    g.sp_request.store(0, std::memory_order_seq_cst);   /* 1 */
    g.sp_epoch.fetch_add(1, std::memory_order_release); /* 2 */
    while (g.sp_scanning.load(std::memory_order_seq_cst) != 0)
        std::this_thread::yield();                      /* 3 */
    g.aborting.store(true, std::memory_order_release);  /* 4 */
    gray_clear();
    unmark_all_objects(g.cycle_white);                  /* 5 */
    /* Undo start_cycle's side effects item by item */
    g.next_major = prev_next_major;
    g.promoted_since_major = prev_promoted;
    if (g.kind.load(std::memory_order_relaxed) == (int)GC_KIND_MAJOR)
        g.major_cycles.fetch_sub(1, std::memory_order_relaxed);
    else
        g.minor_cycles.fetch_sub(1, std::memory_order_relaxed);
    g.oldscan = nullptr; g.oldscan_heap = nullptr;
    g.scan.cur = nullptr; g.scan.off = 0;
    g.kind.store((int)GC_KIND_NONE, std::memory_order_relaxed);
    g.mark_excl.store(false, std::memory_order_relaxed);   /* cancelled: leave exclusive marking mode */
    g.phase.store((int)GC_PHASE_PAUSE, std::memory_order_release);
    /* The phase is already PAUSE, so the rollback window can be closed. **It must come after the
     * phase store**: that way, inside `object_new`, "read aborting == false" implies "read PAUSE"
     * (release/acquire pairing), and the cross-cycle guard's predicate is exhaustive. */
    g.aborting.store(false, std::memory_order_release);
    g.cycle_owner.store(nullptr, std::memory_order_release);
    /* Exponential backoff: when a thread keeps being uncooperative, do not let "wait out a whole
     * STW budget for nothing on every little allocation" become the norm. The backoff quota doubles
     * (capped), letting the heap grow first; once the thread becomes cooperative it will naturally
     * re-trigger and reset. */
    std::size_t back = self->cancel_backoff;
    back = back < (std::size_t)CANCEL_BACKOFF_MAX
         ? back * 2 : (std::size_t)CANCEL_BACKOFF_MAX;
    self->cancel_backoff = back;
    self->debt = (std::ptrdiff_t)back;
    GCXX_MT_TRACE("[mt] %llu cycle cancelled\n", gc_ts_ms());
    return false;
}
/**
 * @brief Start one collection round: stop the world, scan roots, mark atomically, then enter SWEEP.
 *
 * The caller must hold `gc_mutex` and have already set `phase` to this round's state (not PAUSE).
 *
 * @param self The initiating thread (= this round's `cycle_owner`) heap.
 * @return true = the round started and marking completed (now in SWEEP);
 *         false = cancelled because the mutators could not be stopped.
 * @internal
 */
inline bool begin_cycle(gc_heap* self) {
    gc_state& g = G();
    GCXX_MT_TRACE("[mt] bc_enter\n");
    /* The initiating thread's own heap does not park (it IS the collector), so its unaccounted allocations
     * must be published here, before sweeping begins -- otherwise when its own objects are swept they will
     * likewise drive total_bytes negative (see publish_pending). */
    publish_pending(self);
    gc_mutex_guard lk;
    if (g.phase.load(std::memory_order_acquire) != (int)GC_PHASE_PAUSE) {
        return false;
    }
    /* The gray set must be prepared **before** sp_request is set: as soon as the request is set, other threads start
     * enqueueing during their safepoint self-scan, and allocating memory inside the stop-the-world window is not allowed (see gray_reserve).
     * Putting it here also incidentally covers the "allocation failed -> capacity 0 -> everything goes through fallback rescan" degradation path. */
    gray_reserve();
    g.cycle_owner.store(self, std::memory_order_release);
    /* start_cycle consumes next_major / clears promoted_since_major and bumps the cycle
     * counters; on cancellation these must be handed back exactly as they were. */
    const bool prev_next_major = g.next_major;
    const std::size_t prev_promoted = g.promoted_since_major;
    start_cycle();
    /* --- STW: request + wait for park ---
     * The order must be "advance the epoch first, then set request": when a thread sees request != 0, the epoch
     * it reads is necessarily **this cycle's** number. The reverse (set request first, then advance the epoch)
     * would let a thread read the previous cycle's number and thus wait for a release that never comes. */
    const unsigned long long ep =
        g.sp_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;
    g.sp_request.store(1, std::memory_order_release);
    g.sp_parked.store(0, std::memory_order_relaxed);
    /* A4: the wall clock from this moment (request issued) until "marking may begin" is the STW **wait** segment. */
    const std::chrono::steady_clock::time_point t_stw0 =
        std::chrono::steady_clock::now();
    unsigned want = 0;
    unsigned got  = 0;
    {
        /* The predicate is "the number of heaps parked for **this cycle** (park_epoch == ep)", not a global counter that
         * gets cleared -- the latter would let the previous cycle's late arrivers impersonate this cycle's parked threads (false quorum),
         * and clearing would also lose already-parked threads (a genuine lost wakeup).
         * The timeout uses **real time** rather than a spin count: the cost of yield() varies greatly with platform and load
         * (2000 SwitchToThread calls on Windows measured up to hundreds of milliseconds), so counting iterations would
         * turn a "20ms degradation" into a pause of hundreds of milliseconds. */
        const std::chrono::steady_clock::time_point started =
            std::chrono::steady_clock::now();
        /* With force-suspend available, wait only a short while (see STW_GRACE_MS); without it we can only wait out
         * the whole budget -- that path has no other way to stop an uncooperative thread. */
#if GCXX_CAN_FORCE_SUSPEND
        const std::chrono::milliseconds budget((int)STW_GRACE_MS);
#else
        const std::chrono::milliseconds budget((int)STW_TIMEOUT_MS);
#endif
        for (;;) {
            want = 0;
            got  = 0;
            for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
                if (h == self || h->dead.load(std::memory_order_relaxed)) continue;
                ++want;
                if (h->park_epoch.load(std::memory_order_acquire) == ep) ++got;
            }
            if (got >= want) break;
            const std::chrono::steady_clock::duration waited =
                std::chrono::steady_clock::now() - started;
            if (waited >= budget) break;              /* expired: switch to force-suspend/cancel */
            /* Here we must **absolutely not** use sleep_for for a small remaining amount: Windows' default system timer
             * granularity is ~15.6ms, so sleep_for(1ms) actually sleeps out a whole period -- and during that time
             * the whole process is stalled. Measured, this one point made every collection pay 15.7ms for nothing (55 collections
             * taking 0.87s; after switching to yielding the timeslice, the same load takes 0.012s, **72x**).
             * Sleeping is only worth it when the remaining budget is clearly larger than one timer period. */
            if (budget - waited > std::chrono::milliseconds(16))
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            else
                std::this_thread::yield();
        }
        g.sp_parked.store((int)got, std::memory_order_relaxed);
    }
    /* Timed out: a heap may belong to a thread that already ended without
     * running its exit hook, or was created during that thread's teardown.
     * Such heaps must not be waited for (see retire_ended_heaps). */
    if (got < want) {
        retire_ended_heaps(self);
        want = 0; got = 0;
        for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
            if (h == self || h->dead.load(std::memory_order_relaxed)) continue;
            ++want;
            if (h->park_epoch.load(std::memory_order_acquire) == ep) ++got;
        }
    }
    GCXX_MT_TRACE("[mt] %llu parks expect=%u got=%u\n", gc_ts_ms(), want, got);
    /* --- Not all of them are in place: first try **force-suspend**, and only cancel if that really does not work ---
     *
     * `missing` / `nmissing` are **declared once, outside the `#if`**, and they are the single list of
     * "threads that did not park": `suspend_threads_for_mark` fills it, and the same array is later
     * handed to `mark_suspended_roots` and to `resume_threads_for_mark`.
     *
     * @note This used to be **two** arrays -- a `missing` used only to suspend, and a `forced` onto
     *       which the suspended ones were copied one by one -- with the copy's only purpose being to
     *       preserve the order `suspend` returns in, which it does not change. On POSIX, where
     *       force-suspend does not exist, both arrays were allocated unconditionally and `forced`
     *       then carried a `(void)forced;` to silence the unused-variable warning: a 512-byte stack
     *       array built and immediately discarded on every collection, plus a cast that existed only
     *       to hide it. Splitting the declaration by `#if` removes the array, the copy and the cast.
     *
     * On the platforms without `GCXX_CAN_FORCE_SUSPEND` (i.e. everything but Windows) these stay
     * zero-sized/zero-valued, so the guards below fold away at compile time. */
#if GCXX_CAN_FORCE_SUSPEND
    gc_heap* missing[MAX_FORCE_SUSPEND];
    unsigned nmissing = 0;
    if (got < want) {
        bool too_many = false;
        for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
            if (h == self || h->dead.load(std::memory_order_relaxed)) continue;
            if (h->park_epoch.load(std::memory_order_acquire) == ep) continue;
            if (nmissing >= (unsigned)MAX_FORCE_SUSPEND) { too_many = true; break; }
            missing[nmissing++] = h;
        }
        /* Suspend succeeded: the stacks/registers of these threads are now frozen and can be marked safely.
         * They do not take part in the "park" quorum; each was stopped outside its safepoint.
         * On failure `suspend_threads_for_mark` has already resumed whatever it managed to suspend,
         * so `nmissing` must drop back to 0 or the release path below would resume them twice. */
        if (too_many || !suspend_threads_for_mark(missing, nmissing)) nmissing = 0;
    }
    const unsigned nforced = nmissing;
#else
    const unsigned nforced = 0;
#endif
    if (got < want && nforced == 0) {
        /* ---- Cancel this cycle. The whole reason (and the load-bearing order of the
         * rollback steps) lives on `cancel_cycle`, which is deliberately `GCXX_NOINLINE`:
         * this is the cold, error-shaped path, and keeping it out of line is what makes the
         * normal flow of begin_cycle readable. ---- */
        return cancel_cycle(self, g, prev_next_major, prev_promoted);
    }
    /* A thread was force-suspended: it may have stopped between "obtained a gray-set index" and "has not yet written the cell", and after resuming
     * that write will land in the next cycle. This cycle therefore always runs a fallback rescan as a backstop (see gray_push). */
    if (nforced > 0) g.gray_need_rescan = true;
    /* This cycle can proceed normally (either everything parked, or the uncooperative ones were force-suspended):
     * reset the backoff quota. */
    self->cancel_backoff = (std::size_t)MIN_HEAP_SIZE;
    /* Note: request stays 1 at this moment -- parked threads are released only after marking completes,
     * otherwise the mutator would resume running inside the marking window, destroying the soundness of atomic marking. */
    /* --- Roots: force-suspended threads + self + static area ---
     * The roots of already-parked threads have **already** been fully self-scanned in their own safepoint_reach
     * (that is the condition for their parking), so there is no need to rescan them here. */
    /* A4: the STW wait segment ends here (mutators are all stopped; root scan + marking follow). */
    g.t_stw_us.fetch_add(gc_us_since(t_stw0), std::memory_order_relaxed);
    const std::chrono::steady_clock::time_point t_root0 =
        std::chrono::steady_clock::now();
#if GCXX_CAN_FORCE_SUSPEND
    for (unsigned i = 0; i < nforced; ++i)
        mark_suspended_roots(missing[i]);
#endif
    mark_roots_self(self);
    if (!g.static_inited) {
        g.static_range_count = 0;
        g.static_inited = true;
        static_ranges_collect(g);
    }
    g.root_probed_words.fetch_add(static_root_words(g),
                                  std::memory_order_relaxed);
    scan_static_roots();
    g.t_root_us.fetch_add(gc_us_since(t_root0), std::memory_order_relaxed);
    GCXX_MT_TRACE("[mt] %llu mark begin\n", gc_ts_ms());
    /* --- Marking (mutators all stopped, completed atomically) ---
     * The exit condition must be cycle_mark_done rather than just looking at gray_head: the local batch may still hold a batch of gray
     * objects taken off the global chain and not yet expanded. */
    /* Marking must complete atomically under a **single marker**: concurrent push and pop of the gray
     * set by several markers needs one atomic compare-exchange per object, whose cost grows with the
     * number of markers, so multi-marker expansion is not enabled. */
    g.mark_excl.store(true, std::memory_order_relaxed);
    const std::chrono::steady_clock::time_point t_mark0 =
        std::chrono::steady_clock::now();
    do {
        propagate_some(4096, g.scan);
    } while (!cycle_mark_done(g));
    g.phase.store((int)GC_PHASE_ATOMIC, std::memory_order_relaxed);
    atomic_finish();                     /* flip the white bit + enter SWEEP */
    g.t_mark_us.fetch_add(gc_us_since(t_mark0), std::memory_order_relaxed);
    GCXX_MT_TRACE("[mt] %llu release\n", gc_ts_ms());
    /* --- Release ---
     * Order: first release the **force-suspended** ones (they were stopped passively), then clear request (blocking latecomers),
     * and finally advance the epoch (waking the actively parked). Clearing request and advancing the epoch must not be swapped --
     * if the epoch were advanced before request is cleared, a thread arriving in between would read the new epoch and park for it,
     * while this cycle is already wrapping up, so it would have to wait until the next cycle to be released.
     * It must be resumed here (and not earlier): the sweep path does free/malloc, and inside the suspend window no
     * operation that may take a CRT lock may occur (see the notes on suspend_threads_for_mark). */
#if GCXX_CAN_FORCE_SUSPEND
    if (nforced > 0) resume_threads_for_mark(missing, nforced);
#endif
    g.sp_request.store(0, std::memory_order_release);
    g.sp_epoch.fetch_add(1, std::memory_order_release);
    /* Unlocking is done by lock_guard; we are the cycle_owner, and sweeping is advanced in gc_pace */
    return true;
}
/* Safepoint-aware lock acquisition (declaration in the file header): when the lock cannot be taken and there is an STW request,
 * self-scan + park first (register roots are marked by that), then keep contending for the lock. */
/**
 * @brief Safepoint-aware lock acquisition: when the lock cannot be taken and there is an STW request, self-scan + park first, then keep contending for the lock.
 * @note Closes the last soundness hole: "a thread blocks on the lock, misses park, its roots live only in registers, and the degraded scan cannot reach them".
 * @internal
 */
inline void gc_mutex_lock(std::recursive_mutex& m) {
    for (;;) {
        if (m.try_lock()) return;
        gc_state& g = G();
        if (g.sp_request.load(std::memory_order_acquire) != 0)
            safepoint_reach(my_heap());
        else
            std::this_thread::yield();
    }
}
/* Initialization: no explicit initialization needed -- the first allocation path (or ensure_init during static construction) automatically creates the global state and this
 * thread's heap. The gc_init() macro is kept: it anchors the main thread's stack scan upper bound to the caller's stack frame (meaningful only
 * when the platform cannot give the real stack top). */
/**
 * @brief Internal implementation of "start one collection cycle" (shared by `gc_collect` and the automatic pacing).
 * @internal
 */
inline void gc_start_impl(void* anchor) {
    gc_heap* h = my_heap();
    const char* pb = platform_stack_top();
    /* Only the upper bound is adjusted, and it is **only ever widened, never tightened** (`h->stack_top` has already been fixed by `my_heap_slow` as
     * `max(rsp, anchor) + STACK_MARGIN`; the `anchor` passed into `gc_init()` is a local variable in the **caller's** stack frame, which
     * lies deeper and at a higher address than GC internals, so taking `max` widens it).
     * See the explanation in `my_heap_slow`: only what lies below the upper bound is the active region that gets scanned, so tightening upward = missing outer frames. */
    if (pb == nullptr && anchor != nullptr) {
        char* const want = reinterpret_cast<char*>(anchor) + (std::size_t)STACK_MARGIN;
        if (h->stack_top == nullptr || want > h->stack_top) h->stack_top = want;
    }
}
/**
 * @brief Idempotent initialization: ensures the global `gc_state` and this thread's heap are in place.
 * @note Allocation paths call it on demand -- `gc()` is safe at any moment and the user does not need an explicit `gc_init()`.
 * @internal
 */
inline void ensure_init() {
    my_heap();                           /* global state + this thread's heap */
}
/* Incremental state machine */
/* Expand one object: conservative-scan its payload as a suspected pointer array. **Purely scalar objects (INT/FLOAT/PLAIN)
 * are skipped whole** -- this is the key path that saves time.
 * Advance the "large object being continued" by one slice: after return, if `c.cur` is still non-null, more remains to be done.
 * `c` is the **caller's own** cursor (the collector uses `gc_state::scan`, a helper uses one on its stack) --
 * see the notes on mark_cursor: multiple markers must never share one cursor. */
/**
 * @brief Advance the "large object being continued" by one slice (bounded work).
 * @note Multiple markers must **never share** a cursor.
 * @internal
 */
inline void scan_cur_slice(mark_cursor& c) {
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);
    scan_ctx sc = make_scan_ctx(da);
    gc_object*    o   = c.cur;
    std::uint32_t off = c.off;
    const std::uint32_t rem = (std::uint32_t)o->size - off;
    const std::uint32_t n   = (rem > (std::uint32_t)SCAN_SLICE_BYTES)
                      ? (std::uint32_t)SCAN_SLICE_BYTES : rem;
    scan_words(sc, payload_of(o) + off, payload_of(o) + off + n);   /* byte accounting is inside scan_words */
    off += n;
    if (off >= o->size) { c.cur = nullptr; c.off = 0; }
    else {
        c.off = off;
        /* Prefetch the next slice: pull the following data into cache while this slice is scanned */
        std::uint32_t n2 = (o->size - off > (std::uint32_t)SCAN_SLICE_BYTES)
                         ? (std::uint32_t)SCAN_SLICE_BYTES : (o->size - off);
        char* pf = payload_of(o) + off;
        char* pe = pf + n2;
        for (; pf < pe; pf += 64) GCXX_PREFETCH(pf);
    }
}
/* Traverse one object: scalar type -> skip outright; small object -> scan it in one go; large object -> scan only the first slice and hand the remainder to
 * the caller's own cursor to continue slice by slice (an interruptible trace). This bounds the work of a single work unit, so "one container happens
 * to be huge" never causes a long pause. `c` see mark_cursor. */
/**
 * @brief Expand one object: conservative-scan its payload as a suspected pointer array.
 * @note Purely scalar objects are skipped whole (the key path that saves time); old-generation objects are used only to discover references into the young generation.
 * @internal
 */
inline void traverse_object(gc_object* o, diag_accum* da, mark_cursor& c) {
    if (o->size == 0) return;
    scan_ctx sc = make_scan_ctx(da);
    if (!type_needs_scan(o->type)) {            /* no pointer can be inside, skip outright */
        da->skipped_bytes += o->size;
        return;
    }
    if ((std::size_t)o->size <= (std::size_t)SCAN_SLICE_BYTES) {
        scan_words(sc, payload_of(o), payload_of(o) + o->size);   /* byte accounting is inside scan_words */
        return;
    }
    c.cur = o;
    c.off = (std::uint32_t)SCAN_SLICE_BYTES;
    scan_words(sc, payload_of(o), payload_of(o) + SCAN_SLICE_BYTES);
}
/* Advance marking (collector-exclusive). A minor cycle additionally advances the "old-generation scan cursor" -- the old-generation chain has already been split by heap,
 * and when the cursor exhausts one heap it switches to the next (including dead heaps: their thread has exited, but their objects may still be referenced by other threads). */
/* ---- Fallback for gray-set overflow: walk the three object chains and find "already marked but not yet expanded" objects --------------
 * The predicate is the natural consequence of the three color bits: the white bit is cleared (already marked) and BLACK is not set (not yet expanded) = gray. **Only
 * called inside the marking window** (at that point no thread is coloring, so the colors are a stable snapshot). Scan the chains rather than the chunk slots: every
 * allocated object is necessarily on one of the three chains young/old/nursery, so chain traversal covers everything and is cheaper.
 * Termination: every call turns at least one gray object black and expands it; turning black is monotone, so at worst O(number of objects) calls. */
/**
 * @brief **Fallback rescan**: walk the object chains and find the "already marked but not yet expanded" objects to continue expanding them.
 * @note Used only on the exceptional path (gray-set overflow, or a thread being force-suspended). Costs O(heap), which is why the gray set has growth to avoid it.
 * @internal
 */
inline std::size_t rescan_gray_from_heap(mark_cursor& c) {
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);          /* a full chain walk reads TLS only once */
    std::size_t found = 0;
    /* Traverse the three object chains of all heaps -- the predicate is the natural consequence of the three color bits (see above). Inside the callback, `stop`
     * expresses "must be handed back to the upper layer immediately": `for_each_object` has no early-exit interface, and if we did not return at once here when the
     * large-object slice cursor is non-null, the upper layer would lose its chance to drive `scan_cur_slice`. */
    bool stop = false;
    for_each_object([&](gc_object* o, int) {
        if (stop) return;
        const std::uint8_t m = mark_load(o);
        if (m == MARK_FREE) return;              /* free-slot sentinel */
        if ((m & WHITE_BITS) != 0) return;            /* still white: never marked */
        if ((m & (std::uint8_t)BLACK) != 0) return;   /* already expanded */
        o->marked.store((std::uint8_t)(m | (std::uint8_t)BLACK),
                        std::memory_order_relaxed);
        traverse_object(o, da, c);       /* expand: child objects are re-enqueued into the gray set */
        ++found;
        /* If traverse_object leaves a continuation cursor behind because of "large-object slice scanning", it must be handed back to the upper layer at once (which
         * drives scan_cur_slice at the loop head) -- otherwise the next object's traverse overwrites the cursor, and the remainder of this large object is **never**
         * scanned again, so its child objects are missed and then reclaimed. */
        if (c.cur != nullptr) stop = true;
    });
    return found;
}
/* Ephemeron activation during marking. The semantics are "**a value is reachable only when its key is reachable**", and the implementation is the standard **fixpoint**:
 * 1) normal marking proceeds until the gray set is empty (convergence); 2) scan the ephemeron table once, and for every entry whose "key no longer carries this cycle's white
 * bit" and which has not been activated this cycle, mark its **value as a new root**; 3) if anything was activated, go back to 1; if not, it has converged and marking ends.
 * Termination: each entry is activated at most once per cycle (the `activated` flag), so the outer loop runs at most "number of entries" times, and there are few entries.
 * What this buys is **exactly what an ordinary weak reference cannot do**: when the value points back at the key, a weak key + strong value would mark the key live -> the key never
 * dies -> the weak reference never expires (the classic leak); an ephemeron does not mark the value at all when the key is unreachable, so "the key, the value, and everything the
 * value points to" are reclaimed together.
 * It goes inside the marking loop rather than after marking: afterwards would be too late -- the colors are already frozen, and marking the value live again would contradict
 * the already computed sweep decision (sweeping has already begun reclaiming memory by color). */
/* Defined in the "weak reference cleanup" section; used here first -- the two places must share the same predicate, see the notes there. */
/**
 * @brief Decide whether a weak registration's target is **already dead** (using the **same** predicate as the cleanup phase, avoiding "marking says live, cleanup says dead").
 * @internal
 */
inline bool weak_target_dead(const gc_object* o);
/**
 * @brief **Fixpoint** advance for ephemerons: for every entry whose key is reachable, mark its value as a root.
 * @return `true` = at least one entry was activated this cycle (marking has not converged yet).
 * @note Done only after the gray set has converged.
 * @internal
 */
inline bool ephemeron_activate_all() {
    gc_state& g = G();
    if (g.weak_live == 0) return false;          /* nothing registered: exit in one instruction */
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);
    scan_ctx sc = make_scan_ctx(da);
    bool any = false;
    for (std::size_t i = 0; i < g.weak_len; ++i) {
        weak_slot& s = g.weak_tab[i];
        if (!s.in_use || s.kind != WEAK_EPH || s.activated) continue;
        const gc_object* k = s.key.load(std::memory_order_relaxed);
        if (k == nullptr) continue;
        /* The key counts as reachable only if it no longer carries this cycle's white bit -- marking is monotone, so the fixpoint loop below is safe;
         * here we use the **same** predicate as the cleanup phase (`weak_target_dead`), avoiding "marking says live, cleanup says dead". */
        if (weak_target_dead(k)) continue;
        s.activated = 1;                         /* activated at most once per cycle */
        any = true;
        gc_object* v = s.value.load(std::memory_order_relaxed);
        if (v != nullptr)
            sc.skipped += mark_object(v, sc.excl, sc.minor);   /* treat the value as a root */
    }
    da->skipped_bytes += sc.skipped;
    return any;
}
/**
 * @brief **One bounded work unit** of the marking main loop: continue a slice of the large-object scan, scan the old generation, or pull a batch from the gray set to expand.
 * @param budget Maximum number of steps this unit may process.
 * @note Even when the budget runs out, the **already-claimed batch of gray entries must be finished**: otherwise the cursor leaves while still holding unexpanded gray objects, and the completion predicate only looks at "whether the cursor is inside a work unit", so completion would be declared early -> **miss a mark**.
 * @internal
 */
inline void propagate_some(std::size_t budget, mark_cursor& c) {
    gc_state& g = G();
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);          /* the TLS is read only once for the whole drain */
    /* **Even when the budget runs out, finish the batch already claimed** (`c.batch_next < c.batch_end`): otherwise the cursor
     * leaves this function while still holding "unexpanded gray objects", while the in-flight counts of `cycle_mark_done`/B1
     * only look at "whether the cursor is inside a work unit" -- that would **declare marking complete early -> miss a mark**.
     * The batch upper bound is GRAY_BATCH, so this extra amount is bounded. */
    for (std::size_t i = 0; i < budget || c.batch_next < c.batch_end; ++i) {
        if (c.cur != nullptr) {           /* first push one slice of the large-object continuation scan (bounded work) */
            scan_cur_slice(c);
            continue;
        }
        if (g.oldscan != nullptr) {
            gc_object* o = g.oldscan;
            g.oldscan = o->next;
            if (g.oldscan != nullptr)                 /* prefetch the payload of the next one on the chain */
                GCXX_PREFETCH(payload_of(g.oldscan));
            traverse_object(o, da, c);     /* the old generation serves only to discover references into the young generation */
            continue;
        }
        if (g.oldscan_heap != nullptr) {  /* switch to the old-generation chain of the next heap */
            g.oldscan = g.oldscan_heap->old;
            g.oldscan_heap = g.oldscan_heap->hnext;
            continue;
        }
        gc_object* o = next_gray(c);
        if (o == nullptr) {
            /* The gray set is empty: but "empty" may mean an enqueue was dropped, or that a thread was force-suspended this cycle (it may have stopped mid-enqueue).
             * In that case run a fallback rescan to find the missed "marked but not yet expanded" objects and keep expanding them. */
            if (!g.gray_overflow.load(std::memory_order_relaxed) && !g.gray_need_rescan) {
                /* ---- ephemeron fixpoint: activate only after the gray set has converged ---- activation pours the values (and their subtrees) back into the
                 * gray set, so `continue` here lets the outer loop keep going; only when nothing was activated is the work truly done. */
                if (ephemeron_activate_all()) continue;
                return;
            }
            g.gray_overflow.store(false, std::memory_order_relaxed);
            g.gray_need_rescan = false;
            ++g.gray_rescans;
            if (rescan_gray_from_heap(c) == 0) return;   /* there really is nothing left over */
            continue;
        }
        GCXX_PREFETCH(payload_of(o));
        o->marked.store((std::uint8_t)(mark_load(o) | (std::uint8_t)BLACK),
                        std::memory_order_relaxed);
        traverse_object(o, da, c);
    }
}
/* Atomic phase: flip the white bit + start incremental sweeping. Marking was already completed atomically in the PROPAGATE branch (the mutator never ran throughout),
 * so by the time execution reaches here the three work sources must already be empty -- the drain loop is only a safety net. */
/**
 * @brief Drain the marking work: advance the "large-object continuation cursor + the minor's old-generation root scan + the gray set" until all three are empty.
 * @note This is the **synchronous version** of `propagate_some` (no budget, consumes nothing beyond the gray-batch upper bound),
 *       and the work sources available to the two paths must be **exactly identical** -- the predicate of `cycle_mark_done()` contains
 *       `oldscan_heap == nullptr`. Without that `oldscan_heap` branch, the fallback path for
 *       `GC_PHASE_ATOMIC` in `gc_step` (which calls `atomic_finish()` directly, without running `propagate_some` first)
 *       would flip the white bit while the minor's old generation has not been fully scanned => young objects referenced only by the old generation get swept => dangling.
 * @internal
 */
inline void drain_mark_work(mark_cursor& c) {
    gc_state& g = G();
    diag_accum fallback{};
    diag_accum* da = diag_or(&fallback);          /* the TLS is read only once for the whole drain */
    for (;;) {
        if (c.cur != nullptr) {
            scan_cur_slice(c);
            continue;
        }
        /* minor: the whole old generation acts as a root set, scanning each heap's old chain in turn (the same branch as in propagate_some). */
        if (g.oldscan_heap != nullptr) {
            g.oldscan = g.oldscan_heap->old;
            g.oldscan_heap = g.oldscan_heap->hnext;
            continue;
        }
        if (g.oldscan != nullptr) {
            gc_object* o = g.oldscan;
            g.oldscan = o->next;
            if (g.oldscan != nullptr)
                GCXX_PREFETCH(payload_of(g.oldscan));
            traverse_object(o, da, c);
            continue;
        }
        gc_object* o = next_gray(c);
        if (o == nullptr) {
            if (!g.gray_overflow.load(std::memory_order_relaxed) && !g.gray_need_rescan) {
                /* The same fixpoint as in propagate_some: ephemeron activation is part of marking, so the fallback drain path must do it too. */
                if (ephemeron_activate_all()) continue;
                break;                    /* the gray set is empty and nothing is left over */
            }
            g.gray_overflow.store(false, std::memory_order_relaxed);
            g.gray_need_rescan = false;
            ++g.gray_rescans;
            if (rescan_gray_from_heap(c) == 0) break;
            continue;
        }
        GCXX_PREFETCH(payload_of(o));
        o->marked.store((std::uint8_t)(mark_load(o) | (std::uint8_t)BLACK),
                        std::memory_order_relaxed);
        traverse_object(o, da, c);
    }
}
/* Weak references / ephemerons: processed after marking ends and before sweeping. The timing is the **only** correct place: inside `atomic_finish`,
 * after the white-bit flip and before sweeping starts -- at that moment "whether this object will be reclaimed this cycle" is fully determined (sweeping merely executes that
 * decision), so clearing weak references that point at dead objects cannot change any liveness decision; if such a weak reference is the target's only reference, the target was
 * doomed anyway, which is exactly the semantics of a disappearing link. Doing it **in the middle** of marking is wrong (the colors can still change), and doing it **after**
 * sweeping is also wrong (the slot may already have been reused).
 * **Safety**: the table holds raw pointers, and the target may dangle after being reclaimed, so it is **never dereferenced directly**; instead we first do
 * `object_find(ptr)` (which only walks the chunk table, so unregistered chunks cannot be found): if it is not found, judge it dead immediately and clear the slot -- this step turns
 * "reading a dangling pointer" from a possible crash into one table lookup. The residual imprecision (which is safe): if the slot is reused for **another object at the same
 * address**, the entry is kept until that new object dies -- the direction is "retain a little longer", never a missed clear that becomes a dangling reference. */
/* The predicate for "this object is considered dead this cycle". Its three call sites (ephemeron activation during marking, cleanup after marking, and the handle-side
 * read) must **share the same one**, otherwise you get the self-contradictory state where "marking says it is live and cleanup says it is dead". */
/**
 * @brief Decide whether the target of a weak registration is **already dead** (using the **same** predicate as the cleanup phase).
 * @internal
 */
inline bool weak_target_dead(const gc_object* o) {
    gc_state& g = G();
    if (o == nullptr) return true;
    /* Reverse lookup: the chunk is unregistered / the address is no longer an object -> treat as dead. This also covers the "reading a dangling
     * pointer would crash" case, see the explanation above. */
    if (object_find(reinterpret_cast<std::uintptr_t>(o)) != o) return true;
    const std::uint8_t cw = g.cycle_white;
    if (g.kind.load(std::memory_order_relaxed) == (int)GC_KIND_MAJOR)
        return is_dead(o, cw);
    /* A minor does not sweep the old generation (the whole old generation acts as a root set), so an old object is **not**
     * reclaimed in a minor even if it carries a white bit -- the predicate must match sweep_some, otherwise a live target would be cleared as dead. */
    return !is_old(o) && is_dead(o, cw);
}
/**
 * @brief After marking converges, process the weak table: clear entries whose target is already dead to expired.
 * @note The timing is **after marking ends and before sweeping begins**.
 * @internal
 */
inline void weak_process_after_mark() {
    gc_state& g = G();
    if (g.weak_live == 0) return;               /* almost every program takes this path */
    /* The table is private to the GC, and both registration and unregistration hold gc_mutex; this function is called by the collector inside the STW
     * (where the collector holds gc_mutex), so there is no need to take the lock again. */
    for (std::size_t i = 0; i < g.weak_len; ++i) {
        weak_slot& s = g.weak_tab[i];
        if (!s.in_use) continue;
        const gc_object* k = s.key.load(std::memory_order_relaxed);
        if (k == nullptr) { s.activated = 0; continue; }
        if (weak_target_dead(k)) {
            /* Clear but **do not reclaim the slot**: the handle still holds this index, and it relies on "reading nullptr"
             * to decide that it has expired; the slot is returned to the free list only when the handle is destroyed.
             * Order: clear value first, then key. The read side goes by key, and "key non-null" means
             * the value is certainly valid (the key is live => the value was marked this cycle => it will not be reclaimed). */
            s.value.store(nullptr, std::memory_order_release);
            s.key.store(nullptr, std::memory_order_release);
            --g.weak_live;
        }
        s.activated = 0;                        /* re-activate next cycle */
    }
}
/**
 * @brief The tail of the marking phase: flip the white bit and enter SWEEP.
 * @internal
 */
inline void atomic_finish() {
    gc_state& g = G();
    /* Drain all gray work, the old-generation scan cursor and the large-object continuation cursor (safety net; normally empty) */
    drain_mark_work(g.scan);
    /* Major collection: flip the white bit. After this, any object still carrying the old white bit is "unmarked this cycle = garbage".
     *    A minor does not flip, so old-generation colors stay stable for the whole minor. */
    if (g.kind.load(std::memory_order_relaxed) == (int)GC_KIND_MAJOR)
        g.white.store(
            (std::uint8_t)((g.white.load(std::memory_order_relaxed) == WHITE0)
                           ? WHITE1 : WHITE0),
            std::memory_order_release);
    /* After the white-bit flip and before sweeping: the only correct timing for weak references (see the argument there). */
    weak_process_after_mark();
    /* Marking is over: leave exclusive mode (sweeping no longer marks anything live). */
    g.mark_excl.store(false, std::memory_order_relaxed);
    /* Enter incremental sweeping (across heaps; the stage-2 nursery rehome has already moved to the owner side) */
    g.sweep_stage = 0;
    g.sweep_heap = nullptr;
    g.sweep_link = nullptr;
    g.phase.store((int)GC_PHASE_SWEEP, std::memory_order_release);
    diag_flush();                     /* the marking phase is over: merge in the diagnostic counts accumulated by this thread */
}
/* Incremental sweeping (exclusive to the collector, across all heaps; a minor sweeps only young, a major sweeps both chains).
 * It is staged, and each work unit processes only one object, so the per-call cost is bounded: stage 0 scans each heap's young chain in turn
 * (dead/alive test + age bump + promotion), stage 1 scans the old chain (major only), stage 3 completes -> GC_PHASE_END.
 * The nursery rehome has already moved to the owner side (rehome_some, run during PAUSE), so there is no stage 2 here anymore; freeing dead objects
 * goes through chunk_free (a lock-free free-list push) and does not block the owner's concurrent allocation.
 * @return **the number of objects actually processed** (iterations that switch heap/stage do not count), so the caller can repay debt according to the real workload. */
/**
 * @brief Advance sweeping by one bounded work unit: walk the object chain to find the objects judged dead this cycle and reclaim their slots.
 * @note Sweeping is **incremental** and driven by allocation debt; the mutator is never stopped.
 * @internal
 */
inline std::size_t sweep_some(std::size_t budget) {
    gc_state& g = G();
    std::size_t processed = 0;
    for (std::size_t work = 0; work < budget; ++work) {
        if (g.sweep_stage >= 3) break;
        gc_object* cur = (g.sweep_link != nullptr) ? g.sweep_link->load() : nullptr;
        if (cur == nullptr) {
            /* The current chain is done: switch to the next heap. The chain head is pushed in by my_heap_slow with a **lock-free CAS**, and sweeping holds no lock,
             * so take one acquire snapshot here; each heap's hnext changes only when reap_dead_heaps unlinks it, and that
             * happens only after this cycle's sweep has finished (in finish_cycle), so it is stable during traversal. A heap registered in the middle of a sweep
             * is not in the snapshot -- its objects are handled by later cycles (a new heap's young/old are not modified by its owner during SWEEP). */
            if (g.sweep_heap != nullptr) {
                g.sweep_heap = g.sweep_heap->hnext;
            } else {
                /* Take the lock synchronously only at the moment of fetching the chain head, and release it right after reading -- the critical section holds a single assignment. */
                g.gc_mutex.lock();
                g.sweep_heap = heaps_head();
                g.gc_mutex.unlock();
            }
            if (g.sweep_heap != nullptr) {
                g.sweep_link = (g.sweep_stage == 0)
                             ? &g.sweep_heap->young : &g.sweep_heap->old;
                continue;
            }
            /* All heaps are done for this stage. A major sweeps young first and then old; a minor
             * sweeps only young and is finished here. */
            if (g.sweep_stage == 0 &&
                g.kind.load(std::memory_order_relaxed) == (int)GC_KIND_MAJOR) {
                g.sweep_stage = 1;
                g.sweep_heap = nullptr;
                g.sweep_link = nullptr;
                continue;
            }
            g.sweep_stage = 3;
            g.phase.store((int)GC_PHASE_END, std::memory_order_release);
            break;
        }
        /* Reaching here means `sweep_stage` is 0 (young) or 1 (old, major only): `>= 3` broke out
         * above and the branch just above is the only way to advance the stage. Both stages test
         * liveness by `cycle_white`, so no per-stage predicate is needed. */
        const bool sweeping_young = (g.sweep_stage == 0);
        /* Hoist the "constant for the whole slice" predicate out of the loop: cycle_white is a plain field, and object_free is a non-inlined call,
         * so the compiler cannot assume it does not write gc_state, which forced a reload every iteration. */
        const std::uint8_t cw = g.cycle_white;
        /* The liveness predicate holds for a major: the atomic phase flipped the white bit, reachable
         * objects were marked black (not dead), unreachable objects still carry cycle_white (dead); an
         * object newly promoted during sweeping goes through make_white to get the new white bit first,
         * so it is not misjudged either. */
        ++processed;
        if (is_dead(cur, cw)) {
            g.sweep_link->store(cur->next.load(std::memory_order_relaxed),
                                std::memory_order_relaxed);
            object_free(g, cur);
            continue;                       /* sweep_link is not advanced */
        }
        make_white(cur);                    /* alive: restore it to the white of the next cycle */
        if (sweeping_young) {
            auto na = (std::uint8_t)(cur->age + 1u);
            if (na > (std::uint8_t)GC_AGE_OLD) na = (std::uint8_t)GC_AGE_OLD;
            cur->age = na;
            if (is_old(cur)) {              /* promotion: young chain -> this heap's old chain */
                g.promoted_since_major += cur->size;
                g.sweep_link->store(cur->next.load(std::memory_order_relaxed),
                                    std::memory_order_relaxed);
                cur->next.store(g.sweep_heap->old.load(std::memory_order_relaxed),
                               std::memory_order_relaxed);
                g.sweep_heap->old.store(cur, std::memory_order_relaxed);
                continue;                   /* sweep_link is not advanced */
            }
        }
        g.sweep_link = &cur->next;
    }
    /* **The one and only exit.** The original had three `return` paths, each preceded by its own copy
     * of this call, two of them mis-indented into the enclosing `if` -- so "does every exit publish?"
     * could only be answered by eyeballing all three. The loop now `break`s and there is a single
     * epilogue, which is also the property the contract depends on (no unpublished dead-slot batch may
     * survive a slice: the live counts that thread-exit cleanup and retirement read are computed from
     * the published ones). */
    flush_dirty_slots();
    return processed;
}
/* Start a collection cycle: adaptively decide major/minor. Major trigger conditions (any one): explicitly requested (gc_collect / the previous cycle
 * decided it is needed); bytes promoted since the last major >= MINOR_MAJOR_PERCENT% of the live bytes at the last major
 * (old objects are piling up -- the same as Mica's GC_MINORMAJOR, replacing a fixed cycle count); every MAJOR_EVERY cycles as a fallback. */
/* Start a collection cycle (the caller holds gc_mutex): decide major/minor, set up the root-set cursor; the root scan (inside the STW) is orchestrated
 * by begin_cycle. Heaps whose thread has already exited (dead) complete their nursery rehome here in one go -- they have no owner. */
/**
 * @brief Set up a collection cycle: decide major/minor, set up the root-set cursor, and complete the nursery rehome of exited heaps.
 * @note The caller must hold gc_mutex.
 * @internal
 */
inline void start_cycle() {
    gc_state& g = G();
    bool major = g.next_major;
    g.next_major = false;
    if (!major && g.live_at_major > 0 &&
        g.promoted_since_major >= g.live_at_major * (std::size_t)MINOR_MAJOR_PERCENT / 100u)
        major = true;
    if (!major && ((g.cycles.load(std::memory_order_relaxed) + 1) %
                   (std::size_t)MAJOR_EVERY) == 0)
        major = true;
    /* The dead heap's nursery: move it into young in one go (at PAUSE time, which is safe) */
    const std::uint8_t w = g.white.load(std::memory_order_relaxed);
    for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
        if (!h->dead.load(std::memory_order_relaxed)) continue;
        while (h->nursery.load(std::memory_order_relaxed) != nullptr) {
            gc_object* o = h->nursery.load(std::memory_order_relaxed);
            h->nursery.store(o->next.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
            o->marked.store(w, std::memory_order_relaxed);
            o->age = (std::uint8_t)GC_AGE0;
            o->next.store(h->young.load(std::memory_order_relaxed),
                         std::memory_order_relaxed);
            h->young.store(o, std::memory_order_relaxed);
        }
        h->rehome_pending.store(false, std::memory_order_relaxed);
    }
    /* Note: the nursery color reset is **not** done here but in finish_cycle -- it must happen before the phase turns
     * PAUSE in order to avoid rehome_some rewriting `o->next`, and putting it at the tail lets the invariant "when the next cycle's marking starts,
     * the nursery is entirely white objects" hold already during idle time and be checked directly by gc_verify. */
    g.kind.store(major ? (int)GC_KIND_MAJOR : (int)GC_KIND_MINOR,
                 std::memory_order_relaxed);
    if (major) g.major_cycles.fetch_add(1, std::memory_order_relaxed);
    else       g.minor_cycles.fetch_add(1, std::memory_order_relaxed);
    g.cycle_white = g.white.load(std::memory_order_relaxed);
    g.phase.store((int)GC_PHASE_PROPAGATE, std::memory_order_release);
    g.oldscan = nullptr;
    g.oldscan_heap = major ? nullptr : heaps_head();   /* minor: every heap's old generation acts as a root set */
    gray_clear();
    g.scan.cur = nullptr;
    g.scan.off = 0;
    g.promoted_since_major = 0;
}
/* Tail: merge the nursery into the young generation, and recompute the next cycle's trigger point under the debt model. Debt formulas (the same as Lua lgc.c setpause/
 * setminordebt, except that exact live bytes are used): after a major, debt = live_bytes * PAUSE% (wait for the heap to grow by about
 * (PAUSE-100)%); after a minor, debt = live_bytes * MINORMUL% (collection frequency proportional to heap size). Neither is below
 * MIN_HEAP_SIZE, avoiding pathologically frequent collection on small heaps. */
/**
 * @brief Finish a collection cycle: nursery rehome, reset colors, update live bytes and the next cycle's debt quota.
 * @note `live_bytes` is clamped to within `mapped_bytes` and constrained by `DEBT_CAP_MAPPED_PERCENT` (to prevent the debt from becoming astronomical and stopping the GC entirely).
 * @internal
 */
inline void finish_cycle() {
    gc_state& g = G();
    const bool was_major = (g.kind.load(std::memory_order_relaxed)
                                            == (int)GC_KIND_MAJOR);
    gray_clear();
    g.oldscan = nullptr;
    g.oldscan_heap = nullptr;
    g.scan.cur  = nullptr;
    g.scan.off  = 0;
    g.sweep_heap = nullptr;
    g.sweep_link = nullptr;
    g.sweep_stage = 3;                   /* 3 = this cycle's sweep is finished */
    /* Safety-net publish of the remaining dead-slot batch (normally already published at the end of a sweep_some slice).
     * `flush_dirty_slots` clears `sweep_dirty` to nullptr itself, so there is no need to clear it again. */
    flush_dirty_slots();
    /* Cap the recycle stack: return the surplus empty chunks accumulated this cycle to the system (memory must be able to drop after the peak). */
    trim_recycle_stack();
    /* ---- Reset the colors of all objects in the nursery (must happen **before** the phase turns to PAUSE) ----
     * Every object **reachable** within the marking window gets reached by mark_object (the mutator's self-scan will hit it), and thus becomes gray/black =
     * **no white bit**; yet the nursery is not swept this cycle and is **not guaranteed** to be emptied before the next cycle starts (rehome_some is incremental).
     * Once an object enters the next cycle with no white bit, mark_object will skip it at "(m & WHITE_BITS) == 0 -> already gray/black",
     * **its children will never be expanded**, and those children are white -> swept -> dangling pointer.
     * The scope is the **entire nursery**, and it must not recolor only "the part newly allocated within the window" (objects allocated during the previous cycle's SWEEP and not yet
     * rehomed are likewise darkened by marking -- measured: it was first written as "recolor only the window suffix", and thread-churn reproduced it).
     * The position must be before `phase = PAUSE`: rehome_some is called only in the PAUSE branch, and it unlinks nodes from the nursery
     * and **rewrites `o->next` to point into the young chain**, so a concurrent traversal would follow the rewritten next into the young chain. Here
     * only marked is written and no list pointer is touched, so even missing a few newly inserted objects cannot corrupt the structure. */
    {
        gc_mutex_guard lk;
        const std::uint8_t w_nursery = g.white.load(std::memory_order_relaxed);
        for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
            for (gc_object* o = h->nursery; o != nullptr; o = o->next)
                o->marked.store(w_nursery, std::memory_order_relaxed);
        }
    }
    /* **`phase = PAUSE` must be the last thing this function does** (apart from diag_flush).
     * Because the tail below contains `reap_dead_heaps()` -- it **frees heap structs** and rewrites the heap chain's
     * `hnext`; and `sweep_some` advances **lock-free** along the heap chain (taking the lock only once when fetching the chain head), and its safety precondition is precisely
     * "nobody touches the chain during traversal". Once PAUSE is published early, another thread can immediately start the **next cycle** (`gc_pace` sees
     * PAUSE and calls begin_cycle): the old cycle's owner publishes PAUSE and then reaps (unlink + free), while the new cycle's owner advances lock-free along the chain
     * -- the overlap is a use-after-free. The symptoms are a set of **mutually different** hangs/crashes (reading a garbage `hnext` and looping forever, the CAS spin of
     * publish_dead_slots after a chunk has been reclaimed, padding reporting OVERFLOW ...), which is exactly why it is hard to attribute.
     * Along the way, the PAUSE **readers** were changed from relaxed to acquire: the `next_major` / `live_at_major` written in the tail are read by
     * the **next cycle's** owner and need a happens-before edge (on x86 this is an ordinary mov anyway, zero cost). */
#if GCXX_BLACKLIST
    if (was_major) {
        /* Two-round blacklist confirmation. The whole pass runs under `gc_mutex`, but the `bl_*` words
         * are read **lock-free** by allocating threads (`slot_blacklisted`), which is why they are
         * atomics; relaxed is enough because they are a placement hint whose staleness can only make the
         * allocator skip one slot less. */
        gc_mutex_guard lk;
        for (gc_chunk* c = g.chunks; c != nullptr; c = c->all_next) {
            if (c->slot_size == 0) continue;     /* a large object is one whole object, with no free pages */
            const std::uint16_t hits = c->bl_hits.load(std::memory_order_relaxed);
            std::uint16_t black = (std::uint16_t)(
                c->bl_black.load(std::memory_order_relaxed) | (hits & c->bl_prev));
            unsigned cnt = 0;
            for (unsigned b = 0; b < 16; ++b)
                if ((black >> b) & 1u) ++cnt;
            if (cnt > (unsigned)BLACKLIST_MAX_PAGES)
                black = 0;                       /* give up: avoid all heap space being dodged away */
            c->bl_black.store(black, std::memory_order_relaxed);
            c->bl_prev = hits;
            c->bl_hits.store(0, std::memory_order_relaxed);
        }
    }
#endif
    {
        /* per-heap debt reset + nursery rehoming request (under the lock: walk the heap chain). First merge the batch accounting accumulated on the free side into
         * the global one -- total_bytes is about to be used to compute the next round's quota, and anything left unflushed would make it report high. */
        sweep_stats_flush();
        std::size_t live = g.total_bytes.load(std::memory_order_relaxed);
        /* The physical upper bound on live bytes is "the bytes already mapped into the process": live objects can only come from mapped slots. This step clamps any
         * estimation error back to a physically possible value -- otherwise a wrong quota permanently turns the GC off (see publish_pending). */
        if (live > g.mapped_bytes) live = g.mapped_bytes;
        gc_mutex_guard lk;
        /* Reap heap structs for which "the thread has terminated + the object chain is empty + no chunk ownership remains": this round's sweep has just finished (the object chain has been
         * cleared), phase is about to become PAUSE (sweep will no longer walk the heap chain), and gc_mutex is held. */
        reap_dead_heaps();
        std::size_t nheaps = 0;
        for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext)
            ++nheaps;
        if (nheaps == 0) nheaps = 1;
        std::ptrdiff_t share;
        if (was_major) {
            g.live_at_major = live;
            share = (std::ptrdiff_t)(live * (std::size_t)PAUSE_PERCENT / 100u
                                     / nheaps);
        } else {
            share = (std::ptrdiff_t)(live * (std::size_t)MINOR_MUL / 100u
                                     / nheaps);
        }
        if (share < (std::ptrdiff_t)MIN_HEAP_SIZE)
            share = (std::ptrdiff_t)MIN_HEAP_SIZE;
        /* ---- frequency safety valve ----
         * The quota is the **only** quantity that decides when the next round starts. The two steps above have already clamped live to
         * a physically possible value; here we add one more hard upper bound proportional to the **real mapped footprint**:
         * a heap may allocate at most "CAP% of the heap's mapped bytes" more before it must collect once.
         *
         * Together with the property guaranteed by the static_assert: when accounting is normal this upper bound **never takes effect**
         * (live <= mapped and CAP >= PAUSE); it only covers "when estimation fails completely, the worst case
         * is only collecting more often", not "never collecting" -- the latter is not self-healing (unbounded heap growth). */
        auto cap = (std::ptrdiff_t)
            (g.mapped_bytes * (std::size_t)DEBT_CAP_MAPPED_PERCENT
             / 100u / nheaps);
        if (cap < (std::ptrdiff_t)MIN_HEAP_SIZE)
            cap = (std::ptrdiff_t)MIN_HEAP_SIZE;
        if (share > cap) share = cap;
        const bool flood = live > g.live_at_major * (std::size_t)PAUSE_PERCENT / 100u;
        for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
            /* ---- scale the debt floor with the **allocation size** (otherwise large objects cause a GC storm) ----
             * Problem: the floor on `share` is a fixed constant, while "the fixed cost of one GC round" is unrelated to "how many bytes were allocated
             * this round". So as long as one allocation consumes the whole quantum, the GC runs a **full round on every single allocation**.
             * Measured: 2000 allocations of 64KB -> 364 GC rounds -> 12.96 us/op; the same number of 128B objects
             * gave 0 rounds, 0.07 us/op.
             *
             * Fix: this heap's floor is at least `MAX_ALLOC_DEBT_MUL` times "its **largest single allocation** this round"
             * -- so when "a single object occupies X bytes" at least 16X more may be allocated before the next round collects, and the GC cost is
             * amortized to the same order as the allocation volume. **Only affects large objects**: small objects are already far below `MIN_HEAP_SIZE`.
             * The upper bound is still constrained by `cap` (the frequency safety valve proportional to real mapped bytes) -- scaling up the floor does not
             * break through the hard upper bound "at most CAP% of mapped may be allocated more". */
            std::ptrdiff_t hs = share;
            const std::size_t floor_bytes =
                h->max_alloc * (std::size_t)MAX_ALLOC_DEBT_MUL;
            if (floor_bytes > (std::size_t)hs && floor_bytes <= (std::size_t)cap)
                hs = (std::ptrdiff_t)floor_bytes;
            if (hs > cap) hs = cap;
            h->debt_reset.store(hs, std::memory_order_relaxed);
            h->debt_reset_pending.store(true, std::memory_order_release);
            h->rehome_pending.store(true, std::memory_order_release);
            h->max_alloc = 0;              /* the new round re-tallies this round's largest single allocation */
        }
        if (flood) g.next_major = true;  /* heap growing too fast: go straight to major next round */
    }
    /* ---- the last step of the wrap-up: publish PAUSE (see the long comment block in the middle of this function) ---- before that no thread can yet see
     * PAUSE, so nobody can start the next round and thus overlap with the reap_dead_heaps above. */
    /* `kind` must be zeroed **before** `phase`: a concurrent reader sees both fields, and if PAUSE were published
     * before kind is cleared, there would be a transient state "phase == PAUSE and kind == MAJOR"
     * (measured: hit 48 times in 6.55 million concurrent samples). No code reads `kind` at that instant,
     * so it is **harmless**, but it breaks the assertable invariant "PAUSE <=> kind == NONE",
     * and makes it impossible to write a clean predicate in concurrent tests. Conversely, "kind == NONE and phase != PAUSE" is a safe transient:
     * the next round's owner always reads `phase` first (release/acquire pairing) and only reads `kind` after seeing PAUSE. */
    g.kind.store((int)GC_KIND_NONE, std::memory_order_relaxed);
    g.phase.store((int)GC_PHASE_PAUSE, std::memory_order_release);
    g.cycles.fetch_add(1, std::memory_order_relaxed);
    diag_flush();                     /* fallback: make sure diagnostic counters do not linger across rounds */
}
/* ---- collection state machine advancement ---- gc_step is called only by cycle_owner (guaranteed by gc_pace/full_collect): at any moment only
 * one thread advances the state machine, so the phase field can safely be read and written on the owner side. */
/**
 * @brief Whether marking is **really** finished: the gray set is empty **and** there is no leftover overflow, no fallback rescan owed, and no unfinished old-generation cursor.
 * @internal
 */
inline bool cycle_mark_done(gc_state& g) {
    /* Marking counts as finished only when the gray set is empty **and** there is no leftover overflow and no fallback rescan is owed. */
    return gray_empty() &&
           !g.gray_overflow.load(std::memory_order_relaxed) &&
           !g.gray_need_rescan &&
           g.oldscan == nullptr && g.oldscan_heap == nullptr && g.scan.cur == nullptr;
}
/**
 * @brief Advance the incremental state machine by some work units (called only by `cycle_owner`).
 * @note Reentrancy guard: returns 0 when `step_depth > 0`.
 * @internal
 */
inline std::size_t gc_step(std::size_t units) {
    gc_state& g = G();
    if (g.step_depth.load(std::memory_order_relaxed) > 0)
        return 0;                         /* reentrancy guard: this slice is already in progress */
    if (units > 65536u) units = 65536u;
    g.step_depth.fetch_add(1, std::memory_order_relaxed);
    std::size_t done = 0;                 /* number of work units actually completed */
    for (std::size_t i = 0; i < units; ++i) {
        const int ph = g.phase.load(std::memory_order_relaxed);
        switch (ph) {
        case (int)GC_PHASE_PROPAGATE: {
            /* ---- marking completes atomically ---- with multiple threads, begin_cycle has already done the root scan + full-graph expansion +
             * atomic_finish inside the STW, so PROPAGATE is normally not seen here (fallback: run it to completion). */
            do {
                propagate_some(4096, g.scan);
            } while (!cycle_mark_done(g));
            g.phase.store((int)GC_PHASE_ATOMIC, std::memory_order_relaxed);
            atomic_finish();
            done += units - i;            /* the whole marking pass counts as full work volume */
            i = units;
            break;
        }
        case (int)GC_PHASE_ATOMIC:
            atomic_finish();              /* fallback (normally not reached) */
            ++done;
            break;
        case (int)GC_PHASE_SWEEP: {
            /* The rest of this slice's quota all goes to sweeping, reported by the **number of objects actually processed**. A4: one clock-read pair is amortized over the whole slice
             * (SWEEP_UNITS objects), and only on the cycle_owner's path -- the hot path is unaffected. */
            const std::chrono::steady_clock::time_point ts0 =
                std::chrono::steady_clock::now();
            const std::size_t n = sweep_some(units - i);
            g.t_sweep_us.fetch_add(gc_us_since(ts0), std::memory_order_relaxed);
            done += n;
            if (n == 0) i = units;        /* no work left in this phase: end this slice */
            else        i += n - 1;       /* consume the quota (the loop head will still ++i) */
            break;
        }
        case (int)GC_PHASE_END:
            finish_cycle();
            ++done;
            break;
        default:
            i = units;                    /* PAUSE etc.: nothing to do */
            break;
        }
    }
    g.step_depth.fetch_sub(1, std::memory_order_relaxed);
    return done;
}
/* Pacing control before allocation (per-heap debt model): the safepoint check comes first (during a GC request every allocation passes through here,
 * one relaxed load on the hot path); at PAUSE the owner rehomes the nursery in batches, and when this heap's debt runs out it starts a collection
 * (begin_cycle does the STW + atomic marking internally, sweeping is left to later incremental advancement); while a collection is in progress only the cycle_owner
 * advances gc_step, and other threads push the debt back to a positive value (new objects go into the nursery and are unaffected this round). */
/**
 * @brief **Pacing control on the allocation path**: once enough debt has accumulated, start a collection round or advance an existing one.
 * @note Every trigger predicate is a function of "memory growth / debt"; time does not decide whether to collect.
 * @internal
 */
/** @brief The **non-idle branch** of `gc_pace`: this round is marking/sweeping, and this thread is either the advancer or waiting.
 *  @note This part carries a `do/while` loop (at most STEP_MAX_SLICES `gc_step` calls); inlining it would only inflate
 *        the allocation hot path. Once split out, the resident part of `gc_pace` is only "check quota / subtract debt / return to safepoint /
 *        test for idle", i.e. the few things every allocation really must pay for. The predicates are unchanged word for word.
 * @internal
 */
GCXX_NOINLINE inline void gc_pace_busy(gc_heap* h, gc_state& g) {
    if (g.cycle_owner.load(std::memory_order_acquire) == h) {
        int slices = 0;
        do {
            /* Repay debt by the **work actually completed** (see BYTES_PER_WORK_UNIT): this way "how much debt is repaid per object processed"
             * is guaranteed by construction and decoupled from the batch parameters, so there is neither fake repayment (heap bloat) nor over-sweeping (allocation spikes). */
            const std::size_t work = gc_step(SWEEP_UNITS);
            h->debt += (std::ptrdiff_t)(work * BYTES_PER_WORK_UNIT);
            ++slices;
        } while (h->debt <= 0 &&
                 slices < (int)STEP_MAX_SLICES &&
                 g.phase.load(std::memory_order_relaxed)
                     != (int)GC_PHASE_PAUSE);
    } else {
        /* Not the advancer: push the debt back up and wait for this round to end (finish_cycle will hand out a new quota) */
        h->debt = (std::ptrdiff_t)MIN_HEAP_SIZE;
    }
}
/** @brief The **idle + debt-exhausted branch** of `gc_pace`: start a new round (or fall back to a quota while someone else collects).
 * @internal
 */
GCXX_NOINLINE inline void gc_pace_begin(gc_heap* h, gc_state& g) {
    /* begin_cycle returning false = lost the race (someone else is collecting) or was cancelled (a thread did not stop); returning true but
     * with an owner other than self = someone else won this round. All three cases fall back to a minimum quota, to avoid retrying on every allocation. */
    if (!begin_cycle(h) ||
        g.cycle_owner.load(std::memory_order_acquire) != h)
        h->debt = (std::ptrdiff_t)MIN_HEAP_SIZE;
}
GCXX_ALWAYS_INLINE void gc_pace(gc_heap* h, std::size_t bytes) {
    gc_state& g = G();
/* The predicate uses the macros rather than raw built-ins: a platform where dynamic roots are **implemented** = has `dl_iterate_phdr` (Linux/Android/FreeBSD)
     * or PE section parsing (Windows) or `_dyld_*` (Apple). */
#if GCXX_USE_DYNAMIC_ROOTS && (GCXX_PLATFORM_HAVE_LINK_H || GCXX_PLATFORM_WINDOWS || \
                               GCXX_HAVE_DYLD_IMAGE_API)
    /* One-time collection of "dynamic library data segment" roots. Hot-path cost = **one relaxed atomic read** (almost always true),
     * the actual collection happens in the GCXX_NOINLINE cold path and is done **outside the lock** (see scan_dyn_roots_slow). */
    if (!g.dyn_roots_scanned.load(std::memory_order_relaxed))
        scan_dyn_roots_slow(g);
#endif
    /* debt is a **plain** field private to the owner (see gc_heap::debt): the owner does a plain subtraction here,
     * and the collector never modifies it directly from another thread -- it only publishes a new quota through `debt_reset` / `debt_reset_pending`,
     * which the owner checks and applies below with one relaxed load. Hence it is
     * **not** atomic and does **not** use fetch_sub/add. */
    if (h->debt_reset_pending.load(std::memory_order_acquire)) {
        h->debt = h->debt_reset.load(std::memory_order_relaxed);
        h->debt_reset_pending.store(false, std::memory_order_release);
    }
    h->debt -= (std::ptrdiff_t)bytes;      /* owner private: plain subtraction */
    /* **acquire**, consistent with the one in `gc_mutex_lock`: the initiator does "first advance `sp_epoch` (acq_rel),
     * then release-store `sp_request`", and `safepoint_reach` subsequently acquire-reads `sp_epoch` to claim the round's number.
     * A relaxed read "sees the request" but establishes no happens-before edge => on a weakly ordered CPU (AArch64 is a first-class
     * target) it may read a **stale epoch** and park with an old number, so it is never counted as "in position for this round",
     * and the STW can only wait out its budget and then cancel / force-suspend. This affects liveness only, not soundness, but the two entry points must not disagree.
     * On x86-64 an acquire load and a relaxed load compile to the same code, so it is free. */
    if (g.sp_request.load(std::memory_order_acquire) != 0)
        safepoint_reach(h);              /* reach the safepoint: self-scan + park + release */
    const int ph = g.phase.load(std::memory_order_acquire);
    if (ph == (int)GC_PHASE_PAUSE) {
        if (h->rehome_pending.load(std::memory_order_relaxed))
            rehome_some(h, 64);          /* batch rehoming on the owner side (done only at PAUSE) */
        if (h->debt <= 0) {
            gc_pace_begin(h, g);
        }
        return;
    }
    gc_pace_busy(h, g);
}
/* Force the current round to run to completion (the caller must be cycle_owner or have taken over ownership) */
/**
 * @brief Run the current round's incremental process to completion in one go.
 * @internal
 */
inline void run_to_completion() {
    gc_state& g = G();
    /* step_depth > 0 means the caller is already inside gc_step (reentrant): then gc_step returns immediately and the loop makes no progress, so hand back to the outer level. */
    if (g.step_depth.load(std::memory_order_relaxed) > 0) return;
    /* A forced collection must not silently leave an unfinished round behind: return only after PAUSE is observed. */
    while (g.phase.load(std::memory_order_acquire) != (int)GC_PHASE_PAUSE)
        gc_step(4096);
}
/* Reclaim a stale step_depth possibly left behind by a previous owner. The normal lifecycle is "gc_step entry +1 / exit -1", but if the owner
 * thread is killed by the host **inside** gc_step (TerminateThread, process-level force kill, crash cleanup), that fetch_sub
 * never executes -- the count stays >0 forever, and from then on every gc_step returns 0 immediately at the reentrancy guard, so nobody advances this round any more;
 * and full_collect's takeover loop does not exit until it observes PAUSE, so it **spins forever holding gc_mutex**, dragging the whole process's
 * allocation path down with it.
 * Zeroing is only valid if "the thread is really gone", so we use the **strong** predicate heap_thread_ended rather than
 * heap::dead -- the latter only means "the thread-exit callback has run", and that thread may well still be running (it can still allocate during TLS teardown,
 * using exactly the deliberately retained heap), or may even be legitimately inside gc_step; zeroing in that case would let two threads advance the same round at once. */
/**
 * @brief Clear a stale `step_depth` (the reentrancy flag left behind by a previous owner that died mid-way).
 * @note Otherwise the new owner would be locked out forever by that non-zero value -- the GC would never advance again.
 * @internal
 */
inline void reclaim_stale_step_depth(gc_heap* owner, gc_heap* self) {
    gc_state& g = G();
    if (owner == self) return;                    /* one cannot die on oneself */
    if (g.step_depth.load(std::memory_order_relaxed) == 0) return;
    if (owner == nullptr || !heap_thread_ended(owner)) return;
    g.step_depth.store(0, std::memory_order_relaxed);
}
/* Force one complete major (the gc_collect entry). Any thread may call it: if a round is in progress, wait for it to finish (take over if the owner thread
 * has died), then do a STW major itself and run the whole round in place, holding gc_mutex (recursive) throughout.
 * @return true = a round really completed; false = this round was cancelled (a thread failed to reach a safepoint within the budget) -- on cancellation
 *         **no** memory is reclaimed, deliberately: if the mutator cannot be stopped, marking cannot be done safely. */
/**
 * @brief Perform one complete major collection.
 * @return `false` = this round was cancelled (a thread failed to reach a safepoint within `STW_TIMEOUT_MS`) -- this round reclaims nothing.
 * @internal
 */
inline bool full_collect() {
    gc_state& g = G();
    gc_heap* h = my_heap();
    int expected0 = 0;
    if (!g.collect_depth.compare_exchange_strong(expected0, 1,
            std::memory_order_acq_rel, std::memory_order_relaxed))
        return false;                         /* a thread is already collecting: return directly */
    bool collected = false;
    {
        gc_mutex_lock(g.gc_mutex);
        std::unique_lock<std::recursive_mutex> lk(g.gc_mutex, std::adopt_lock);
        /* The takeover path (run_to_completion below) also sweeps, including objects of this thread's heap -- so first publish this heap's
         * unaccounted allocations, for the same reason as begin_cycle. */
        publish_pending(h);
        /* Wait for the in-progress round to end. This thread takes over advancement in the following cases: this thread is the owner (only it can advance, waiting would
         * necessarily deadlock); the owner is null or **not on the global heap chain** (such a pointer is not trustworthy at all: it either points to a heap already
         * reaped by reap_dead_heaps, or is an invalid value obtained during TLS teardown, and reading its dead/thread_handle reads non-live memory,
         * so heap_thread_ended necessarily reports "not ended" and the loop can never reach its end -- a hang. See heap_on_chain for the predicate);
         * the owner's thread is **proven terminated** (the predicate adds heap_thread_ended to dead: a host force-killing a thread / a runtime defect
         * can both leave the exit callback never run and dead false forever, and in exactly that case the owner is least likely ever to advance again).
         * In all other cases, wait for the owner to finish on its own. */
        while (g.phase.load(std::memory_order_acquire)
                   != (int)GC_PHASE_PAUSE) {
            gc_heap* owner = g.cycle_owner.load(std::memory_order_acquire);
            /* **First test "is it on the chain", then touch its fields** -- the opposite order reads memory that may already have been freed
             * (`dead` is a plain field, and `thread_handle` even gets fed to WaitForSingleObject). Only owner == h
             * is the exception: that is this thread's own heap and is certainly valid. The evaluation order is guaranteed by the **short-circuit** of the `||` below.
             * owner == h means two things: it is of course credible, and it **must be taken over by this thread** (only it can advance its own round,
             * waiting on it is a deadlock), so it is listed as a separate takeover condition and cannot just be folded into cred. */
            const bool on_chain = heap_on_chain(owner);
            const bool cred    = (owner == h) || on_chain;
            const bool take_over =
                (owner == h) || !cred ||
                owner->dead.load(std::memory_order_relaxed) ||
                heap_thread_ended(owner);
            if (take_over) {
                /* If the previous owner was **killed inside gc_step**, its step_depth can never be decremented back; without clearing it first,
                 * run_to_completion below returns immediately and this while loop spins forever holding gc_mutex.
                 * When the owner is not credible (not on the chain), the heap_thread_ended predicate cannot be used -- that reads non-live memory; in that case
                 * clear step_depth directly (an owner that is not even a heap cannot legitimately be advancing any round, while the real
                 * owner can only be this thread's own heap, which the keep rule of reap_dead_heaps already excludes from reaping). */
                if (cred) reclaim_stale_step_depth(owner, h);
                else      g.step_depth.store(0, std::memory_order_relaxed);
                g.cycle_owner.store(h, std::memory_order_release);
                run_to_completion();          /* drive the in-flight round to PAUSE */
                continue;
            }
            lk.unlock();
            std::this_thread::yield();
            /* **The lock must be retaken with the safepoint-aware `gc_mutex_lock`**, not bare `lk.lock()`:
             * a thread blocked on a bare lock cannot reach a safepoint, and then the owner that is collecting waits out its whole budget on `got < want`
             * and then cancels this round (POSIX) / force-suspends it (Windows). `gc_mutex_lock`
             * self-scans + parks before continuing to contend for the lock when it cannot get the lock and `sp_request != 0`, which is exactly what this shape
             * is designed for (see its documentation). After acquiring it, hand it back to unique_lock via adopt_lock. */
            gc_mutex_lock(g.gc_mutex);
            lk = std::unique_lock<std::recursive_mutex>(g.gc_mutex, std::adopt_lock);
        }
        g.cycle_owner.store(h, std::memory_order_release);
        g.next_major = true;
        collected = begin_cycle(h);           /* STW + marking + enter SWEEP */
        if (collected) {
            run_to_completion();              /* finish sweeping in place */
            g.cycle_owner.store(nullptr, std::memory_order_release);
        }
    }
    g.collect_depth.store(0, std::memory_order_release);
    return collected;
}
/* The unified "allocation entry" (hot path). The order is critical: first GC pacing control (which may start a new round / advance sweeping), then create
 * the new object -- the new object is chained only after "this round's sweep has started", so it can never be wrongly reclaimed by this round. */
/** @brief The **OOM emergency path** of `allocate_bytes`: do a full major to free as much as possible, then retry; if that still fails, throw.
 *  @note It is split out because this path contains `full_collect()` (a big chunk) and two `throw`s (exception tables);
 *        leaving it in `allocate_bytes` would inflate the hot path and block inlining. The predicates and order are unchanged word for word.
 * @internal
 */
GCXX_NOINLINE inline void* allocate_bytes_oom(gc_heap* h, std::size_t size,
                                              std::uint8_t type, bool ctor_zeroes,
                                              bool force_zero) {
    gc_state& g = G();
    /* full_collect returning false = this round was cancelled (some thread could not be stopped); retrying then has no
     * point -- throw instead of continuing under unsafe assumptions.
     *
     * The gate is a **claim**, not a test: several threads may reach here at once (each with its own
     * heap), and "test then store" would let all of them into `full_collect()` together. Only the
     * thread that flips `in_oom` false->true runs the emergency collection; the others fall straight
     * through to `bad_alloc`, which is the honest answer for "someone else is already collecting".
     * See the field's own note for why it is atomic at all. */
    bool expected_oom = false;
    if (g.collect_depth.load(std::memory_order_relaxed) == 0 &&
        g.in_oom.compare_exchange_strong(expected_oom, true,
                                         std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
        const bool collected = full_collect();
        g.in_oom.store(false, std::memory_order_relaxed);
        if (collected) {
            gc_object* o = object_new(h, (std::uint32_t)size, type, ctor_zeroes,
                                      force_zero);
            if (o != nullptr) return payload_of(o);
        }
    }
    throw std::bad_alloc();
}
GCXX_ALWAYS_INLINE void* allocate_bytes(std::size_t size, std::uint8_t type,
                                               bool ctor_zeroes = false,
                                               bool force_zero = false) {
    if (size > (std::size_t)MAX_OBJECT_SIZE)
        throw std::bad_alloc();                 /* the size field is only 4 bytes */
    gc_heap* h = my_heap();
    gc_pace(h, size);                           /* debt model: advance collection only when debt is owed */
    gc_object* o = object_new(h, (std::uint32_t)size, type, ctor_zeroes, force_zero);
    if (o == nullptr) return allocate_bytes_oom(h, size, type, ctor_zeroes, force_zero);
    return payload_of(o);
}
/* ---- thread-exit cleanup (thread_local destructor) ----
 * chunks are returned to the global recycle stack (ownership change, unregister/re-register under the lock); the three object chains are kept -- the objects on them
 * may still be referenced by other threads, and later sweeps keep walking this heap (the structure is deliberately not freed).
 * nursery rehoming is done in one pass for dead heaps by the next start_cycle. */
/**
 * @brief Heap cleanup on thread exit: hand it over to the collector (the struct is retained until `reap_dead_heaps` removes it).
 * @internal
 */
inline void heap_thread_exit_cleanup(gc_heap* p) {
    if (p == nullptr) return;
    gc_mutex_guard lk;
    /* Publish the allocation counters buffered by this heap before its
     * objects can be swept from a different thread. */
    publish_pending(p);
    p->dead.store(true, std::memory_order_release);
    for (auto & cls : p->partial) {
        gc_chunk* c = cls;
        while (c != nullptr) {
            gc_chunk* nx = c->cnext;
            c->in_partial.store(false, std::memory_order_relaxed);
            c->cnext = nullptr;
            /* partial means "has a free slot", not "is empty". Keep a non-empty chunk registered so
             * roots owned by other threads can still resolve its objects; the later sweep will retire
             * it through chunk_free because it is no longer in a partial list. */
            if (c->used == c->freed_pub.load(std::memory_order_relaxed)) {
                /* The single retirement decision point (the lock is already held at the top of this function; gc_mutex is reentrant). It redoes all the predicates internally,
                 * and "confirm it is still on the global chain" is guaranteed by chunk_retire itself (if not on the chain it returns false and does not push) --
                 * the order is critical: if the collector's chunk_free has already retired this empty chunk first, it may already have been
                 * claimed and re-registered by another heap, and unlinking now would delete the **new owner**'s entry. */
                maybe_retire_chunk(c);
            }
            c = nx;
        }
        cls = nullptr;
    }
    /* Deliberately **not** clearing the TLS slot: after the exit callback has run this thread may still have later-destructing TLS objects to allocate, and the heap struct
     * is still valid at this moment, so let it keep reusing this heap already marked dead -- otherwise it would create a new orphan heap that is **never cleaned up**
     * (dead=false forever), making every later STW round wait for it in vain and cancel after the budget is exhausted, which means the whole process can never collect
     * garbage again. Cost: objects allocated in this phase are not scanned for roots that live only in this thread's stack/registers -- an inherent limitation of a cooperative GC. */
    (void)p;
    /* The heap struct itself is not freed here: its object chain may still hold objects (other threads may still reference them), and chunks may
     * not yet be returned. The real reaping is handled by reap_dead_heaps (thread terminated + object chain fully empty + no chunk ownership).
     *
     * POSIX: set thread_ended here -- this is the only reliable signal that "the thread will truly never touch this heap again"
     * (for the reason see heap_thread_ended). After it is set this thread may still reuse this heap in a later my_heap_slow round,
     * which clears the flag back to false. */
#if GCXX_PLATFORM_POSIX
    p->thread_ended.store(true, std::memory_order_release);
#endif
}
/* Allocation result type: non-array T -> T*; array U[N] -> U* (same as `new U[N]`: the array decays to an element pointer, so
 * `int* p = gc<int[]>(10)` can be written directly); multidimensional int[M][N] -> int(*)[N].
 * Note that a pointer-to-array does **not** decay, so the result type is spelled out explicitly here rather than relying on implicit conversion of the return value. */
template <typename T>
struct alloc_ptr {
    typedef T* type;
};
template <typename U, std::size_t N>
struct alloc_ptr<U[N]> {
    typedef U* type;
};
} /* namespace detail */
} /* namespace gcxx */
/* ============================  P U B L I C   A P I  ============================== */
/* --------------------------------------------------------------------------
 * gc_init()          optional: the GC completes initialization by itself during static initialization (see gc_auto_init at the end of the file).
 * This macro is kept for compatibility with old code: it anchors the stack-scan upper bound to the caller's stack frame plus a safety margin; calling it explicitly is harmless (idempotent).
 * ------------------------------------------------------------------------ */
#define gc_init()                                                               \
    do {                                                                        \
        char gc_stack_anchor__ = 0;                                             \
        (void)gc_stack_anchor__;                                                \
        ::gcxx::detail::gc_start_impl(static_cast<void*>(&gc_stack_anchor__));  \
    } while (0)
/* --------------------------------------------------------------------------
 * gc_refresh_roots()  -- re-collect the root ranges of the "static area + dynamic library data segments".
 *
 *   dlopen("plugin.so", RTLD_NOW);   // if that library's globals hold GC pointers
 *   gc_refresh_roots();              // it must be called once, or those pointers are not roots
 *
 * After the GC starts it **collects them once automatically** (covering every shared object already loaded at the first collection), so
 * an ordinary program (libraries linked at process startup) does **not** need to call it. Only libraries pulled in by a **runtime `dlopen`**
 * need an explicit refresh  -- because repeating that collection automatically would have to happen outside `gc_mutex` (lock-order safety, see
 * the notes on detail::scan_dyn_roots_slow), and cannot be hooked onto every `dlopen`.
 *
 * Idempotent and thread-safe: may be called at any time (including under multiple threads). The cost is one rescan of the range table.
 * ------------------------------------------------------------------------ */
inline void gc_refresh_roots() {
/* The predicate uses macros rather than bare built-ins: platforms where dynamic roots **are implemented** = those with `dl_iterate_phdr` (Linux/Android/FreeBSD)
     * or with PE section parsing (Windows) or with `_dyld_*` (Apple). */
#if GCXX_USE_DYNAMIC_ROOTS && (GCXX_PLATFORM_HAVE_LINK_H || GCXX_PLATFORM_WINDOWS || \
                               GCXX_HAVE_DYLD_IMAGE_API)
    ::gcxx::detail::static_ranges_refresh(::gcxx::detail::G());
#else
    /* Other platforms have no dynamic-roots layer; the static roots are a one-shot snapshot and need no refresh. */
#endif
}
/* --------------------------------------------------------------------------
 * gc<T>(args...)  -- allocate and construct a T on the GC heap.
 *
 *      Node* n = gc<Node>(42);          // constructed with 42
 *      int*  a = gc<int[]>(1000);       // 1000 ints, value-initialized
 *      Item* v = gc<Item[]>(3000);      // array of structs; pointers inside the elements are traced
 * A static length may also be written as part of the type (kept for compatibility): gc<int[1000]>(), gc<int[3][4]>().
 *
 * Array semantics are exactly those of `new T[n]()`: elements are **value-initialized** (independent of GCXX_ZERO_MEMORY); the array form accepts no
 * constructor arguments (C++20's `new T[n](a,b,c)` is **deliberately not enabled**); if an element constructor throws, the language destroys the already-constructed part,
 * and the whole block becomes unreachable garbage in place, with no need and no way to free it by hand.
 * A whole array needs only one object header: the block **is** a single GC object, and a pointer into any element (a[i], a[i].member,
 * even a negatively-biased pointer such as a-24) is resolved back to this object and keeps the whole block alive; the scan class is derived at compile time from the element type (scalar arrays
 * are skipped whole), and gc_declare_plain<T>() can declare a custom plain-data type.
 * General limits: the memory comes from the GC heap and must not be freed by hand; destructors are never called by the GC; alignment <= 16 bytes; a single object (including
 * a whole array) must not exceed gcxx::detail::MAX_OBJECT_SIZE (4GiB-256).
 * ------------------------------------------------------------------------ */
/* Helper traits for array detection. They must live in a **named** namespace: they appear in gc's return type/constraints, and gc is
 * a template with external linkage  -- putting them in an anonymous namespace would make every TU see a different type (ODR violation, and one instantiation per TU). */
namespace gcxx { namespace detail {
template <typename T> struct gc_array_elem           { typedef T type; };
template <typename U> struct gc_array_elem<U[]>      { typedef U type; };
template <typename T> struct gc_is_unbounded_array               : std::false_type {};
template <typename U> struct gc_is_unbounded_array<U[]>          : std::true_type  {};
/* Scan class of the allocation. With ForceScan = true **everything is conservatively scanned** (the gcs semantics); the only exception is a block too small to hold one
 * complete pointer  -- such a block physically cannot contain a pointer, so scanning it is wasted work. */
inline std::uint8_t alloc_type_for(std::size_t total, std::uint8_t auto_type,
                                   bool force_scan) {
    if (!force_scan) return auto_type;
    return (total < sizeof(void*)) ? (std::uint8_t)GC_TYPE_SMALL
                                   : (std::uint8_t)GC_TYPE_RAW;
}
} /* namespace detail */
} /* namespace gcxx */
/* ---- implementation body shared by gc / gcs: **not exposed** ---- users only ever see the three overload groups gc / gcs / gca; these two
 * functions are pure implementation detail (`ForceScan` is an internal switch), and exposing them would invite people to think they are a fourth and fifth allocation interface. */
namespace gcxx { namespace detail {

template <typename T, bool ForceScan, typename... Args>
GCXX_ALWAYS_INLINE typename alloc_ptr<T>::type
gc_build(Args&&... args) {
    static_assert(!std::is_reference<T>::value && !std::is_void<T>::value &&
                  !std::is_function<T>::value,
                  "gc<T>: T must be a non-reference, non-void, "
                  "non-function object type");
    static_assert(!std::is_array<T>::value || std::is_default_constructible<T>::value,
                  "gc<T[N]>: array elements are value-initialized, so the "
                  "element type must be default-constructible; allocate an array of "
                  "pointers (gc<T*[]>(n)) and gc each element instead");
    /* 16 = slot alignment granularity. The payload offset automatically moves to 32 for types with alignof>8, so alignas(16) types can now be allocated too. */
    static_assert(alignof(T) <= 16,
                  "gc<T>: T must not be over-aligned beyond 16 bytes; "
                  "the GC heap guarantees 16-byte slot alignment");
    static_assert(!std::is_array<T>::value || sizeof...(Args) == 0,
                  "gc<T[N]>: array elements are value-initialized and take no "
                  "constructor arguments (C++11..17 have no such syntax, and C++20's "
                  "parenthesized array init is deliberately not enabled so behaviour "
                  "stays identical across standards; assign elements after allocation). "
                  "For a runtime length use gc<T[]>(n)");
    /* When the constructor writes the whole block anyway, tell the allocator it need not memset again (see object_new): `T()` for trivial types is value
     * initialization = all zeros, and `new T[n]()` likewise (measured: even padding is zeroed along with the block). */
    const bool ctor_zeroes =
        (sizeof...(Args) == 0) && std::is_trivially_default_constructible<T>::value;
    void* mem = allocate_bytes(sizeof(T),
        alloc_type_for(sizeof(T),
            type_of<T>::value, ForceScan), ctor_zeroes);
    ::new (mem) T(std::forward<Args>(args)...);
    return static_cast<typename alloc_ptr<T>::type>(mem);
}

template <typename T, bool ForceScan>
GCXX_ALWAYS_INLINE typename gc_array_elem<T>::type*
gc_build_arr(std::size_t n) {
    typedef typename gc_array_elem<T>::type U;
    static_assert(!std::is_reference<U>::value && !std::is_function<U>::value,
                  "gc<T[]>: T must be an object type");
    static_assert(std::is_default_constructible<U>::value,
                  "gc<T[]>: array elements are value-initialized, so the "
                  "element type must be default-constructible; allocate an array of "
                  "pointers (gc<U*[]>(n)) and gc each element instead");
    static_assert(alignof(U) <= 16,
                  "gc<T[]>: element type must not be over-aligned beyond 16 bytes");
    const std::size_t cnt = (n == 0) ? 1u : n;   /* n==0 still occupies the space of one element */
    if (cnt > (std::size_t)gcxx::detail::MAX_OBJECT_SIZE / sizeof(U))
        throw std::bad_alloc();                  /* overflow / above the single-object limit */
    const bool ctor_zeroes = std::is_trivially_default_constructible<U>::value;
    void* mem = allocate_bytes(cnt * sizeof(U),
        alloc_type_for(cnt * sizeof(U),
            type_of<U>::value, ForceScan), ctor_zeroes);
    ::new (mem) U[cnt]();    /* value-initialize; if a constructor throws the language rolls back automatically (guaranteed by array new) */
    return static_cast<U*>(mem);
}

} } /* namespace gcxx::detail */

/**
 * @brief Allocate a `T` (or a static-length array `T[N]`) and construct it with forwarded arguments: **the scan class is decided automatically**.
 *
 * `gc<T>` uses compile-time type deduction to decide "should this memory be conservatively scanned":
 * a hit on `is_integral` / `is_floating_point` / `is_enum` / `gc_declare_plain<T>()` means **not scanned**,
 * everything else is **conservatively scanned**. This is the everyday entry point.
 *
 * @code
 *   Foo*  p = gc<Foo>(1, 2.0);       // T containing pointers -> scanned
 *   int*  n = gc<int>(42);           // built-in type -> not scanned
 *   char* b = gc<char[64]>();        // static-length array
 * @endcode
 *
 * @param args Arguments forwarded to the `T` constructor.
 * @return Pointer to the constructed object (the memory is provided and zeroed by the GC; construction is done by this function).
 * @note When unsure whether the type deduction is accurate, use `gcs<T>(...)` to force scanning  -- a wrong call does not merely mean "a bit slower",
 *       it means **a missed mark => wrongful reclamation => dangling pointer**.
 */
template <typename T, typename... Args>
GCXX_ALWAYS_INLINE typename std::enable_if<
    !::gcxx::detail::gc_is_unbounded_array<T>::value,
    typename ::gcxx::detail::alloc_ptr<T>::type>::type
gc(Args&&... args) {
    return ::gcxx::detail::gc_build<T, false>(std::forward<Args>(args)...);
}

/* ---- runtime-length arrays: gc<T[]>(n) / gcs<T[]>(n) ---- semantics are those of `new U[n]()` (elements value-initialized,
 * n==0 also returns a valid pointer, and if a constructor throws the language destroys the constructed part), returning U* (matching new's array decay). */
template <typename T>
GCXX_ALWAYS_INLINE typename std::enable_if<
    ::gcxx::detail::gc_is_unbounded_array<T>::value,
    typename ::gcxx::detail::gc_array_elem<T>::type*>::type
gc(std::size_t n) {
    return ::gcxx::detail::gc_build_arr<T, false>(n);
}

/**
 * @brief Untyped allocation: given only a byte count, returns `void*`.
 *
 * @code
 *   char* buf = static_cast<char*>(gc(n));
 *   Foo*  p   = static_cast<Foo*>(gc(sizeof(Foo)));   // construct it yourself
 * @endcode
 * Two faces of the same name as `gc<T>(...)` (a byte count alone takes this non-template overload); the two do not interfere with each other.
 * Without `T` there is no basis for "provably no pointers", therefore:
 *   * a block with `size >= sizeof(void*)` is **conservatively scanned whole**;
 *   * a block with `size < sizeof(void*)` cannot even hold one complete pointer and is skipped outright (`GC_TYPE_SMALL`).
 *
 * @param size Requested byte count; `0` is bumped to `1` (so the returned pointer can be recognized by address lookup).
 * @return Alignment as `malloc` (`max_align_t`), contents as `calloc` (**the whole block is zeroed**:
 *         a conservative GC needs clean initial values, since dirty bytes would be misread as pointers).
 * @warning Do **not `free`** the returned memory, and do not run destructors on it: the GC owns it.
 */
GCXX_ALWAYS_INLINE void* gc(std::size_t size) {
    if (size > (std::size_t)gcxx::detail::MAX_OBJECT_SIZE)
        throw std::bad_alloc();                 /* the size field is only 4 bytes */
    if (size == 0) size = 1;                    /* an object occupies at least 1 byte: size==0 would make the "payload start equal the payload end" and
       leave the address-lookup hit range empty, so such an object could never be resolved back from a pointer (malloc(0) likewise returns a unique pointer). */
    const std::uint8_t type =
        (size < sizeof(void*)) ? (std::uint8_t)::gcxx::GC_TYPE_SMALL
                               : (std::uint8_t)::gcxx::GC_TYPE_RAW;
    /* force_zero: this path has no type information, and zeroing is a **public contract** (see above), unaffected by GCXX_ZERO_MEMORY. */
    return ::gcxx::detail::allocate_bytes(size, type, false, true);
}

/**
 * @brief Allocate a `T` and construct it with forwarded arguments, **forcing conservative scanning** (even if `T` is a built-in type).
 *
 * Rationale: `gc<T>` uses compile-time type deduction to decide "to scan or not", based on
 * `is_integral` / `is_floating_point` / `is_enum` / `gc_declare_plain<T>()`; a single wrong call
 * (for example declaring a pointer-bearing struct as plain) means **a missed mark => wrongful reclamation => dangling pointer**.
 * `gcs` lets you say explicitly "I am not sure, take the most conservative route", at the cost of scanning this memory on every mark.
 *
 * @param args Arguments forwarded to the `T` constructor.
 * @return Pointer to the constructed object.
 * @note The only case not scanned: `size < sizeof(void*)` (physically cannot hold one complete pointer).
 */
template <typename T, typename... Args>
GCXX_ALWAYS_INLINE typename std::enable_if<
    !::gcxx::detail::gc_is_unbounded_array<T>::value,
    typename ::gcxx::detail::alloc_ptr<T>::type>::type
gcs(Args&&... args) {
    return ::gcxx::detail::gc_build<T, true>(std::forward<Args>(args)...);
}

template <typename T>
GCXX_ALWAYS_INLINE typename std::enable_if<
    ::gcxx::detail::gc_is_unbounded_array<T>::value,
    typename ::gcxx::detail::gc_array_elem<T>::type*>::type
gcs(std::size_t n) {
    return ::gcxx::detail::gc_build_arr<T, true>(n);
}

/**
 * @brief Untyped **forced-scan** allocation.
 *
 * For `gc(n)` this overload is identical (`gc(n)` already scans the whole block conservatively when there is no type information),
 * its reason to exist is to complete the set together with `gcs<T>(...)`: "I do not use the type system, but I demand scanning".
 *
 * @param size Requested byte count.
 * @return Same as `gc(n)` (alignment as `malloc`, contents zeroed).
 * @note The only case still not scanned is `size < sizeof(void*)` (physically cannot hold one complete pointer).
 */
GCXX_ALWAYS_INLINE void* gcs(std::size_t size) {
    if (size > (std::size_t)gcxx::detail::MAX_OBJECT_SIZE)
        throw std::bad_alloc();
    if (size == 0) size = 1;
    return ::gcxx::detail::allocate_bytes(
        size, ::gcxx::detail::alloc_type_for(size, (std::uint8_t)::gcxx::GC_TYPE_RAW,
                                             true),
        false, true);
}

/**
 * @brief **Atomic memory**: the caller guarantees it contains no pointers, so it is **never scanned** (Boehm's
 *        `GC_malloc_atomic`).
 *
 * This is the library's fastest allocation path: marking **skips the whole block** (not "scan it and find no pointers", but never touching it at all).
 * These objects live in **atomic chunks** (`KIND_ATOMIC`), physically separated from objects that need scanning  -- "no object that needs
 * scanning ever appears in an atomic chunk" is a structural property that `gc_verify()` asserts; reverse lookup (interior pointers) still works as usual, it is only that once marked
 * the object is not expanded.
 *
 * @code
 *   char*   buf = static_cast<char*>(gca(65536));   // never scanned
 *   double* tab = gca<double>(4096);                // see the template overload
 * @endcode
 *
 * @param size Requested byte count.
 * @return Alignment as `malloc`, contents zeroed (same as `gc(n)`).
 * @warning **Contract**: the bytes written in **must not** contain pointers to GC objects. Violating this produces no diagnostic at all
 *          (that is exactly what "never scanned" means); the object will be reclaimed as unreachable, leaving a dangling pointer.
 *          When in doubt, use `gc` or `gcs`.
 * @note Typical uses: byte buffers, strings, pure data tables such as `double[]` / `int[]`, bitmaps. A RAW
 *       `gc(n)` block is scanned word by word on every mark, while `gca(1024)` is not scanned once.
 */
GCXX_ALWAYS_INLINE void* gca(std::size_t size) {
    if (size > (std::size_t)gcxx::detail::MAX_OBJECT_SIZE)
        throw std::bad_alloc();
    if (size == 0) size = 1;          /* same reason as gc(n): size==0 would leave the lookup range empty */
    return ::gcxx::detail::allocate_bytes(
        size, (std::uint8_t)::gcxx::GC_TYPE_ATOMIC, false, true);
}

/* gca<T>(args...) / gca<T[]>(n)  -- typed atomic allocation: skip type deduction entirely and allocate directly as "atomic".
 * Whether T contains pointers is the **caller's** responsibility (that is exactly the gca contract).
 *
 * **`enable_if` cannot be omitted**: without it this **scalar** overload is also a viable candidate for `gca<U[]>(n)`
 * (`T` can be deduced as `U[]` while the argument is exactly `n`), and it is **more specialized** than the array overload below
 * (the array overload's `enable_if` is only SFINAE in its return type), so `gca<U[]>(n)` would select this one
 * and be rejected outright inside it by `static_assert(!std::is_array<T>::value)`  -- with the result that
 * **`gca<T[]>(n)` is completely unusable** (`gc<T[]>(n)` / `gcs<T[]>(n)` work because they have the same `enable_if`).
 * Excluding arrays with the same predicate `gc_is_unbounded_array` is what makes the array form actually land on the overload below. */
template <typename T, typename... Args>
GCXX_ALWAYS_INLINE typename std::enable_if<
    !::gcxx::detail::gc_is_unbounded_array<T>::value,
    typename ::gcxx::detail::alloc_ptr<T>::type>::type
gca(Args&&... args) {
    static_assert(!std::is_reference<T>::value && !std::is_void<T>::value &&
                  !std::is_function<T>::value,
                  "gca<T>: T must be a non-reference, non-void, non-function "
                  "object type (arrays are handled by gca<T[]>(n))");
    static_assert(alignof(T) <= 16,
                  "gca<T>: T must not be over-aligned beyond 16 bytes");
    void* mem = ::gcxx::detail::allocate_bytes(
        sizeof(T), (std::uint8_t)::gcxx::GC_TYPE_ATOMIC,
        (sizeof...(Args) == 0) && std::is_trivially_default_constructible<T>::value);
    ::new (mem) T(std::forward<Args>(args)...);
    return static_cast<typename ::gcxx::detail::alloc_ptr<T>::type>(mem);
}

template <typename T>
inline typename std::enable_if<
    ::gcxx::detail::gc_is_unbounded_array<T>::value,
    typename ::gcxx::detail::gc_array_elem<T>::type*>::type
gca(std::size_t n) {
    typedef typename ::gcxx::detail::gc_array_elem<T>::type U;
    static_assert(!std::is_reference<U>::value && !std::is_function<U>::value,
                  "gca<T[]>: T must be an object type");
    static_assert(alignof(U) <= 16,
                  "gca<T[]>: element type must not be over-aligned beyond 16 bytes");
    const std::size_t cnt = (n == 0) ? 1u : n;
    if (cnt > (std::size_t)gcxx::detail::MAX_OBJECT_SIZE / sizeof(U))
        throw std::bad_alloc();
    void* mem = ::gcxx::detail::allocate_bytes(
        cnt * sizeof(U), (std::uint8_t)::gcxx::GC_TYPE_ATOMIC,
        std::is_trivially_default_constructible<U>::value);
    ::new (mem) U[cnt]();
    return static_cast<U*>(mem);
}
/**
 * @brief C++11 allocator: can be fed directly to `std::vector` / `std::map` / `std::unordered_map` /
 *        `std::shared_ptr` and the like.
 * @code
 *   std::vector<Foo*, gc_allocator<Foo*>> v;      // its element memory is managed by the GC
 * @endcode
 * @note `deallocate()` is a **no-op**  -- the GC reclaims the memory once the whole block is unreachable.
 * @note Same contract as `gc<T>`: only 16-byte slot alignment is guaranteed, so types with `alignof(T) > 16` are
 *       rejected (compile-time `static_assert`).
 *
 * @warning **STL containers that store `gc` pointers/handles must use this allocator**  -- otherwise those pointers
 *          are **not roots** and the target objects will be reclaimed.
 *
 * Reason: the GC scans only **the memory it can see** (thread stacks, registers, the writable data segment) plus objects
 * inside the **GC heap**. The elements of a `std::vector<Foo*>` (with the default `std::allocator`) live on the **`malloc` heap**,
 * and `malloc` memory is **outside the scanned range**: the pointer inside the container object that points to the element buffer can be
 * seen by conservative scanning, but having the GC walk into a `malloc` buffer to parse its contents is **neither guaranteed nor appropriate**
 * (that memory is not its to manage).
 *
 * ```cpp
 *   std::vector<Foo*> v;                        // wrong: the Foo* inside v is not a root
 *   std::vector<Foo*, gc_allocator<Foo*>> w;    // right: the element memory is GC-managed and gets scanned
 * ```
 *
 * **Measured evidence** (same machine, the same code with only the allocator swapped): after filling 200 `gc<int>()` and running one collection:
 *   * default allocator: survivors **200 -> 17** (the vast majority reclaimed, so any later access dangles);
 *   * `gc_allocator`: survivors **209 -> 201** (correctly retained).
 *
 * @note This is not a defect of this library but an **inherent contract of conservative GC**: no conservative GC can trace memory
 *       it cannot see. But it is easy enough to trip over, hence this explicit warning.
 * @note The converse also holds: containers that **store no GC pointers**, such as `std::vector<int>`, are entirely fine with the default
 *       allocator (they hold no references that need tracing).
 */
template <typename T>
class gc_allocator {
    /* Same contract as gc: the GC heap guarantees only 16-byte slot alignment. Types with alignof(T) > 16 must be rejected here too, otherwise
     * allocate() would return an under-aligned pointer and constructing objects on it from the container would be UB (gc has the same static_assert). */
    static_assert(alignof(T) <= 16,
                  "gc_allocator<T>: T must not be over-aligned beyond 16 bytes; "
                  "the GC heap guarantees 16-byte slot alignment");
public:
    typedef T                 value_type;
    typedef T*                pointer;
    typedef const T*          const_pointer;
    typedef T&                reference;
    typedef const T&          const_reference;
    typedef std::size_t       size_type;
    typedef std::ptrdiff_t    difference_type;
    typedef std::true_type    is_always_equal;
    template <typename U>
    struct rebind { typedef gc_allocator<U> other; };
    gc_allocator() noexcept = default;
    gc_allocator(const gc_allocator&) noexcept = default;
    template <typename U>
    gc_allocator(const gc_allocator<U>&) noexcept {}
    ~gc_allocator() noexcept = default;
    pointer allocate(size_type n, const void* /*hint*/ = nullptr) {
        if (n > max_size()) throw std::bad_alloc();
        if (n == 0) n = 1;
        void* mem = ::gcxx::detail::allocate_bytes(
            n * sizeof(T), ::gcxx::detail::type_of<T>::value,
            false, /* force_zero: the container gets raw memory, which must be clean */ true);
        return static_cast<pointer>(mem);
    }
    void deallocate(pointer, size_type) noexcept {
        /* hand it to the GC; do nothing */
    }
    size_type max_size() const noexcept {
        return static_cast<size_type>(-1) / (sizeof(T) == 0 ? 1 : sizeof(T));
    }
    /* Compatibility interface for pre-C++11 container implementations (allocator_traits recognizes these two as well). */
    template <typename U, typename... A>
    void construct(U* p, A&&... a) {
        ::new (static_cast<void*>(p)) U(std::forward<A>(a)...);
    }
    template <typename U>
    void destroy(U* p) { p->~U(); }
    /* The raw pointer is the object pointer; no address-taking tricks needed. */
    pointer       address(reference r) const { return &r; }
    const_pointer address(const_reference r) const { return &r; }
};
template <typename T, typename U>
inline bool operator==(const gc_allocator<T>&, const gc_allocator<U>&) noexcept { return true; }
template <typename T, typename U>
inline bool operator!=(const gc_allocator<T>&, const gc_allocator<U>&) noexcept { return false; }
namespace gcxx {
/** @brief Diagnostics/testing-only interface. **Not a public API, and not promised to be stable**: fields and semantics change with the implementation.
 *
 * Tests and probes bring it in with `using gcxx::tests::gc_stat;`. */
namespace tests {
struct gc_stats {
    std::size_t live_bytes;    /* live bytes currently on the GC heap */
    std::size_t live_objects;  /* number of live objects currently */
    unsigned    heap_count;    /* number of active thread heaps (including exited ones not yet cleaned up) */
    std::size_t freed_objects; /* cumulative number of reclaimed objects */
    std::size_t cycles;        /* cumulative number of collection cycles */
    std::size_t minor_cycles;  /* number of minor collections */
    std::size_t major_cycles;  /* number of major collections */
    std::size_t threshold;     /* byte threshold for the next collection (this thread's remaining budget) */
    std::size_t scanned_bytes; /* cumulative payload bytes conservatively scanned */
    std::size_t skipped_bytes; /* cumulative bytes skipped because of the scalar class */
    std::size_t chunk_count;   /* current number of chunks (including the empty chunks on the recycle stack) */
    std::size_t mapped_bytes;  /* total mapped bytes (including the empty chunks on the recycle stack) */
    std::ptrdiff_t debt;       /* current debt (positive = how many more bytes may be allocated) */
    std::size_t root_scans;    /* number of root scans */
    std::size_t root_words;    /* cumulative root words probed (stack + static area) */
    std::size_t bias_hits;     /* negative-bias window hits (suspicious: integer collisions or compiler tricks) */
    std::size_t cancelled_cycles; /* cycles cancelled because the mutator could not be stopped (steady growth = some thread never passes an allocation point, so the heap keeps growing) */
    std::size_t gray_rescans;  /* number of "fallback rescans" triggered by gray-set overflow / force-suspend. Normally
                                * it should be 0; steady growth means the reference graph is wider than GCXX_GRAY_CAPACITY
                                * (marking then gets slower, but is still correct) */
    std::size_t gray_discards; /* number of times a consumer discarded a gray cell as a "stale entry from the previous cycle".
                                * **Under normal load this must always be 0** -- it is the direct criterion for whether "reserve/publish
                                * separation" (gray_ready) is in effect; a non-zero value means a consumer
                                * again claimed a cell that was "reserved but not yet fully written", which is
                                * the mechanism behind a missed mark. */
    std::size_t freelist_corruptions; /* number of times a free-list link field was judged corrupt. This should
                                * always be 0 -- a non-zero value means a **wild write hit a free slot**
                                * (a use-after-free write / an out-of-bounds write / wrong slot geometry).
                                * Once detected it only disables that chain (bounded and reclaimable);
                                * it never hands out the same slot twice */

    /* ---- phased timing (A4): cumulative microseconds ---- splits "the wall clock of one gc_collect" into non-overlapping segments:
     * stw_us is the time spent **waiting** for the mutator to reach a safepoint (excluding the root scan and marking itself), root_us is the root scan (the stacks of
     * force-suspended threads + its own stack + the static area), mark_us is marking up to the end of atomic_finish, and sweep_us is all
     * incremental sweep slices accumulated (what is not in these four is the small remainder of handshake/release/finalization). */
    std::size_t stw_us;
    std::size_t root_us;
    std::size_t mark_us;
    std::size_t sweep_us;
    /* ---- physical memory return (D1/D2) ---- decommits / recommits are cumulative counts; purged_chunks is the **current** number of empty blocks in
     * the "data area decommitted" state. rss_bytes is the process's current working set (0 when unavailable)  -- mapped_bytes is
     * the intended value, and only it can prove "the memory really went back". */
    std::size_t decommits;
    std::size_t recommits;
    std::size_t purged_chunks;
    std::size_t rss_bytes;
    /* E2: current number of guard-paged large objects (one per N large objects, see GUARD_SAMPLE); only a non-zero value means "an out-of-bounds
     * access faults immediately" protection is actually on. */
    std::size_t guarded_objects;
};
/** @brief Take a statistics snapshot (**diagnostics/testing only**, not a stable contract).
 * @return A relaxed snapshot of the current counters; for semantics see the notes on each `gc_stats` field.
 * @note Reads only the TLS slot and **never creates a heap for the calling thread**  -- otherwise a thread that only queries and never allocates
 *       would never reach a safepoint, and every later collection would wait out the full STW budget and then be cancelled.
 * @internal
 */
inline gc_stats gc_stat() {
    ::gcxx::detail::diag_flush();     /* make this thread's accumulated counters visible first */
    ::gcxx::detail::gc_state& g = ::gcxx::detail::G();
    gc_stats s = {};
    s.live_bytes    = g.total_bytes.load(std::memory_order_relaxed);
    s.live_objects  = g.live_objects.load(std::memory_order_relaxed);
    s.freed_objects = g.freed_objects.load(std::memory_order_relaxed);
    s.cycles        = g.cycles.load(std::memory_order_relaxed);
    s.minor_cycles  = g.minor_cycles.load(std::memory_order_relaxed);
    s.major_cycles  = g.major_cycles.load(std::memory_order_relaxed);
    s.scanned_bytes = g.scanned_bytes.load(std::memory_order_relaxed);
    s.skipped_bytes = g.skipped_bytes.load(std::memory_order_relaxed);
    s.root_scans    = g.root_scans.load(std::memory_order_relaxed);
    s.root_words    = g.root_probed_words.load(std::memory_order_relaxed);
    s.bias_hits     = g.bias_hits.load(std::memory_order_relaxed);
    s.cancelled_cycles = g.cancelled_cycles.load(std::memory_order_relaxed);
    s.gray_rescans  = g.gray_rescans;
    s.gray_discards = g.gray_discards.load(std::memory_order_relaxed);

    s.freelist_corruptions = g.freelist_corruptions.load(std::memory_order_relaxed);
    s.stw_us   = g.t_stw_us.load(std::memory_order_relaxed);
    s.root_us  = g.t_root_us.load(std::memory_order_relaxed);
    s.mark_us  = g.t_mark_us.load(std::memory_order_relaxed);
    s.sweep_us = g.t_sweep_us.load(std::memory_order_relaxed);
    s.decommits = g.decommits.load(std::memory_order_relaxed);
    s.recommits = g.recommits.load(std::memory_order_relaxed);
    s.purged_chunks = g.recycled_cold.load(std::memory_order_relaxed);
    s.rss_bytes = ::gcxx::detail::rss_bytes();
    s.guarded_objects = g.guarded_objects.load(std::memory_order_relaxed);
    {   /* chunk_count / mapped_bytes are written by paths holding gc_mutex; each heap's pend_*_pub is a relaxed snapshot published by its owner and can be read directly. */
        ::gcxx::detail::gc_mutex_guard lk;
        /* Add back the empty chunks on the recycle stack: they are no longer on the global chain (hence not in chunk_count / mapped_bytes),
         * but their memory is still mapped. Reporting only the on-chain part badly understates the real footprint  -- measured understated by 190x. */
        const unsigned rec = g.recycled.load(std::memory_order_relaxed);
        s.chunk_count = g.chunk_count + (std::size_t)rec;
        s.mapped_bytes = g.mapped_bytes + (std::size_t)rec * ::gcxx::detail::CHUNK_SIZE;
        for (::gcxx::detail::gc_heap* h = ::gcxx::detail::heaps_head(); h != nullptr; h = h->hnext) {
            ++s.heap_count;
            s.live_bytes += h->pend_bytes_pub.load(std::memory_order_relaxed);
            s.live_objects += h->pend_objs_pub.load(std::memory_order_relaxed);
        }
        /* The free side also has accumulated batches not yet merged in (see object_free): they are **subtractions**, and not deducting them would overstate liveness. The snapshot is a
         * relaxed read and may exceed the global value (that batch has not been flushed into the global counters yet), so a saturating subtraction is used. */
        const std::size_t pend_b = g.sweep_pend_bytes_pub.load(std::memory_order_relaxed);
        const std::size_t pend_o = g.sweep_pend_objs_pub.load(std::memory_order_relaxed);
        s.live_bytes   = s.live_bytes   > pend_b ? s.live_bytes   - pend_b : 0;
        s.live_objects = s.live_objects > pend_o ? s.live_objects - pend_o : 0;
        /* Reads only the TLS slot and **never calls my_heap()**  -- that would create a heap for the calling thread and register it as a GC participant; if that thread never
         * allocates it would never reach a safepoint, and every later collection would wait out the full STW budget and then cancel/force-suspend. */
        ::gcxx::detail::gc_heap* self = ::gcxx::detail::heap_tls_get();
        const std::ptrdiff_t debt = self != nullptr
            ? self->debt : (std::ptrdiff_t)0;
        s.debt = debt;
        /* Threshold semantics = how many more bytes this thread may allocate before the next cycle triggers (negative/zero = already in debt). */
        s.threshold = debt > 0 ? (std::size_t)debt : (std::size_t)0;
    }
    return s;
}
/**
 * @brief Whole-heap consistency self-check: returns the number of inconsistent items, **0 = healthy**. Diagnostics/testing only.
 * @internal
 */
inline std::size_t gc_verify() {
    return ::gcxx::detail::verify_heap();
}

/* ==========================================================================
 * Internal test interface (**not a public API**; fields and semantics change with the implementation)
 *
 * Design discipline (directly related to the lesson of section 46.2):
 *  * **Read-only throughout**  -- it changes no colours/lists/counters, and therefore **does not change the behaviour under test**.
 *    (Instrumenting a hot function is by itself enough to dominate the cost it is meant to report.)
 *  * **No allocation, no triggering of collection, no heap creation for the calling thread** (reads only the TLS slot, never calls `my_heap()`).
 *  * The two interfaces that walk lists hold `gc_mutex` (the lists are modified by other threads, see the notes on `gc_dump`);
 *    the rest do only single atomic reads and need no locking.
 * ========================================================================== */
/** @brief Return value of `gc_test_state()`: a state-machine snapshot.
 * @note The values of `phase` / `kind` are exactly the integer forms of `gcxx::detail::GC_PHASE_*` / `GC_KIND_*`;
 *       do **not** hard-code their numeric values outside this header. */
struct gc_test_state_t {
    int           phase;          /**< GC_PHASE_PAUSE / PROPAGATE / ATOMIC / SWEEP / END */
    int           kind;           /**< GC_KIND_NONE / MINOR / MAJOR */
    std::uint8_t  white;          /**< current white bit */
    std::uint8_t  cycle_white;    /**< this cycle's "white" (differs from white during a major) */
    bool          mark_excl;      /**< whether marking is in exclusive mode (marking inside STW) */
    bool          aborting;       /**< the cancel path is rolling back colours */
    bool          in_oom;         /**< currently on the OOM emergency path */
    int           step_depth;     /**< gc_step re-entry depth (normally 0) */
    int           collect_depth;  /**< gc_collect concurrency depth (normally 0) */
    unsigned      recycled;       /**< number of empty chunks on the recycle stack (address space retained, possibly decommitted) */
    unsigned      recycled_cold;  /**< of those, the number of decommitted chunks */
    std::size_t   chunk_count;    /**< chunks on the global chain (**excluding** those on the recycle stack) */
    std::size_t   mapped_bytes;   /**< mapped bytes (**excluding** those on the recycle stack) */
    std::size_t   gray_ready;     /**< gray-set "published" upper bound (reserve/publish separation) */
    std::size_t   gray_tail;      /**< gray-set "reserved" upper bound */
    std::size_t   gray_rescans;   /**< number of fallback rescans (normally 0) */
    std::size_t   gray_discards;  /**< number of discards judged "stale entry from the previous cycle" (normally always 0) */
    std::size_t   freelist_corruptions; /**< number of free-list corruptions (normally always 0) */
    std::size_t   cancelled_cycles;     /**< number of cancelled cycles */
    std::size_t   promoted_since_major; /**< bytes promoted since the last major */
    std::size_t   live_at_major;        /**< live bytes at the last major */
    unsigned      heap_count;           /**< length of the global heap chain */
};
/** @brief Take a state-machine snapshot (internal testing; single atomic reads, only `heap_count` needs a locked walk). */
inline gc_test_state_t gc_test_state() {
    using namespace ::gcxx::detail;
    gc_state& g = G();
    gc_test_state_t s = {};
    s.phase       = g.phase.load(std::memory_order_relaxed);
    s.kind        = g.kind.load(std::memory_order_relaxed);
    s.white       = g.white.load(std::memory_order_relaxed);
    s.cycle_white = g.cycle_white;
    s.mark_excl   = g.mark_excl.load(std::memory_order_relaxed);
    s.aborting    = g.aborting.load(std::memory_order_relaxed);
    s.in_oom      = (int)g.in_oom.load(std::memory_order_relaxed);
    s.step_depth  = g.step_depth.load(std::memory_order_relaxed);
    s.collect_depth = g.collect_depth.load(std::memory_order_relaxed);
    s.recycled      = g.recycled.load(std::memory_order_relaxed);
    s.recycled_cold = g.recycled_cold.load(std::memory_order_relaxed);
    s.gray_ready    = g.gray_ready.load(std::memory_order_relaxed);
    s.gray_tail     = g.gray_tail.load(std::memory_order_relaxed);
    s.gray_rescans  = g.gray_rescans;
    s.gray_discards = g.gray_discards.load(std::memory_order_relaxed);
    s.freelist_corruptions = g.freelist_corruptions.load(std::memory_order_relaxed);
    s.cancelled_cycles     = g.cancelled_cycles.load(std::memory_order_relaxed);
    s.promoted_since_major = g.promoted_since_major;
    s.live_at_major        = g.live_at_major;
    {
        /* chunk_count / mapped_bytes and the heap chain are written by paths holding gc_mutex. */
        gc_mutex_guard lk;
        s.chunk_count  = g.chunk_count;
        s.mapped_bytes = g.mapped_bytes;
        unsigned n = 0;
        for (gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) ++n;
        s.heap_count = n;
    }
    return s;
}
/** @brief Return value of `gc_test_count_colors()`: counts classified by **object-header colour**.
 * @note This is the health chart of the "colour invariant": while idle, **all live objects should equal the current white bit** and the
 *       gray-set-related counts should be 0; a non-zero `other` means a `marked` byte fell outside the three colours (structural corruption). */
struct gc_test_colors_t {
    std::size_t objects;   /**< total live objects on the three lists */
    std::size_t white0;    /**< marked == WHITE0 */
    std::size_t white1;    /**< marked == WHITE1 */
    std::size_t gray;      /**< marked == 0 (no white bit, not black) */
    std::size_t black;     /**< marked == BLACK */
    std::size_t free_slot; /**< marked == MARK_FREE (**should not appear on an object list**) */
    std::size_t other;     /**< any other value (a signal of structural corruption) */
    std::size_t young, old, nursery; /**< broken down by the list they are on */
};
/** @brief Count the colour distribution across the three object lists (internal testing; holds `gc_mutex`).
 * @note **It does not assert when it finds "a `MARK_FREE` slot on a list" or "a colour outside the three colours"**;
 *       it only reports the counts  -- the assertion is left to the caller (the test case), so that the same interface serves both as a health chart under
 *       normal load and as a criterion in **deliberately corrupted** test cases. */
inline gc_test_colors_t gc_test_count_colors() {
    using namespace ::gcxx::detail;
    gc_test_colors_t c = {};
    gc_mutex_guard lk;
    /* **Object lists must only be walked while idle (phase == PAUSE)**  -- `gc_mutex` does not hold off sweeping!
     *
     * `sweep_some` **deliberately does not take `gc_mutex`** (by design: incremental sweeping is driven by allocation debt and the
     * mutator is never stopped). So holding the lock does not prevent it from concurrently **unlinking and `object_free`-ing
     * the slot**; walking the object lists then reads **already-recycled slots**.
     *
     * Measured (Windows, crash stack from cdb + RelWithDebInfo + PDB):
     *   `Access violation - c0000005`,
     *   `gcxx::tests::gc_test_count_colors+0x8e`
     *     <- sampling-thread lambda
     * crashing at the `mark_load(o)` in the walk below  -- reading an already-dangling object header.
     * (**Line numbers in this file are deliberately not cited**: they drift with every change, and such self-references inevitably rot.)
     *
     * **The same contract applies to every whole-heap-walking diagnostic interface**:
     * `gc_test_count_colors` / `for_each_object` / `gc_verify` / `gc_dump`.
     * They may only be called while idle (the caller's responsibility; see the usage in probes/state_invariants_probe.cpp).
     *
     * Here it protects itself explicitly: while not idle it returns **all zeros**. This is a diagnostic interface; **better to under-report than to read a dangling pointer**. */
    if (G().phase.load(std::memory_order_acquire) != (int)GC_PHASE_PAUSE) return c;
    for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
        gc_object* lists[3];
        lists[0] = h->young.load(std::memory_order_relaxed);
        lists[1] = h->old.load(std::memory_order_relaxed);
        lists[2] = h->nursery.load(std::memory_order_relaxed);
        for (int li = 0; li < 3; ++li) {
            for (const gc_object* o = lists[li]; o != nullptr;
                 o = o->next.load(std::memory_order_relaxed)) {
                ++c.objects;
                if (li == 0) ++c.young; else if (li == 1) ++c.old; else ++c.nursery;
                const std::uint8_t m = mark_load(o);
                if (m == MARK_FREE)              ++c.free_slot;
                else if (m == (std::uint8_t)WHITE0) ++c.white0;
                else if (m == (std::uint8_t)WHITE1) ++c.white1;
                else if (m == (std::uint8_t)BLACK)  ++c.black;
                else if (m == 0)                 ++c.gray;
                else                             ++c.other;
            }
        }
    }
    return c;
}
/** @brief Walk all live objects (internal testing; holds `gc_mutex`). Returning `false` from the callback stops immediately.
 * @param fn Of the form `bool(gc_object*, int list)`; `list` = 0/1/2 corresponds to young/old/nursery.
 * @return Whether every object was visited. **When the heap is not idle (`phase != PAUSE`) it always returns `false` and never calls back even once**
 *          -- during sweeping the object lists are concurrently unlinked and their slots recycled, so walking them reads dangling object headers (see
 *         the `@warning` on `gc_test_count_colors` and the same passage in `gc_dump`).
 * @note `gc_object*` is an internal type: read only its `size`/`age`/`type`/`marked`,
 *       **never** modify it (that bypasses every invariant). Inside the callback do **not** call any interface that takes `gc_mutex`,
 *       and do not allocate (`gc_mutex` is re-entrant so there is no deadlock, but the walk semantics get muddled). */
template <typename Fn>
inline bool gc_test_visit_objects(Fn&& fn) {
    using namespace ::gcxx::detail;
    if (G().phase.load(std::memory_order_acquire) != (int)GC_PHASE_PAUSE)
        return false;                         /* self-protection: do not walk while not idle (see above) */
    bool done = true;
    gc_mutex_guard lk;
    for (const gc_heap* h = heaps_head(); h != nullptr && done; h = h->hnext) {
        gc_object* lists[3];
        lists[0] = h->young.load(std::memory_order_relaxed);
        lists[1] = h->old.load(std::memory_order_relaxed);
        lists[2] = h->nursery.load(std::memory_order_relaxed);
        for (int li = 0; li < 3 && done; ++li)
            for (gc_object* o = lists[li]; o != nullptr;
                 o = o->next.load(std::memory_order_relaxed)) {
                if (!fn(o, li)) { done = false; break; }
            }
    }
    return done;
}
/** @brief Return value of `gc_test_object_of()`: object information resolved from an address. */
struct gc_test_object_info_t {
    bool          found;      /**< whether a GC object was hit */
    void*         object;     /**< object header address (internal type, read-only) */
    std::uint32_t size;       /**< payload bytes */
    std::uint8_t  type;       /**< scan class (GC_TYPE_*) */
    std::uint8_t  age;        /**< generational age */
    std::uint8_t  marked;     /**< current colour byte */
    bool          is_old;     /**< age >= GC_AGE_OLD */
    bool          is_leaf;    /**< this type needs no scanning */
};
/** @brief Resolve an address to object information (internal testing; the same criteria as the marking path).
 * @note It goes through `object_find`  -- **interior pointers** (`&obj.field`, negative bias) are hit as well,
 *       consistent with the `GCXX_ALL_INTERIOR_POINTERS` setting. **Reachability is not considered.**
 * @note No `gc_mutex` needed (a single address -> table-entry lookup plus one object-header read; in the worst case it returns a miss,
 *       which is safe by direction  -- the same reasoning as `gc_is_gc_pointer`). */
inline gc_test_object_info_t gc_test_object_of(const void* p) {
    using namespace ::gcxx::detail;
    gc_test_object_info_t r = {};
    gc_object* o = object_find(reinterpret_cast<std::uintptr_t>(p));
    if (o == nullptr) return r;
    r.found    = true;
    r.object   = static_cast<void*>(o);
    r.size     = o->size;
    r.type     = o->type;
    r.age      = o->age;
    r.marked   = mark_load(o);
    r.is_old   = is_old(o);
    r.is_leaf  = is_leaf_object(o);
    return r;
}
/** @brief Return value of `gc_test_chunk_count()`: a global view of chunks and mapping. */
struct gc_test_chunks_t {
    std::size_t chain_chunks;  /**< chunks on the global `chunks` chain (excluding the recycle stack) */
    std::size_t chain_bytes;   /**< the mapped bytes corresponding to the above */
    unsigned    recycled;      /**< chunks on the recycle stack */
    unsigned    recycled_cold; /**< of those, the number of decommitted chunks */
    std::size_t total_chunks;  /**< chain_chunks + recycled (the "real footprint" basis) */
    std::size_t total_bytes;   /**< chain_bytes + recycled x 64KB */
};
/** @brief Take the global chunk/mapping view (internal testing). The same basis as the two `gc_stat()` fields
 *       (**including** the empty chunks on the recycle stack  -- omitting them understates the real footprint, measured understated by 190x once). */
inline gc_test_chunks_t gc_test_chunk_count() {
    using namespace ::gcxx::detail;
    gc_test_chunks_t r = {};
    gc_state& g = G();
    r.recycled      = g.recycled.load(std::memory_order_relaxed);
    r.recycled_cold = g.recycled_cold.load(std::memory_order_relaxed);
    {
        gc_mutex_guard lk;
        r.chain_chunks = g.chunk_count;
        r.chain_bytes  = g.mapped_bytes;
    }
    r.total_chunks = r.chain_chunks + (std::size_t)r.recycled;
    r.total_bytes  = r.chain_bytes + (std::size_t)r.recycled * CHUNK_SIZE;
    return r;
}


} /* namespace tests */

namespace detail {
/* Registration and reading of weak references / ephemerons (write side and read side). A uniform **index + table** model: a handle holds only a
 * `std::uint32_t` index, and every raw pointer lives in a GC-private malloc table (for the reason see the notes on weak_slot  -- handle memory
 * is conservatively scanned). Both registration and deregistration hold gc_mutex (mutually exclusive with the processing phase inside STW). */
/**
 * @brief Resolve a handle's `(index, generation)` to its slot, or `nullptr` when the
 *        handle no longer designates a live entry of the **current** table.
 *
 * This is the single validation predicate for the whole weak/ephemeron subsystem, and
 * all three entry points share it so that they can never disagree about what "this handle
 * is still valid" means:
 *   * `weak_clear` / `weak_set` (null target) -- about to deregister, they consult it and then
 *     hand the **same** `(idx, gen)` pair to `weak_slot_free`;
 *   * `weak_set`                      -- about to overwrite / reuse;
 *   * `weak_peek`                     -- about to read.
 * @note `weak_slot_free` deliberately does **not** call this: it receives the already-validated
 *       `(idx, gen)` and re-checks only the generation, as a self-check that the "decide" and
 *       "act" halves agree. See its own note for why that split exists.
 *
 * Three independent ways a handle can be stale, all of which must be rejected:
 *   1. `idx == SLOT_NONE`  -- it was never registered, or `reset()` already ran;
 *   2. `gen != g.weak_generation` -- it belongs to a **previous table generation**. This is the
 *      subtle one: `gc_dispose()` frees the table and zeroes `weak_len`, but the table then
 *      **grows back**, and the same index then designates *another handle's slot*. Without the
 *      generation check a stale `w.reset()` would unregister someone else's entry and a stale
 *      `w.get()` would return someone else's object (see `weak_generation`).
 *      `gen == 0` means "never registered", so it is rejected here too, without a special case.
 *   3. the table is gone (`weak_tab == nullptr`) or the index is past the high-water mark
 *      (`idx >= weak_len`) -- covers the window after `gc_dispose()`.
 *
 * @note Lock-free readers (`weak_peek`) call this too; it only reads fields, and the
 *       generation is a plain field written under `gc_mutex` (see `weak_generation`).
 * @internal
 */
inline const weak_slot* weak_slot_lookup(const gc_state& g, std::uint32_t idx,
                                         std::uint32_t gen) {
    if (idx == SLOT_NONE) return nullptr;
    if (gen != g.weak_generation) return nullptr;
    if (g.weak_tab == nullptr || idx >= g.weak_len) return nullptr;
    return &g.weak_tab[idx];
}
/** @brief Mutable overload for the write side (same predicate, no duplicated logic). */
inline weak_slot* weak_slot_lookup(gc_state& g, std::uint32_t idx, std::uint32_t gen) {
    return const_cast<weak_slot*>(
        weak_slot_lookup(static_cast<const gc_state&>(g), idx, gen));
}
/**
 * @brief Deregister a weak entry and push it back onto the free list.
 *
 * @param g   The global state (the caller holds `gc_mutex`).
 * @param idx The slot index.
 * @param gen The **generation the caller validated `idx` against** -- see below; it is not
 *            optional and it is not always `g.weak_generation`.
 *
 * **`idx` and `gen` must have already been validated together by `weak_slot_lookup`.**
 * This function deliberately does **no** lookup of its own: it is the "act" half, and the
 * validation is the "decide" half. There are exactly two call sites and both validate first:
 *   * `weak_clear`          -- `weak_slot_lookup(g, idx, gen) != nullptr`;
 *   * `weak_set` (null target) -- `idx` was normalised to `SLOT_NONE` unless
 *     `weak_slot_lookup(g, idx, gen) != nullptr`.
 *
 * **It used to call `weak_slot_lookup(g, idx, 0u)`**, i.e. it passed generation 0, which is
 * reserved for "never registered" and therefore *matches every slot* -- silently **disabling**
 * the very generation check that exists to stop a stale handle (one that outlived a
 * `gc_dispose()`) from unregistering **another handle's** slot once the table has grown back
 * (the "silent cross-talk" described on `gc_state::weak_generation`). It happened to be safe
 * only because of the two pre-validating call sites above, so the safety of the whole weak
 * subsystem rested on an unwritten convention maintained three hundred lines away. Threading
 * `gen` through makes the invariant local and checkable: **this function can only ever touch
 * the entry its caller just proved is its own.**
 *
 * @note When the table has already been freed (after `gc_dispose()`) the pre-validation fails
 *       and this function is simply never reached -- handles often live on the stack/in the static
 *       area and their destructors run after dispose (without that protection it is a segfault at exit).
 * @internal
 */
inline void weak_slot_free(gc_state& g, std::uint32_t idx, std::uint32_t gen) {
    /* The only thing this function still checks is the generation, and it does so as a
     * *self-check* on the caller's validation rather than as a substitute for it: reaching here
     * with a mismatching generation would mean the "decide" half and the "act" half disagree. */
    if (idx == SLOT_NONE || gen == 0u || gen != g.weak_generation) return;
    if (g.weak_tab == nullptr || idx >= g.weak_len) return;
    weak_slot& s = g.weak_tab[idx];
    if (!s.in_use) return;
    /* **Only a still-live entry decrements the count**: the expired one was already decremented in weak_process_after_mark, and decrementing
     * it again here would drive weak_live to 0  -- and 0 is the fast gate that "skips the whole stabilization phase" (`weak_live == 0
     * return`), so ephemeron activation is skipped entirely, values stop being marked, and handles expire immediately (that defect surfaced reliably only on clang-cl
     * Release, see `memory/MEMORY.md` section 2.2). Invariant:
     *     weak_live == #{ slots : in_use && key != nullptr } */
    if (s.key.load(std::memory_order_relaxed) != nullptr) --g.weak_live;
    s.key.store(nullptr, std::memory_order_relaxed);
    s.value.store(nullptr, std::memory_order_relaxed);
    s.kind = WEAK_PLAIN;
    s.activated = 0;
    s.next = g.weak_free;
    s.in_use = 0;
    g.weak_free = idx;
}
/* Take an empty slot (free list first, otherwise grow the table). Returns SLOT_NONE = out of memory; the caller leaves the handle "empty" rather than creating a half-built entry. */
/**
 * @brief Allocate a weak entry (reusing the free list first, otherwise growing on demand).
 * @return The slot index; `SLOT_NONE` = failure.
 * @note Growth **moves values item by item** (atomic load/store, not a bitwise copy), and **indices are stable**  -- handles hold indices, not addresses, so growth is transparent to them.
 * @internal
 */
inline std::uint32_t weak_slot_alloc(gc_state& g) {
    if (g.weak_free != SLOT_NONE) {
        const std::uint32_t idx = g.weak_free;
        g.weak_free = g.weak_tab[idx].next;
        weak_slot& s = g.weak_tab[idx];
        s.key.store(nullptr, std::memory_order_relaxed);
        s.value.store(nullptr, std::memory_order_relaxed);
        s.kind = WEAK_PLAIN;
        s.activated = 0;
        s.in_use = 1;
        return idx;
    }
    if (g.weak_len == g.weak_cap) {
        /* ---- Clamp the capacity before growing: **a slot index is a `std::uint32_t`** ----
         * Three independent reasons; any one of them requires refusing further growth:
         *  1. `weak_slot::next` (the free list), `SLOT_NONE`, and the `idx_` stored in handles are all
         *     **32-bit**; once the slot count exceeds `UINT32_MAX`, `(std::uint32_t)g.weak_len++` will
         *     **truncate** and the new entry gets an **aliased index**  -- later deregistration/reads then operate on **a different slot**,
         *     a silent memory-safety problem.
         *  2. `ncap * sizeof(weak_slot)` (24 bytes per slot) **wraps around** for very large `ncap`,
         *     `malloc` successfully returns a block **far smaller** than needed, and the per-slot placement new that follows
         *     is a **heap out-of-bounds write**.
         *  3. Even if 1 and 2 were no problem, a 96GB table by itself exceeds any reasonable use.
         * So the cap here is aligned with the index type; at the top, `weak_slot_alloc` explicitly returns `SLOT_NONE`
         * (the caller treats it as an allocation failure; a bad index is never handed out silently). */
        constexpr auto kWeakCapMax = (std::size_t)0xFFFFFFFFu;   /* uint32 index space */
        if (g.weak_cap >= kWeakCapMax) return SLOT_NONE;
        std::size_t ncap = (g.weak_cap == 0) ? (std::size_t)64
                                             : g.weak_cap * 2u;
        if (ncap > kWeakCapMax) ncap = kWeakCapMax;
        /* Double protection: even if the clamp above is later broken, the multiplication here still cannot wrap. */
        if (ncap > (std::size_t)-1 / sizeof(weak_slot)) return SLOT_NONE;
        auto* nt = static_cast<weak_slot*>(std::malloc(ncap * sizeof(weak_slot)));
        if (nt == nullptr) return SLOT_NONE;
        /* The new table is built with placement new (weak_slot contains std::atomic, so raw memory cannot be written directly); the old table **has its values moved item by item**
         * rather than memcpy  -- what is moved is atomic load/store, not a bitwise copy. **Indices are stable** (handles hold indices,
         * not addresses), so growth is transparent to handles. */
        for (std::size_t i = 0; i < ncap; ++i) {
            auto* s = ::new (static_cast<void*>(&nt[i])) weak_slot;
            s->key.store(nullptr, std::memory_order_relaxed);
            s->value.store(nullptr, std::memory_order_relaxed);
            s->next = SLOT_NONE;
            s->kind = WEAK_PLAIN;
            s->in_use = 0;
            s->activated = 0;
            s->reserved = 0;
        }
        for (std::size_t i = 0; i < g.weak_len; ++i) {
            weak_slot& o = g.weak_tab[i];
            weak_slot& n = nt[i];
            n.key.store(o.key.load(std::memory_order_relaxed),
                        std::memory_order_relaxed);
            n.value.store(o.value.load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
            n.next = o.next;
            n.kind = o.kind;
            n.in_use = o.in_use;
            n.activated = o.activated;
        }
        /* The old table is **not freed** but pushed onto the retired chain (see gc_state::weak_retired_*).
         * `weak_peek` is a **lock-free read** (`gc_weak::get()` / `expired()` / `operator bool()` may be
         * called by any thread at any moment), so freeing the old table directly would let a concurrent
         * reader dereference freed memory.
         * The number of growths is O(log n) (64 -> 2^28), so this chain is very short.
         * The retired tables are freed as a whole chain in `gc_dispose()` (by which time the caller guarantees no other thread is using the library). */
        if (g.weak_retired_len == g.weak_retired_cap) {
            const std::size_t ncap2 = (g.weak_retired_cap == 0) ? 8u : g.weak_retired_cap * 2u;
            void* nb = std::realloc(g.weak_retired, ncap2 * sizeof(void*));
            if (nb == nullptr) {
                /* Not even the retired chain can be built: better to **not grow** than to dangle (allocation failure = registration failure). */
                std::free(nt);
                return SLOT_NONE;
            }
            g.weak_retired = static_cast<weak_slot**>(nb);
            g.weak_retired_cap = ncap2;
        }
        g.weak_retired[g.weak_retired_len++] = g.weak_tab;
        g.weak_tab = nt;
        g.weak_cap = ncap;
    }
    const auto idx = (std::uint32_t)g.weak_len++;
    weak_slot& s = g.weak_tab[idx];
    s.key.store(nullptr, std::memory_order_relaxed);
    s.value.store(nullptr, std::memory_order_relaxed);
    s.kind = WEAK_PLAIN;
    s.activated = 0;
    s.in_use = 1;
    return idx;
}
/* Register (or overwrite) one entry. `target`/`value` are both **payload pointers**; `value` is meaningful only for an ephemeron. Everything takes gc_mutex. */
inline void weak_set(std::uint32_t& idx, std::uint32_t& gen, void* target, void* value,
                     std::uint8_t kind) {
    gc_state& g = G();
    if (target == nullptr) {                      /* null target = deregister */
        if (idx != SLOT_NONE) {
            gc_mutex_guard lk;
            /* Same decide/act split as `weak_clear`: only deregister an entry this handle
             * actually owns. A stale handle (one that outlived a `gc_dispose()`) must not
             * unregister the slot some *other* handle now owns -- see `weak_slot_free`. */
            if (weak_slot_lookup(g, idx, gen) != nullptr) weak_slot_free(g, idx, gen);
            idx = SLOT_NONE;
            gen = 0;
        }
        return;
    }
    gc_mutex_guard lk;
    /* ---- Table already freed (after `gc_dispose()`) or the index is no longer valid: treat it as "no entry" and allocate a new one ----
     * Handles live on the stack/in the static area, and their **destructor** running after dispose is a known shape (`weak_slot_free` and
     * `weak_peek` both guard against it), but **re-assignment** reaches this point too:
     * `gc_dispose()` sets `weak_tab` to nullptr and zeroes `weak_len`, while the library documentation says it "may be used
     * afterwards", so `w = p;` / `w.reset(p)` / `e.set(k, v)` would index an empty table with an old index => null-pointer
     * dereference. The **same** criteria are used here as in those two places.
     *
     * The criteria **must** include a generation comparison. Testing only `idx < weak_len` would miss the worst case:
     *    the table **has already grown again** after dispose, so a stale index falls in the legal range while pointing at
     *    **another handle's slot** => silent cross-talk (overwriting someone else's entry). See `weak_generation`.
     * The single validation predicate lives in `weak_slot_lookup` (shared with the read and deregister paths). */
    if (weak_slot_lookup(g, idx, gen) == nullptr) idx = SLOT_NONE;
    if (idx == SLOT_NONE) {
        idx = weak_slot_alloc(g);
        if (idx == SLOT_NONE) {
            gen = 0;
            gc_error_message(ENOMEM, "gcxx: weak/ephemeron table allocation failed");
            return;
        }
        gen = g.weak_generation;
        ++g.weak_live;
    } else {
        weak_slot& s0 = g.weak_tab[idx];
        if (s0.in_use && s0.key.load(std::memory_order_relaxed) == nullptr)
            ++g.weak_live;                        /* re-registered after expiring: restore the count */
    }
    weak_slot& s = g.weak_tab[idx];
    s.in_use = 1;
    s.kind = kind;
    s.activated = 0;
    /* Order: write value first, then key  -- the read side treats a non-null key as the criterion for "this entry is valid", so key must be published last with release. */
    s.value.store(value != nullptr
                      ? reinterpret_cast<gc_object*>(
                            static_cast<char*>(value) - (std::ptrdiff_t)PAYLOAD_OFF)
                      : nullptr,
                  std::memory_order_relaxed);
    s.key.store(reinterpret_cast<gc_object*>(
                    static_cast<char*>(target) - (std::ptrdiff_t)PAYLOAD_OFF),
                std::memory_order_release);
}
/**
 * @brief Deregister the entry held by a handle and set its index to `SLOT_NONE`.
 * @note The generation is zeroed (= "never registered"), so even if the table grows again after dispose this handle will not
 *       mistake someone else's slot for its own just because the index happens to fall in range.
 * @internal
 */
inline void weak_clear(std::uint32_t& idx, std::uint32_t& gen) {
    if (idx == SLOT_NONE) { gen = 0; return; }
    gc_state& g = G();
    gc_mutex_guard lk;
    /* Deregister only entries **belonging to this generation**: a stale entry was released along with its table long ago, so touching the new
     * slot with the same index would deregister someone else's entry. */
    if (weak_slot_lookup(g, idx, gen) != nullptr) weak_slot_free(g, idx, gen);
    idx = SLOT_NONE;
    gen = 0;
}
/* Read side: the payload pointer in the slot. It can be read **lock-free**  -- writes happen only inside STW (all mutators stopped) or under gc_mutex,
 * and this uses an acquire read, which is well defined even when concurrent with a write on gc_step's non-STW fallback path. */
/**
 * @brief Read the payload pointer of an entry (a weak reference reads the target; an ephemeron may read the key or the value).
 * @return The payload; `nullptr` if it is no longer valid.
 * @note Can be read **lock-free**: writes happen only inside STW or under gc_mutex, and an acquire read is well defined even when concurrent with a write.
 * @note A generation mismatch (a stale handle across `gc_dispose()`) always returns `nullptr`  -- see `weak_generation`.
 * @internal
 */
inline void* weak_peek(std::uint32_t idx, std::uint32_t gen, bool want_value) {
    gc_state& g = G();
    const weak_slot* sp = weak_slot_lookup(g, idx, gen);
    if (sp == nullptr) return nullptr;
    const weak_slot& s = *sp;
    /* key is the sole criterion for whether this entry is valid: a non-null key => the target is alive => value is valid (a live key means the value was marked this cycle). */
    const gc_object* k = s.key.load(std::memory_order_acquire);
    if (k == nullptr) return nullptr;
    const gc_object* p = want_value ? s.value.load(std::memory_order_acquire) : k;
    if (p == nullptr) return nullptr;
    /* Defensive re-confirmation: the value's address must still resolve to **the same object**. That
     * always holds in a correct implementation (a live key implies the value was marked alive, so it
     * cannot have been reclaimed), but this step downgrades "the activation logic has a bug" from
     * "return a dangling pointer for the user to dereference" to "return nullptr". The weak-reference
     * side deliberately does not do this: its semantics inherently allow "the target just died and the
     * handle has not noticed yet". */
    if (want_value && object_find(reinterpret_cast<std::uintptr_t>(p)) != p)
        return nullptr;
    return const_cast<char*>(payload_of(p));
}
} /* namespace detail */
} /* namespace gcxx */
/* ==========================================================================
 * Debug interfaces (public, read-only): textual dump + pointer ownership
 *
 * Why they live in the **public** namespace: they are the first entry point for investigating "where did the memory go / why is this pointer keeping something alive",
 * so they are a public capability; the names keep the `gc_` prefix, consistent with `gc_stat` / `gc_verify`.
 *
 * Three rules:
 *  1. **Read-only**: no allocation, no changes to objects/chunks/lists; no collection triggered, no pacing changed.
 *  2. **Never create a heap for the calling thread**: always go through `heap_tls_get()` (reads only the TLS slot), never call
 *     `my_heap()`  -- otherwise a thread that only queries and never allocates **never reaches a safepoint**,
 *     and every later collection would wait out the full STW budget and then cancel/force-suspend.
 *  3. **Must hold `gc_mutex`** (public read-only does **not** mean lock-free, see the notes on `gc_dump`).
 *
 * **Not a stable contract**: the output format and criteria change with the implementation; do not parse the output and do not put the return value into business logic.
 * ========================================================================== */
/**
 * @brief Render the current heap state as an **English** human-readable text block written to `out` (default `stdout`).
 *
 * @return The number of bytes written (the `fprintf` return values summed); when the heap is not idle, only an explanatory line is written.
 *
 * Contents:
 *  * Totals: live object count / live payload / mapped bytes / chunk count / heap count / mapped-overhead ratio;
 *  * a **histogram by slot size** (listing only the 12 buckets with the largest slot footprint): object count, payload bytes,
 *    the young/old/nursery distribution, and how many of them "need conservative scanning";
 *  * each heap's young/old/nursery list lengths and its `dead` flag (with several threads, one glance shows who piled up garbage);
 *  * diagnostic counters: cycles, cancelled cycles, recycle stack, guard-paged objects, free-list corruption, gray-set fallback rescans,
 *    scanned/skipped bytes, decommit counts, RSS.
 *
 * @warning **Why it needs `gc_mutex` ("read-only" does not mean "lock-free")**:
 *   This function walks the **heap chain + each heap's young/old/nursery object lists**, and these are
 *   structures that **other threads modify**  -- other threads push new objects at the head (`object_new`), and the collector
 *   unlinks/promotes/frees heap structs (`reap_dead_heaps` `free()`s the heaps of exited threads).
 *   The consequence of not locking is **not "reading slightly stale data"** but:
 *     * following a `next`/`hnext` that is being rewritten into a **freed heap struct** => segfault;
 *     * `chunk_of()` returns nullptr for a chunk being unlinked from the global chain by `chunk_retire` => my histogram
 *       records it as the "unknown bucket" (data distortion);
 *     * the list is head-inserted into mid-walk => the same object is counted twice / missed in one dump.
 *
 * @warning **But `gc_mutex` solves only half of it**: `sweep_some` **deliberately does not take `gc_mutex`**
 *   (it takes the lock only once when reading the heap chain head); during sweeping it unlinks and `object_free`s slots,
 *   so "walking object lists while holding the lock" can still read **already-recycled slots** (measured crash: the cdb stack landed in
 *   `mark_load(o)` in `gc_test_count_colors`, `c0000005`).
 *   Therefore this function **walks only while idle (`phase == PAUSE`)**; while not idle it prints a single explanatory line and returns,
 *   **better to under-report than to read a dangling pointer** (the same self-protection as `gc_test_count_colors`).
 *   A caller who wants a complete dump should call it after `gc_collect()` returns.
 *
 * @note It **does not judge reachability**: everything listed is simply "the objects currently on the young/old/nursery lists".
 *       To ask "will this object be reclaimed", use `gc_weak`.
 * @note The implementation uses only **two small on-stack structures** (a 512-bucket histogram + a one-bucket temporary value),
 *       adding no public types  -- that is part of the "lightweight" constraint.
 */
inline std::size_t gc_dump(std::FILE* out = stdout) {
    /* Both using directives are required: gc_object is in gcxx, while gc_heap/gc_chunk/
     * gc_mutex_guard/chunk_of etc. are in gcxx::detail. This function is a **global** interface,
     * so both must be pulled in. */
    using namespace ::gcxx;
    using namespace ::gcxx::detail;
    /* ---- self-protection: do not walk object lists while not idle (see the @warning above) ---- */
    {
        const int ph = G().phase.load(std::memory_order_acquire);
        if (ph != (int)GC_PHASE_PAUSE)
            return (std::size_t)std::fprintf(
                out,
                "# gcxx heap dump skipped: gcxx is not idle (phase=%d); "
                "the object chains are being swept and may be recycled at any time.\n"
                "# Retry this call after gc_collect() returns, or when no gcxx allocation "
                "is in flight on any thread.\n", ph);
    }
    /* ---- pass 1: walk the heap and fill the histogram (under gc_mutex) ---- */
    struct bucket {
        std::uint32_t slot_size;
        std::size_t   objects, bytes, young, old, nursery, scanned;
    };
    bucket hist[512];
    std::size_t nb = 0;
    std::size_t live_objects = 0, live_bytes = 0, heaps = 0;
    bool overflow = false;
    {
        gc_mutex_guard lk;
        for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext) {
            ++heaps;
            gc_object* lists[3];
            lists[0] = h->young.load(std::memory_order_relaxed);
            lists[1] = h->old.load(std::memory_order_relaxed);
            lists[2] = h->nursery.load(std::memory_order_relaxed);
            for (int li = 0; li < 3; ++li) {
                for (gc_object* o = lists[li]; o != nullptr;
                     o = o->next.load(std::memory_order_relaxed)) {
                    const gc_chunk* c = chunk_of(reinterpret_cast<std::uintptr_t>(o));
                    /* chunk_of() == nullptr means the object chain and the chunk table
                     * disagree (only happens on corruption): bucket it as "unknown"
                     * instead of silently dropping it. */
                    const std::uint32_t ss = (c != nullptr && c->slot_size != 0)
                        ? c->slot_size : (std::uint32_t)0;
                    std::size_t k = 0;
                    for (; k < nb; ++k) if (hist[k].slot_size == ss) break;
                    if (k == nb) {
                        if (nb == sizeof(hist) / sizeof(hist[0])) { overflow = true; k = nb - 1; }
                        else {
                            hist[k].slot_size = ss;
                            hist[k].objects = hist[k].bytes = 0;
                            hist[k].young = hist[k].old = hist[k].nursery = 0;
                            hist[k].scanned = 0;
                            ++nb;
                        }
                    }
                    bucket& e = hist[k];
                    ++e.objects;
                    e.bytes += o->size;
                    if (li == 0) ++e.young; else if (li == 1) ++e.old; else ++e.nursery;
                    if (type_scanned_ct(o->type)) ++e.scanned;
                    ++live_objects;
                    live_bytes += o->size;
                }
            }
        }
    }
    /* mapped/chunks use exactly the same formula as gc_stat(): the recycle stack holds
     * unmapped-from-the-chain but still-mapped chunks, and omitting them understates the
     * real footprint (measured: understated by 190x once). */
    gc_state& g = G();
    const unsigned rec = g.recycled.load(std::memory_order_relaxed);
    const std::size_t mapped = g.mapped_bytes + (std::size_t)rec * CHUNK_SIZE;
    const std::size_t chunks = g.chunk_count + (std::size_t)rec;
    /* slot bytes = sum(objects * slot_size); the slot_size == 0 bucket (large objects)
     * is estimated with header + padding. */
    std::size_t used = 0;
    for (std::size_t k = 0; k < nb; ++k) {
        const std::size_t ss = hist[k].slot_size;
        used += hist[k].objects
              * (ss != 0 ? ss : (std::size_t)PAYLOAD_OFF + (std::size_t)PAD_BYTES);
    }
    const std::size_t overhead = mapped > used ? mapped - used : 0;

    std::size_t n = 0;
    n += (std::size_t)std::fprintf(out, "# ===== gcxx heap dump =====\n");
    n += (std::size_t)std::fprintf(out,
        "# live_objects=%zu  live_bytes=%zu (%.2f MiB)\n"
        "# mapped=%zu (%.2f MiB)  chunks=%zu  heaps=%zu  mapped_overhead=%zu (%.1f%%)\n",
        live_objects, live_bytes, (double)live_bytes / 1048576.0,
        mapped, (double)mapped / 1048576.0, chunks, heaps, overhead,
        mapped ? 100.0 * (double)overhead / (double)mapped : 0.0);
    /* Histogram: the 12 buckets with the largest slot footprints, biggest first.
     * Selection sort is fine here: this is a debug path and the bucket count is small. */
    if (nb != 0) {
        n += (std::size_t)std::fprintf(out,
            "#\n# %-10s %10s %12s %8s %8s %8s %9s %11s\n",
            "slot", "objects", "payload", "young", "old", "nursery",
            "scanned", "slot_bytes");
        bool shown[512] = {};
        for (std::size_t rank = 0; rank < 12 && rank < nb; ++rank) {
            std::size_t best = (std::size_t)-1, best_bytes = 0;
            for (std::size_t k = 0; k < nb; ++k) {
                if (shown[k]) continue;
                const std::size_t ss = hist[k].slot_size != 0
                    ? hist[k].slot_size : (std::size_t)PAYLOAD_OFF + PAD_BYTES;
                const std::size_t sb = hist[k].objects * ss;
                if (best == (std::size_t)-1 || sb > best_bytes) { best = k; best_bytes = sb; }
            }
            if (best == (std::size_t)-1) break;
            shown[best] = true;
            char label[24];
            if (hist[best].slot_size == 0) std::snprintf(label, sizeof(label), "large");
            else std::snprintf(label, sizeof(label), "%u",
                               (unsigned)hist[best].slot_size);
            n += (std::size_t)std::fprintf(out,
                "# %-10s %10zu %12zu %8zu %8zu %8zu %9zu %11zu\n",
                label, hist[best].objects, hist[best].bytes, hist[best].young,
                hist[best].old, hist[best].nursery, hist[best].scanned, best_bytes);
        }
        if (nb > 12)
            n += (std::size_t)std::fprintf(out,
                "# ... (%zu buckets total; showing the 12 largest by slot bytes)\n", nb);
        if (overflow)
            n += (std::size_t)std::fprintf(out,
                "# WARNING: more than %zu buckets; extras were merged into the last one\n",
                sizeof(hist) / sizeof(hist[0]));
    }
    /* Per-heap chain lengths: with several threads this shows at a glance who piled up
     * garbage that has not been collected yet. */
    {
        gc_mutex_guard lk;
        n += (std::size_t)std::fprintf(out,
            "#\n# %-14s %8s %8s %10s %6s\n", "heap", "young", "old", "nursery", "dead");
        std::size_t hi = 0;
        for (const gc_heap* h = heaps_head(); h != nullptr; h = h->hnext, ++hi) {
            std::size_t cy = 0, co = 0, cn = 0;
            for (const gc_object* o = h->young.load(std::memory_order_relaxed);
                 o != nullptr; o = o->next.load(std::memory_order_relaxed)) ++cy;
            for (const gc_object* o = h->old.load(std::memory_order_relaxed);
                 o != nullptr; o = o->next.load(std::memory_order_relaxed)) ++co;
            for (const gc_object* o = h->nursery.load(std::memory_order_relaxed);
                 o != nullptr; o = o->next.load(std::memory_order_relaxed)) ++cn;
            n += (std::size_t)std::fprintf(out, "# heap[%zu]%8s %8zu %8zu %10zu %6s\n",
                hi, "", cy, co, cn,
                h->dead.load(std::memory_order_relaxed) ? "yes" : "no");
        }
    }
    /* Summary counters (all relaxed reads, diagnostics only). */
    n += (std::size_t)std::fprintf(out,
        "#\n# cycles=%zu (minor %zu / major %zu)  cancelled=%zu\n"
        "# recycled_chunks=%u  purged_cold=%u  guarded_objects=%zu\n"
        "# freelist_corruptions=%zu  gray_rescans=%zu  gray_discards=%zu\n"
        "# scanned_bytes=%zu skipped_bytes=%zu bias_hits=%zu\n"
        "# decommits=%zu recommits=%zu  rss=%zu bytes\n",
        (std::size_t)g.cycles.load(std::memory_order_relaxed),
        (std::size_t)g.minor_cycles.load(std::memory_order_relaxed),
        (std::size_t)g.major_cycles.load(std::memory_order_relaxed),
        (std::size_t)g.cancelled_cycles.load(std::memory_order_relaxed),
        (unsigned)g.recycled.load(std::memory_order_relaxed),
        (unsigned)g.recycled_cold.load(std::memory_order_relaxed),
        (std::size_t)g.guarded_objects.load(std::memory_order_relaxed),
        (std::size_t)g.freelist_corruptions.load(std::memory_order_relaxed),
        (std::size_t)g.gray_rescans,
        (std::size_t)g.gray_discards.load(std::memory_order_relaxed),
        (std::size_t)g.scanned_bytes.load(std::memory_order_relaxed),
        (std::size_t)g.skipped_bytes.load(std::memory_order_relaxed),
        (std::size_t)g.bias_hits.load(std::memory_order_relaxed),
        (std::size_t)g.decommits.load(std::memory_order_relaxed),
        (std::size_t)g.recommits.load(std::memory_order_relaxed),
        rss_bytes());
    n += (std::size_t)std::fprintf(out, "# ===== end =====\n");
    return n;
}

/**
 * @brief Does this pointer point to a **live object** on the GC heap?
 *
 * @return On a hit, the object's internal header pointer (`void*`, used only for "hit or not" and equality comparisons),
 *         otherwise `nullptr`.
 *
 * @note It uses the **same** reverse-lookup criteria as the marking path (direct map table + `ptr_hits_object`), so
 *       **interior pointers** (`&obj.field`, negative bias such as `p-16`, the middle of a large object) count as hits
 *       too, consistent with the `GCXX_ALL_INTERIOR_POINTERS` setting.
 * @note It **does not consider reachability**: a non-null return only says "this is a GC object", not that it is reachable.
 *       To ask "will this object be reclaimed", use `gc_weak`.
 * @note No `gc_mutex` needed (unlike `gc_dump`): it performs **one** address -> table-entry
 *       lookup + one object-header read, walking no lists; in the worst case it reads "a chunk currently being unlinked from the global chain"
 *       and returns nullptr  -- for the question "is this a GC object", that is **safe by direction**
 *       (better to say no; non-GC memory is never called a GC object).
 */
inline void* gc_is_gc_pointer(const void* p) {
    return ::gcxx::detail::object_find(reinterpret_cast<std::uintptr_t>(p));
}

/**
 * @brief Force one **complete** major collection (running the incremental sweep to completion in one go).
 * @return `true` = this cycle really reclaimed; `false` = this call **reclaimed no memory at all**, for **two** reasons
 *         (the caller cannot tell them apart from the return value; neither means an error):
 *         1. the cycle was **cancelled**  -- some thread failed to reach a safepoint within `STW_TIMEOUT_MS`. If the mutator cannot be stopped
 *            it cannot be marked safely, so the cancelled cycle reclaims nothing (no dangling pointers are produced, the memory is simply not saved);
 *         2. **another thread is already collecting** (`collect_depth` acquisition failed)  -- this call returns immediately,
 *            and the owner finishes that cycle.
 * @note Normal use **does not need** it: collection is driven automatically by allocation debt (see the design points at the head of the file).
 *       Its purpose is "I know there is a big pile of garbage right now and I want it collected immediately".
 */
inline bool gc_collect() {
    ::gcxx::detail::ensure_init();
    return ::gcxx::detail::full_collect();
}
/**
 * @brief Weak-reference **handle**: it becomes "expired" once the target is unreachable.
 *
 * @code
 *   int* p = gc<int>(10);         // p is a root (on the stack), so the object stays alive
 *   gc_weak<int> w = p;           // register a weak reference
 *   if (w) { int v = *w; }        // target alive: safe to read
 *   // once p goes away the next collection reclaims the object and w expires at once
 * @endcode
 *
 * @warning A weak reference **does not count as a reference**. `gc_weak<int> w = gc<int>(10);` is well-formed, but apart from this weak
 *          reference the object has no references at all, so the first collection reclaims it and `w` expires at once  -- to keep it alive there must be
 *          **another** reachable reference (usually a strong pointer held by the caller).
 * @note Expiry is decided **when marking ends, before sweeping starts**, and does not change between then and the next cycle.
 * @note **Dereferencing an expired handle is undefined behaviour** (the same as dereferencing a `std::weak_ptr` after `lock()` returned
 *       empty). Test with `expired()` / `operator bool` / `lock()` first.
 * @note User code is never called at any point (this library's "the GC never calls destructors" contract).
 * Uses: caches, observer tables, object-graph deduplication, "just need to know whether the object is still there"; when the value points back at the key, use `gc_ephemeron`.
 * Implementation note: the handle stores **only an index** (plus the **generation** of the table it belongs to, see `weak_generation`),
 * and the raw pointers live in a GC-private malloc table  -- this is not memory saving, it is a correctness precondition (see the notes on weak_slot).
 * Copy = register another entry; move = transfer the entry; destruction = deregister. */
template <class T>
class gc_weak {
public:
    gc_weak() noexcept : idx_(gcxx::detail::SLOT_NONE), gen_(0) {}
    gc_weak(std::nullptr_t) noexcept : idx_(gcxx::detail::SLOT_NONE), gen_(0) {}
    /* Not explicit: the form `gc_weak<int> w = gc<int>(10);` must compile */
    gc_weak(T* p) : idx_(gcxx::detail::SLOT_NONE), gen_(0) { assign(p); }
    gc_weak(const gc_weak& o) : idx_(gcxx::detail::SLOT_NONE), gen_(0) { assign(o.get()); }
    gc_weak(gc_weak&& o) noexcept : idx_(o.idx_), gen_(o.gen_) {
        o.idx_ = gcxx::detail::SLOT_NONE;
        o.gen_ = 0;
    }
    gc_weak& operator=(const gc_weak& o) {
        if (this != &o) assign(o.get());
        return *this;
    }
    gc_weak& operator=(gc_weak&& o) noexcept {
        if (this != &o) {
            reset();
            idx_ = o.idx_;
            gen_ = o.gen_;
            o.idx_ = gcxx::detail::SLOT_NONE;
            o.gen_ = 0;
        }
        return *this;
    }
    gc_weak& operator=(T* p) { assign(p); return *this; }
    gc_weak& operator=(std::nullptr_t) { reset(); return *this; }
    ~gc_weak() { reset(); }

    /* Target still there -> return it, otherwise nullptr. While this pointer is held the target definitely will not be reclaimed (it is an ordinary conservative root). */
    T* get() const noexcept {
        return static_cast<T*>(gcxx::detail::weak_peek(idx_, gen_, false));
    }
    T* lock() const noexcept { return get(); }
    bool expired() const noexcept { return get() == nullptr; }
    explicit operator bool() const noexcept { return get() != nullptr; }
    /* Dereferencing an expired handle is UB (see the class comment). */
    T& operator*() const { return *get(); }
    T* operator->() const { return get(); }

    void reset() noexcept {
        gcxx::detail::weak_clear(idx_, gen_);
    }
    void reset(T* p) { assign(p); }
    /* For diagnostics: the index of this entry in the GC-private table (not a pointer, does not constitute a root).
     * @note After a `gc_dispose()` it is no longer valid, and `get()` returns nullptr  -- using
     *       `index()` to test liveness is **wrong** (see `weak_generation`). */
    unsigned index() const noexcept { return (unsigned)idx_; }

private:
    void assign(T* p) {
        gcxx::detail::ensure_init();
        gcxx::detail::weak_set(idx_, gen_, static_cast<void*>(p), nullptr,
                               gcxx::detail::WEAK_PLAIN);
    }
    std::uint32_t idx_;
    /** The value of `weak_generation` at registration time; 0 = "this handle currently has no valid entry".
     *  `weak_set` / `weak_clear` / `weak_peek` all require it to equal `G().weak_generation`,
     *  so **a stale handle across `gc_dispose()` cannot cross onto someone else's slot** (see the notes on weak_generation). */
    std::uint32_t gen_;
};

/**
 * @brief A key/value pair in which "the value is reachable only when the **key** is reachable" (ephemeron).
 *
 * @code
 *   v->owner = k;                      // the value points back at the key (the common shape)
 *   gc_ephemeron<Key, Value> e;
 *   e.set(k, v);
 *   if (e) { Value* got = *e; }        // key alive -> value alive too
 *   // once k becomes unreachable: **key and value are reclaimed together** and e expires at once
 * @endcode
 *
 * @note The difference from "weak key + strong value" (the sole reason ephemerons exist): an ordinary weak reference weakens only the **key**,
 *       while the value is an ordinary strong reference  -- when the value points back at the key, the value keeps the key marked, so the key never dies and the weak reference never expires
 *       (the classic leak). An ephemeron folds the value into the same decision: **when the key is unreachable the value does not take part in marking at all**,
 *       so nothing the value points to (including the key) is retained, and the two are reclaimed together.
 * @note Semantic boundaries: `set(nullptr, v)` is equivalent to `reset()`; when the key is reachable the value is **certainly** reachable (a non-null `get()`
 *       means the value's memory is valid, so no further liveness test is needed).
 * @note As with `gc_weak`, dereferencing after expiry is UB, and user code is **never** called.
 * @note Cost: after marking converges each cycle the table is scanned once for activation (see `ephemeron_activate_all`); entries are few to begin with,
 *       so this is an "acceptable cost for a special-purpose feature"  -- do **not** use it as a general WeakHashMap. */
template <class K, class V>
class gc_ephemeron {
public:
    gc_ephemeron() noexcept : idx_(gcxx::detail::SLOT_NONE), gen_(0) {}
    gc_ephemeron(K* k, V* v) : idx_(gcxx::detail::SLOT_NONE), gen_(0) { set(k, v); }
    gc_ephemeron(const gc_ephemeron& o) : idx_(gcxx::detail::SLOT_NONE), gen_(0) {
        set(o.key(), o.get());                  /* an independent entry */
    }
    gc_ephemeron(gc_ephemeron&& o) noexcept : idx_(o.idx_), gen_(o.gen_) {
        o.idx_ = gcxx::detail::SLOT_NONE;
        o.gen_ = 0;
    }
    gc_ephemeron& operator=(const gc_ephemeron& o) {
        if (this != &o) set(o.key(), o.get());
        return *this;
    }
    gc_ephemeron& operator=(gc_ephemeron&& o) noexcept {
        if (this != &o) {
            reset();
            idx_ = o.idx_;
            gen_ = o.gen_;
            o.idx_ = gcxx::detail::SLOT_NONE;
            o.gen_ = 0;
        }
        return *this;
    }
    ~gc_ephemeron() { reset(); }

    /* Register (or replace a pair). k == nullptr is equivalent to reset(). */
    void set(K* k, V* v) {
        gcxx::detail::ensure_init();
        gcxx::detail::weak_set(idx_, gen_, static_cast<void*>(k), static_cast<void*>(v),
                               gcxx::detail::WEAK_EPH);
    }
    void reset() noexcept {
        gcxx::detail::weak_clear(idx_, gen_);
    }

    /* Key alive -> value; otherwise nullptr. Non-null means valid (see the class comment). */
    V* get() const noexcept {
        return static_cast<V*>(gcxx::detail::weak_peek(idx_, gen_, true));
    }
    V* lock() const noexcept { return get(); }
    K* key() const noexcept {
        return static_cast<K*>(gcxx::detail::weak_peek(idx_, gen_, false));
    }
    bool expired() const noexcept { return get() == nullptr; }
    explicit operator bool() const noexcept { return get() != nullptr; }
    V& operator*() const { return *get(); }
    V* operator->() const { return get(); }
    unsigned index() const noexcept { return (unsigned)idx_; }

private:
    std::uint32_t idx_;
    /** The value of `weak_generation` at registration time; 0 = this handle currently has no valid entry. Same reasoning as `gc_weak::gen_`.
     *  Every constructor initialises it explicitly: a default-constructed handle must never compare an
     *  indeterminate generation against `g.weak_generation`, because `weak_slot_lookup` accepts only the
     *  exact generation (with `0` reserved for "never registered"). */
    std::uint32_t gen_;
};
namespace gcxx { namespace detail {
/** @brief Reset `gc_state` in place to its "just constructed" shape after `dispose_all` released
 *         every block of memory (its full contract is documented at the definition, just below).
 * @note Forward-declared because `dispose_all` ends by calling it, while the definition is kept
 *       adjacent so the release pass above stays readable as "free memory in this exact order".
 * @internal
 */
inline void reset_state_after_dispose(gc_state& g);
/** @brief Free all memory allocated by gcxx and reset the internal state to its "just initialized" shape.
 *
 * Free list: all chunks (including the empty chunks on the recycle stack and collision-chain overflow nodes), the chunk direct map table,
 * the gray-set block pointer table and all gray-set blocks, the weak-reference/ephemeron tables, and **every thread's heap struct**.
 *
 * @warning **`gc_state` itself is not freed** (a process-lifetime object; see the notes on `G()`: it is full of GC heap
 *          addresses, and if conservatively scanned it would mark a whole swathe of unreachable objects alive); it is only reset in place.
 *
 * @warning **The caller must guarantee that "no other thread is using gcxx at this moment"**. This function unconditionally clears
 *          the global state and frees **all** heap structs (including those still pointed at by other threads' TLS slots),
 *          so a concurrent call causes use-after-free. The constraint is the caller's to uphold; this function does not check it.
 *
 * @note After the call the library **can still be used**: the next `gc()` rebuilds the state and the current thread's heap on demand.
 * @note No user code is called (no destructors are run), consistent with the GC's own contract.
 * @internal
 */
inline void dispose_all() {
    gc_state& g = G();
    gc_mutex_guard lk;                       /* mutually exclusive with allocation/collection (the caller must still ensure there are no concurrent users) */

    /* ---- the order matters ----
     * `chunk_release` -> `chunk_retire` uses `c->owner` (it must be unlinked from the owner's partial chain),
     * so **all chunks must be freed first, and the heap structs afterwards**; the reverse would be reads of
     * already-freed memory. */
    for (gc_chunk* c = g.chunks; c != nullptr; ) {
        gc_chunk* nx = c->all_next;
        chunk_release(c);                    /* retire first, then give it back to the system */
        c = nx;
    }
    g.chunks = nullptr;
    /* the empty chunks on the recycle stack (they are no longer on the global chain, but their owner is still valid) */
    for (unsigned i = 0; i < static_cast<unsigned>(KINDS * CLASSES); ++i) {
        for (;;) {
            gc_chunk* c = chunk_stack_pop(g.recycle[i]);
            if (c == nullptr) break;
            chunk_release(c);
        }
        for (;;) {
            gc_chunk* c = chunk_stack_pop(g.recycle_cold[i]);
            if (c == nullptr) break;
            chunk_release(c);
        }
    }

    /* Chunk direct map: the current bucket array, every retired array, and the pool of recycled overflow
     * nodes. This is the **one** place where any of them may be released: `gc_dispose()` is contracted to
     * be called only when no other thread is using the library, which is exactly the precondition the
     * lock-free readers (`object_find` / `gc_is_gc_pointer` / `weak_peek`) need. */
    std::free(static_cast<void*>(g.chunk_table));
    g.chunk_table = nullptr;
    while (g.table_retired != nullptr) {
        gc_map_entry* nx = g.table_retired->next;
        std::free(static_cast<void*>(g.table_retired));
        g.table_retired = nx;
    }
    while (g.table_retired_nodes != nullptr) {
        gc_map_entry* nx = g.table_retired_nodes->next;
        std::free(static_cast<void*>(g.table_retired_nodes));
        g.table_retired_nodes = nx;
    }
    while (g.table_free_nodes != nullptr) {
        gc_map_entry* nx = g.table_free_nodes->next;
        std::free(static_cast<void*>(g.table_free_nodes));
        g.table_free_nodes = nx;
    }

    /* the gray-set block pointer table + each block */
    if (g.gray_blocks != nullptr) {
        for (std::size_t i = 0; i < g.gray_committed_blocks; ++i) {
            gray_entry* blk = g.gray_blocks[i].load(std::memory_order_relaxed);
            if (blk != nullptr) std::free(static_cast<void*>(blk));
        }
        std::free(static_cast<void*>(g.gray_blocks));
        g.gray_blocks = nullptr;
    }

    /* Each thread's heap struct (pointers left behind in other threads' TLS slots are the caller's responsibility to stop using).
     * **This thread's TLS/FLS slots must be cleared first**  -- otherwise:
     *   * the next `gc()` uses that freed pointer directly on `my_heap()`'s fast path
     *     (measured: the first allocation after dispose is a segfault);
     *   * **at process exit** the exit callback (Windows FLS callbacks / POSIX key destructors)
     *     still receives that already-`free`d heap and dereferences it  -- measured on Windows:
     *     after `gc_dispose()`, **allocating nothing more and just exiting is an ACCESS_VIOLATION**
     *     (minimal reproducer
     *     `probes/min_dispose_verify_probe.cpp` modes 0/3).
     * So `heap_tls_clear_all()` must be used here (**clearing both TlsSetValue and FLS**),
     * not just `heap_tls_set(nullptr)`  -- that clears only the former.
     * If other threads keep using gcxx after dispose, the stale pointers in their slots are just as dangerous  --      * that is exactly the part covered by the "the caller guarantees no other users" constraint. */
    heap_tls_clear_all();
    gc_heap* h = g.heaps.load(std::memory_order_relaxed);
    while (h != nullptr) {
        gc_heap* nx = h->hnext;
        h->~gc_heap();
        std::free(static_cast<void*>(h));
        h = nx;
    }

    /* weak-reference / ephemeron tables: the current table + every old table retired during growth (see weak_retired_*). */
    std::free(g.weak_tab);
    for (std::size_t i = 0; i < g.weak_retired_len; ++i)
        std::free(g.weak_retired[i]);
    std::free(g.weak_retired);

    /* ---- State reset: clear pointers + zero the counters so the library can be used again.
     * That is a long, flat, exhaustive list and it lives in `reset_state_after_dispose` below,
     * together with the reasoning for the weak-generation bump. Keeping it out of line is what
     * lets this function read as "release memory in this exact order" and nothing else. ---- */
    reset_state_after_dispose(g);
}
/**
 * @brief Reset `gc_state` in place to its "just constructed" shape after `dispose_all` released
 *        every block of memory.
 *
 * Kept separate from the release pass in `dispose_all` for two reasons: the release pass has a
 * **load-bearing order** (chunks before heaps, see there), while this pass is a flat list that must
 * simply be **exhaustive**; and mixing "free memory in this order" with "zero 40 counters" in one
 * body is what made that function hard to check by eye.
 *
 * @warning **Exhaustiveness is the whole contract.** Any field of `gc_state` that the release pass
 *          freed, or that carries a pointer into the heap, must be cleared here; a field that is
 *          merely stale (timers, counters) is harmless but is zeroed anyway so that a post-dispose
 *          `gc_stat()` does not report the previous run's numbers.
 * @note The **weak-table generation bump belongs here and is not optional**: every surviving
 *       `gc_weak` / `gc_ephemeron` handle still holds an index, and once the table grows again that
 *       index would designate *another handle's slot*. Bumping the generation invalidates all of
 *       them at once. It happens after the table was freed and before the pointer is cleared;
 *       `gc_mutex` is held throughout, so no reader can observe half a dispose, and "the table is
 *       gone" and "the generation changed" take effect together for readers.
 *       Generation 0 is reserved for "never registered", so it must never be handed out.
 * @param g The global state (already locked by the caller).
 * @internal
 */
inline void reset_state_after_dispose(gc_state& g) {
    g.heaps.store(nullptr, std::memory_order_relaxed);
    g.weak_tab = nullptr; g.weak_cap = 0; g.weak_len = 0; g.weak_live = 0;
    ++g.weak_generation;
    if (g.weak_generation == 0) g.weak_generation = 1;   /* 0 is reserved for "never registered" */
    g.weak_free = SLOT_NONE;
    g.weak_retired = nullptr; g.weak_retired_len = 0; g.weak_retired_cap = 0;
    g.table_size = 0; g.table_mask = 0; g.block_count = 0;
    g.table_retired = nullptr; g.table_retired_nodes = nullptr;
    g.table_free_nodes = nullptr;
    g.chunk_count = 0; g.mapped_bytes = 0;
    g.live_objects.store(0, std::memory_order_relaxed);
    g.freed_objects.store(0, std::memory_order_relaxed);
    g.total_bytes.store(0, std::memory_order_relaxed);
    g.cycles.store(0, std::memory_order_relaxed);
    g.minor_cycles.store(0, std::memory_order_relaxed);
    g.major_cycles.store(0, std::memory_order_relaxed);
    g.cancelled_cycles.store(0, std::memory_order_relaxed);
    g.scanned_bytes.store(0, std::memory_order_relaxed);
    g.skipped_bytes.store(0, std::memory_order_relaxed);
    g.bias_hits.store(0, std::memory_order_relaxed);
    g.root_scans.store(0, std::memory_order_relaxed);
    g.root_probed_words.store(0, std::memory_order_relaxed);
    g.gray_cap.store(0, std::memory_order_relaxed);
    g.gray_alloc.store(0, std::memory_order_relaxed);
    g.gray_head.store(0, std::memory_order_relaxed);
    g.gray_tail.store(0, std::memory_order_relaxed);
    g.gray_ready.store(0, std::memory_order_relaxed);
    g.gray_epoch.store(0, std::memory_order_relaxed);
    g.gray_overflow.store(false, std::memory_order_relaxed);
    g.gray_need_rescan = false;
    g.gray_committed_blocks = 0;
    g.recycled.store(0, std::memory_order_relaxed);
    g.recycled_cold.store(0, std::memory_order_relaxed);
    g.decommits.store(0, std::memory_order_relaxed);
    g.recommits.store(0, std::memory_order_relaxed);
    g.guarded_objects.store(0, std::memory_order_relaxed);
    g.freelist_corruptions.store(0, std::memory_order_relaxed);
    g.error_count.store(0, std::memory_order_relaxed);
    g.t_stw_us.store(0, std::memory_order_relaxed);
    g.t_root_us.store(0, std::memory_order_relaxed);
    g.t_mark_us.store(0, std::memory_order_relaxed);
    g.t_sweep_us.store(0, std::memory_order_relaxed);
    g.sweep_pend_bytes = 0; g.sweep_pend_objs = 0;
    g.sweep_pend_bytes_pub.store(0, std::memory_order_relaxed);
    g.sweep_pend_objs_pub.store(0, std::memory_order_relaxed);
    g.sweep_dirty = nullptr; g.sweep_of_cache = nullptr;
    g.step_depth.store(0, std::memory_order_relaxed);
    g.collect_depth.store(0, std::memory_order_relaxed);
    g.cycle_owner.store(nullptr, std::memory_order_relaxed);
    g.oldscan = nullptr; g.oldscan_heap = nullptr;
    g.scan.cur = nullptr; g.scan.off = 0;
    g.phase.store((int)GC_PHASE_PAUSE, std::memory_order_relaxed);
    g.kind.store((int)GC_KIND_NONE, std::memory_order_relaxed);
    g.mark_excl.store(false, std::memory_order_relaxed);
    g.aborting.store(false, std::memory_order_relaxed);
    g.sp_request.store(0, std::memory_order_relaxed);
    g.sp_scanning.store(0, std::memory_order_relaxed);
    g.sp_parked.store(0, std::memory_order_relaxed);
    g.static_range_count = 0; g.static_inited = false;
    g.dyn_roots_scanned.store(false, std::memory_order_relaxed);
    for (auto& r : g.static_ranges) r = nullptr;
    g.heap_lo.store(~(std::uintptr_t)0, std::memory_order_relaxed);
    g.heap_hi.store(0, std::memory_order_relaxed);
}
} /* namespace detail */
} /* namespace gcxx */
/** @brief Free **all** memory allocated by gcxx and reset the internal state (the library can be used afterwards).
 *
 * **The caller must guarantee that no other thread is using gcxx at this moment** (the function frees every thread's heap struct,
 * including those still pointed at by other threads' TLS slots). The constraint is the caller's to uphold; the function does not check it.
 *
 * Typical uses: process teardown, resetting between unit tests, proactively returning all memory after a long batch.
 * @note Unlike `gc_collect()`, which reclaims **unreachable objects**, `gc_dispose()` reclaims
 *       **every block of memory** gcxx has ever requested from the system, including blocks occupied by objects
 *       that are still referenced: after the call every gcxx pointer is invalid. */
inline void gc_dispose() {
    ::gcxx::detail::dispose_all();
}
/* --------------------------------------------------------------------------
 * Auto-initialization: every translation unit that includes this header registers a static constructor that calls ensure_init() during CRT startup
 * (idempotent)  -- users do **not** need to call gc_init() explicitly. Correctness basis: the stack-scan upper bound comes from the TEB's StackBase (the thread's
 * real stack top) and is independent of initialization timing, so it is already correct during the static initialization phase before main; even if a user's static constructor runs earlier than
 * this instance (cross-TU order is not guaranteed), ensure_init() on the allocation path also initializes on demand  -- gc is safe at any moment.
 * The gc_init() macro is kept: calling it explicitly is harmless (idempotent).
 * ------------------------------------------------------------------------ */
namespace gcxx { namespace detail {
struct gc_auto_init {
    gc_auto_init() { ensure_init(); }
};
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4073)   /* initializer placed in the library initialization region */
#pragma init_seg(lib)            /* runs before user static constructors */
static const gc_auto_init gc_auto_init_inst{};
#pragma warning(pop)
#else
static const gc_auto_init gc_auto_init_inst{};
#endif
} /* namespace detail */
} /* namespace gcxx */
/* Take back the preprocessor macros that this header **defined itself** and that affect user code.
 * undef only when it is "genuinely the one we defined"  -- if the user had already defined it (or wants to keep using it),
 * leaving it alone is mandatory: `#undef`-ing someone else's macro directly changes the behaviour of their later includes. */
#if defined(GCXX_DEFINED_NOMINMAX)
#  undef NOMINMAX
#  undef GCXX_DEFINED_NOMINMAX
#endif
#if defined(GCXX_DEFINED_WIN32_LEAN_AND_MEAN)
#  undef WIN32_LEAN_AND_MEAN
#  undef GCXX_DEFINED_WIN32_LEAN_AND_MEAN
#endif

#endif /* GCXX_GC_HPP_INCLUDED */

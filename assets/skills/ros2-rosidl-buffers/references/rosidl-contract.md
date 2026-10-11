# ROSIDL storage and transport contract

## Evidence baseline

Checked 2026-10-10 against upstream `ros2/rosidl` revision
`4537874f320ef8b350c3df7f5b6b487b409fa22f`. Recheck the installed revision before
generating code; these are source-level observations, not a certification of
every ROS distribution, client language, RMW, or platform binary.

- [Buffer container](https://github.com/ros2/rosidl/blob/4537874f320ef8b350c3df7f5b6b487b409fa22f/rosidl_buffer/include/rosidl_buffer/buffer.hpp)
- [BufferImplBase](https://github.com/ros2/rosidl/blob/4537874f320ef8b350c3df7f5b6b487b409fa22f/rosidl_buffer/include/rosidl_buffer/buffer_impl_base.hpp)
- [BufferBackend](https://github.com/ros2/rosidl/blob/4537874f320ef8b350c3df7f5b6b487b409fa22f/rosidl_buffer_backend/include/rosidl_buffer_backend/buffer_backend.hpp)
- [Backend registry and tests](https://github.com/ros2/rosidl/tree/rolling/rosidl_buffer_backend_registry)
- [Backend implementations](https://github.com/ros2/rosidl_buffer_backends)

## Separate the layers

1. **Message:** Keep the portable `.msg` contract. Inspect the generated field
   type: eligible primitive sequences such as image `uint8[] data` can use
   `rosidl::Buffer<uint8_t>`. Do not assume every sequence, fixed array, language
   binding, or older generated message uses it. A stable message definition
   does not imply a stable C++ ABI across rebuilt/old ROS packages.
2. **Storage:** `BufferImplBase<T>` provides `get_backend_type()`, `size()`,
   `to_cpu()`, and `clone()`. `size()` counts elements, not necessarily bytes.
   Backend-specific access, mapping, and synchronization live outside this
   minimal interface. It is not a universal GPU-pointer API.
3. **Transport:** `BufferBackend` supplies descriptor typesupport, construction,
   import, and endpoint hooks. A storage wrapper alone does not implement
   inter-process sharing or cause the RMW to use descriptors.
4. **Conversion:** Prefer supported image/tensor/point-cloud conversion APIs when
   available. Verify aliasing versus copying and lifetime contracts. A conversion
   library can allocate/copy even when transport shares the original buffer.

## Container behavior at the inspected revision

- Default construction uses `CpuBufferImpl<T, Allocator>` and vector storage.
  The implementation constructor takes `std::unique_ptr<BufferImplBase<T>>`;
  ownership is established at construction, not through a public `set_impl`.
- Copy construction/assignment invokes `clone()`, whose contract is a deep copy.
  Moving transfers the implementation. Trace the selected `publish` overload,
  intra-process settings, message copies, and callback signatures; `std::move`
  alone is not proof that the complete pipeline avoids copies.
- `.data()`, iterators, indexing, resizing, and vector conversions require the
  recognized CPU implementation and throw for other implementations. A custom
  shared-memory buffer physically residing in RAM is not automatically recognized
  as `CpuBufferImpl`; use its explicit native access API and conversion path.
- `to_vector()` creates CPU data through `to_cpu()` for non-CPU storage; it is an
  explicit copy boundary. Equality comparisons can also materialize CPU copies.
  Do not use these operations in a benchmark without accounting for their cost.
- `to_cpu()` must return the CPU representation expected by the installed
  container/typesupport. Preserve element type, count, completed producer data,
  and allocator/ABI compatibility. Never reinterpret a DXGI handle as a vector.

## Backend implementation checklist

Use actual installed headers and existing plugin tests, not invented signatures.

1. Implement the storage interface with deterministic ownership, deep-copy
   semantics, and correct CPU conversion. Separate the original capture buffer's
   lease from the allocation's existence; both matter during asynchronous use.
2. Implement `BufferBackend`, including `get_descriptor_type_support()`,
   `create_empty_descriptor()`, `create_descriptor_with_endpoint()`, and
   `from_descriptor_with_endpoint()`. The typesupport handle is the generic
   aggregate handle; the consuming RMW resolves its concrete typesupport.
3. Use `on_creating_endpoint()` and `on_discovering_endpoint()` as required by
   the verified implementation. Check peer backend/version, locality, identity,
   device/adapter, and resource compatibility, not only the topic name.
4. At this revision, `create_descriptor_with_endpoint()` can return `nullptr`
   to request CPU serialization. Observe `kMaxBufferDescriptorSize` (4096 bytes
   at the inspected revision). A descriptor describes access to storage; it must
   not embed a large frame or arbitrary process addresses.
5. Validate descriptor version, generation, sizes, offsets, alignment, format,
   access rights, and allocation bounds before mapping/importing. Avoid integer
   overflow and unbounded allocation. Import failures and forged/stale resources
   must fail safely, not bypass authentication or reuse someone else's storage.
6. Bind any native IPC side channel to authorized ROS peers. DDS security does
   not automatically secure a named mapping, Unix socket, or shared GPU handle.
   Define minimal OS permissions and producer/consumer-death cleanup.
7. Register/export the plugin and descriptor typesupport using the installed
   registry mechanism. Check package dependencies, shared-library visibility,
   Windows DLL loading/MSVC runtime, Linux shared libraries, and install rules.
   Keep the plugin loaded while any implementation or deleter can still run.
8. Verify the specific RMW/typesupport can advertise, serialize, and reconstruct
   descriptors and fall back for incompatible connections. Do not publish a
   universal middleware support table from package names alone. If hooks are
   missing, stop claiming backend support and scope the RMW work separately.

## Platform availability boundary

The inspected `ros2/rosidl_buffer_backends` main revision
`d7cd9642d77a1d64fd85f25ba0bf96e108401900` contains CUDA and Qualcomm implementations,
not evidence of a production Media Foundation/D3D11 or general-purpose Windows
shared-memory backend. Recheck current upstream and the workspace before deciding
to implement one. POSIX mappings and Win32 mappings are native primitives, not
drop-in replacements for those plugins.

Keep ordinary CPU publication working when a backend is unavailable. On older
ROS builds this may remain vector-backed publication without any ROSIDL buffer
plugin. Never mix ABI-incompatible core packages to make only one node compile.
Test fallback and direct intra-process CPU consumers explicitly; preserving the
message definition alone does not ensure all existing consumers handle device
storage. Python and .NET support must be checked independently of C++ support.

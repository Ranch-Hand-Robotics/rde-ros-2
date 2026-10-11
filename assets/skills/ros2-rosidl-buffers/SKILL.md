---
name: ros2-rosidl-buffers
description: "Use when designing, implementing, or debugging ROS 2 rosidl::Buffer storage and buffer backend plugins, zero-copy camera/image/point-cloud pipelines, Linux and Windows managed buffers, shared-memory IPC, V4L2 DMA-BUF, or native Windows Media Foundation and D3D11/DXGI capture. Covers ownership, pooling, synchronization, CPU fallback, and measured copy boundaries."
user-invocable: true
---

# ROS 2 ROSIDL buffers

Read the [development ground rules](../ros2-development/SKILL.md) before work;
stop execution if unavailable. Keep standard ROS message semantics separate from
storage, capture, and transport. Here **managed buffers** means explicitly owned,
pooled, lifetime-managed native memory, not .NET garbage-collected arrays or
CUDA Unified Memory. If a managed-language binding is requested, verify its
actual buffer support and interop lifetime separately.

## Establish the supported path

1. Record ROS distro and exact package revisions, generated message headers,
   client library, RMW/typesupport, compiler/ABI, OS/shell/architecture, camera,
   drivers, GPU/adapter, and installation origin. Distinguish native Windows
   from WSL and containers; do not switch environments to satisfy a dependency.
2. Inspect the installed `rosidl_buffer`, `rosidl_buffer_backend`, and
   `rosidl_buffer_backend_registry` APIs and tests. Read the
   [ROSIDL contract](./references/rosidl-contract.md) before choosing an API.
   Lyrical/Rolling documentation is not evidence that Humble, Jazzy, Kilted, a
   vendor fork, or a particular binary package has the same features.
3. Verify the complete chain: message generator, runtime, typesupport, RMW
   endpoint/descriptor support, plugin registration/loading, and consumers.
   A header, installed CUDA toolkit, or DDS shared-memory option proves neither
   backend support nor zero-copy. Label each path **verified**, **unverified**,
   or **requires implementation**, citing the revision or test behind the label.
4. Identify CPU-only versus device consumers, same-process versus same-host IPC,
   different GPUs/adapters, remote subscribers, containers/users, and recording
   or visualization tools. Verify fallback per connection, including mixed
   consumers and intra-process delivery; not all CPU callbacks can directly
   access a non-CPU buffer.
5. Use [Pixi](../ros2-pixi/SKILL.md) for approved dependency/environment work.
   Preserve the existing ROS installation; do not replace core packages or
   upgrade the distro/drivers as an incidental optimization.

## Choose a platform reference

- [Linux managed buffers](./references/linux-buffers.md): CPU ownership and POSIX
  shared memory, V4L2 MMAP and DMA-BUF capture, synchronization, optional CUDA.
- [Windows native buffers and Media Foundation](./references/windows-media-foundation.md):
  CPU samples, shared mappings, D3D11/DXGI textures, secure handle transport, and
  native camera lifetime. Windows primitives are not proof of a shipped ROSIDL
  plugin; verify availability before proposing backend implementation.

Load only the relevant platform reference, plus the ROSIDL contract. Prefer an
existing supported conversion library/backend; if missing, explicitly separate
application work from backend/RMW development and get agreement on that scope.

## Define the data and ownership contracts

1. Preserve `sensor_msgs/msg/Image`, `CameraInfo`, or `PointCloud2` semantics:
   dimensions, encoding, signed source pitch, planes/offsets, point layout,
   endianness, valid bytes, timestamps, and frame IDs. A texture handle is not
   image bytes. Never place process pointers or OS handles in `Image.data`.
2. Establish a correct CPU reference using deterministic frames. For cameras,
   verify actual negotiated media type rather than assuming RGB, contiguous
   rows, one sample buffer, or a shareable GPU texture. Treat format changes as
   a new allocation/metadata generation, not reuse of old-size buffers.
3. Define one owner and explicit read/write access rules. Trace allocation,
   capture completion, publication, every consumer, GPU completion, and release.
   Do not return a camera sample or pool slot while any reader still uses it.
   Callback return, successful publication, and a live texture allocation are
   not interchangeable with completion of the captured frame's use.
4. Bound pools and outstanding frames. Specify behavior for a slow subscriber,
   late subscriber, reliable history, pool exhaustion, disconnect, and shutdown.
   A deadline alone does not make memory safe to overwrite. Use generations and
   validated release/completion tracking; define safe crash recovery rather than
   relying on a shared refcount surviving every process failure.
5. Keep published storage read-only to consumers. For mutation, allocate an
   output or request a documented independent copy. Do not assume copying a
   `rosidl::Buffer` aliases storage or that `.data()` exposes arbitrary backends.

## Implement in measured stages

1. Start with native capture and a valid CPU message, then introduce bounded
   reuse. Keep platform capture code separate from portable message metadata.
2. Add same-process lifetime-aware access through the verified backend/conversion
   APIs. Do not invent a generic external-pointer constructor or a backend flag.
3. Add same-host sharing only after confirming resource export/import, endpoint
   compatibility, synchronization, and secure peer authorization. Retain CPU
   fallback for unsupported consumers; cross-host Windows/Linux interoperability
   uses portable serialization, not shared native handles.
4. Maintain a **copy ledger**: capture/decode, format conversion, publication,
   IPC, CPU readback, serialization, and consumer conversion. For each edge record
   the storage owner/domain, bytes copied, allocations, waits, and evidence.
   One explicit copy into a safe pool can be better than fragile sample aliasing;
   describe it honestly rather than claiming end-to-end zero-copy.
5. Ensure CPU fallback preserves the same logical pixels/points and metadata,
   initializes transmitted padding, waits for producer completion, and never
   serializes descriptor bytes as sensor data. Reject malformed or unauthorized
   descriptors rather than treating them as ordinary fallback candidates.

## Validate correctness before performance

Use [testing](../ros2-test/SKILL.md) for permanent regressions and
[performance](../ros2-performance/SKILL.md) for matched, bounded measurements.

- Test layout/bounds/overflow, padded rows, planar formats, timestamp conversion,
  format changes, and CPU-versus-accelerated logical data equality.
- Test ownership with delayed readers, multiple subscribers, cancelled work,
  pool exhaustion, publisher/subscriber death, stale descriptors, unavailable
  GPU, invalid handles, permission denial, and repeated start/stop.
- Test same-process, same-host multi-process, and CPU/remote fallback separately.
  Add an ordinary image consumer and recorder; verify meaningful data, not just
  successful endpoint discovery. Observers can introduce conversions and copies.
- Use native Windows and native Linux test jobs for their respective adapters.
  Mocks test contracts, not camera drivers, DMA, GPU fences, or cross-process
  resource sharing. Use generated frames before approved camera access; do not
  capture, retain, or upload private images without authorization.
- Measure bytes copied per frame, allocations, pool high-water mark, dropped
  frames, CPU/GPU use, and latency percentiles. Compare identical formats/rates
  and identify uninstrumented driver/decoder copies. Do not infer zero-copy from
  pointer equality across processes or high frame rate alone.

Return the support matrix, source revisions, implementation versus proposal,
ownership diagram, copy ledger, validation results, and hardware/platform checks
not run. Do not present this skill's documentation tests as backend validation.

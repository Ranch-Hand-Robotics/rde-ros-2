# Linux managed buffers

These are native Linux allocation/capture mechanisms and backend design checks,
not a claim that a generic POSIX or V4L2 ROSIDL plugin is already installed.
Verify current kernel, drivers, APIs, and ROS/RMW support before implementation.

## CPU ownership and same-host shared memory

1. Establish ordinary CPU publication first. Reuse a bounded pool only after
   proving published buffers remain immutable until all users release them.
   Allocator reuse, intra-process delivery, DDS shared-memory transport, and
   ROSIDL backend descriptors are separate optimizations.
2. For a custom CPU shared-memory backend, POSIX `shm_open`, `ftruncate`, and
   `mmap` with `MAP_SHARED` provide storage. Use restrictive permissions and
   collision-resistant names with exclusive creation; do not truncate an
   existing live mapping. Initialize the region before advertising readiness.
3. Use offsets and validated lengths, not virtual addresses, in descriptors.
   Transfer file descriptors over an authenticated Unix-domain socket with
   `SCM_RIGHTS` where appropriate; an integer FD sent in a ROS message is not
   an imported descriptor in another process. Keep close-on-exec protection.
4. Define a process-shared synchronization and release protocol with a fixed
   metadata layout. Do not place process-local mutexes, pointers, or ordinary
   C++ owning containers in the region. `MAP_SHARED` does not establish ordering
   or reader completion; validate any cross-process atomic assumptions.
5. Account for `/dev/shm` limits, mount/IPC/user namespaces, container isolation,
   stale names, process death, and generation reuse. Do not disable isolation to
   make a demo pass. Unmap/close local resources and unlink names under the
   defined ownership protocol without invalidating active consumers.

The [POSIX shared-memory contract](https://pubs.opengroup.org/onlinepubs/9799919799/functions/shm_open.html)
describes creation, sizing, mapping, access modes, and persistence. Copying a
camera frame into this pool is still one copy unless the capture API demonstrably
writes into compatible application-provided storage.

## V4L2 capture buffers

1. Inspect capture capabilities, actual negotiated pixel format, plane count,
   `bytesperline`, `sizeimage`, bytes used, timestamps, and driver queue minimums.
   `V4L2_CAP_STREAMING` alone does not prove every memory mode is supported.
2. With `V4L2_MEMORY_MMAP`, request/query buffers and map each required plane.
   `VIDIOC_DQBUF` gives a completed frame to the application. Keep that buffer
   dequeued until every downstream user and device operation is done; only then
   issue `VIDIOC_QBUF`. A mapped address or exported FD does not prevent the
   driver overwriting a buffer that was requeued too early.
3. DMA-BUF **export** and **import** are different capabilities. `VIDIOC_EXPBUF`
   exports MMAP capture buffers (each plane separately where required);
   `V4L2_MEMORY_DMABUF` queues compatible externally allocated buffers for import.
   Probe the actual device/allocator combination; do not assume all UVC cameras,
   USB drivers, or GPU allocators support either direction.
4. Preserve format, plane offsets, stride, and any device layout/modifier
   requirements through export/import. An opaque or tiled device image is not
   necessarily a linear `Image.data` span. Use an explicit conversion/copy when
   the target backend or CPU fallback cannot represent it correctly.
5. For CPU access to mapped DMA-BUF storage, bracket access with
   `DMA_BUF_IOCTL_SYNC` START/END using the correct READ/WRITE flags and handle
   interrupted/retryable calls. This maintains cache coherency, **not mutual
   exclusion or device completion**. Honor the producer/consumer API's implicit
   fences or explicit sync-file protocol in addition to the cache operations.
6. Bound retained capture buffers so slow consumers cannot starve the driver.
   Stop streaming and drain/release outstanding work before unmapping resources;
   test unplug/reconnect, stream-off, and format changes. Copying into an owned
   pool can provide a safer bounded-latency fallback than holding driver buffers.

Primary references:
- [V4L2 memory-mapped capture](https://docs.kernel.org/userspace-api/media/v4l/mmap.html)
- [DMA-BUF importing](https://docs.kernel.org/userspace-api/media/v4l/dmabuf.html)
- [VIDIOC_EXPBUF exporting](https://docs.kernel.org/userspace-api/media/v4l/vidioc-expbuf.html)
- [DMA-BUF ownership and synchronization](https://docs.kernel.org/driver-api/dma-buf.html)

## Optional accelerator backends

The upstream [CUDA backend](https://github.com/ros2/rosidl_buffer_backends/tree/main/cuda_buffer_backend)
uses CUDA VMM and a host-local IPC protocol. At the checked 2026-10-10 baseline,
its documented fast path requires the same host, GPU, user, and supported VMM;
other connections use CPU fallback. This does not mean CUDA can automatically
import any V4L2 DMA-BUF or that Jetson and discrete-GPU capture paths are identical.

Its read/write handles coordinate stream completion and lifetime. Writing a
kernel directly into allocated message storage differs from `to_buffer()`, which
copies an existing host/device pointer. Verify GPU-driver/toolkit compatibility
and measure both initial capture transfers and subscriber-side conversion.
CUDA is optional; do not introduce it into a native CPU or another-vendor path.

## Required Linux validation

Use deterministic frames before approved camera capture. Test padded/multi-plane
layouts, delayed and crashed readers, bounded pools, permission failures, stale
FDs/generations, CPU fallback, and native multi-process release. Verify capture
buffers are not requeued early and native resources return to baseline after
repeated start/stop. Cross-machine or Windows peers require portable serialized
data; Linux FDs and DMA-BUF mappings are not cross-OS transports.

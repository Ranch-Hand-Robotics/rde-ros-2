# Windows native buffers and Media Foundation cameras

## Support boundary and implementation stages

This is native Windows design guidance, **not a shipped Windows ROSIDL backend**.
Microsoft documents the primitives below; their availability does not prove a
ROSIDL plugin or RMW integration exists. Verify the installed ROS revision and
current upstream before selecting or implementing a backend. No CUDA dependency
is required for the proposed CPU mapping or D3D11 paths, and WSL is not a
substitute for validating a native Media Foundation camera.

1. Correct CPU capture and standard ROS image publication.
2. Bounded sample/output ownership and measured same-process access.
3. A verified or explicitly implemented CPU shared-memory backend.
4. A verified or explicitly implemented D3D11 texture backend where camera,
   decoder, device, resource flags, and consumers support the path.

Do not promise end-to-end zero-copy at any stage. A CPU copy into shared storage,
a GPU copy into an exportable texture, decoding, and CPU readback are separate
copy/processing boundaries to measure.

## Media Foundation capture and lifetime

- Verify native MSVC/Windows SDK, ROS DLL/runtime compatibility, COM apartment
  initialization on participating threads, `MFStartup`/`MFShutdown` pairing, and
  the selected device's identity/access permissions. Opening or recording a real
  camera requires approval; use generated frames for initial tests.
- Query actual negotiated media types and each `IMFSample`'s buffers. Handle
  empty samples, stream ticks/end-of-stream, errors, and media-type changes.
  Do not assume a sample contains exactly one contiguous CPU buffer.
- Preserve capture time provenance. Media Foundation timestamps use 100-ns
  units; establish their time base and map deliberately to ROS time. Do not
  treat a stream-relative timestamp as Unix epoch or silently replace it with
  callback arrival time. Keep `CameraInfo` and optical frame IDs consistent.
- Retain the `IMFSample`/buffer and necessary COM references for the entire use
  of the frame, including asynchronous GPU operations and remote readers.
  `AddRef` on a texture keeps the allocation alive but is not proof that the
  capture allocator cannot reuse its contents. Verify the producer's sample
  lease/recycling contract; otherwise copy into an application-owned pool.
- Async Source Reader callbacks execute on worker threads. Keep capture callbacks
  short; bound work queues and define whether overload drops or backpressures.
  On shutdown stop issuing reads, request `Flush`, observe `OnFlush`, and also
  quiesce callbacks and downstream work before destroying state. `OnFlush` is
  not GPU/consumer completion. Do not wait while holding callback-required locks.

Sources: [asynchronous Source Reader](https://learn.microsoft.com/en-us/windows/win32/medfound/using-the-source-reader-in-asynchronous-mode),
[Flush contract](https://learn.microsoft.com/en-us/windows/win32/api/mfreadwrite/nf-mfreadwrite-imfsourcereader-flush).

## CPU sample access and image layout

1. Prefer `IMF2DBuffer2::Lock2DSize` with read-only flags for read access when
   supported. Respect the returned scanline, signed pitch, allocation bounds,
   and accessible length; pair successful locks with `Unlock2D`. A locked view
   cannot outlive the lock. Locking is not cross-thread/process synchronization.
2. `IMFMediaBuffer::Lock` can require a contiguous representation. Read-only 2D
   locking can avoid copy-back on unlock but does not prove there was no GPU
   readback. `ConvertToContiguousBuffer` copies when merging multiple buffers;
   for one buffer it returns the existing one. Record the actual path.
3. Preserve row padding, orientation, plane offsets, color space/range, and
   allocation bounds. NV12 has a Y plane followed by interleaved U,V chroma at
   4:2:0 sampling; it is not RGB and allocation size need not equal tightly packed
   width times height times 1.5. Handle signed RGB pitch without unsigned wrap.
4. Check the ROS distro's image encoding/schema and downstream support for the
   exact planar layout. If it cannot be represented portably, convert explicitly
   to a supported encoding and count that work. Do not relabel NV12 as RGB or
   publish native descriptors in `sensor_msgs/msg/Image.data`.

Sources: [Lock2DSize](https://learn.microsoft.com/en-us/windows/win32/api/mfobjects/nf-mfobjects-imf2dbuffer2-lock2dsize),
[IMFMediaBuffer::Lock](https://learn.microsoft.com/en-us/windows/win32/api/mfobjects/nf-mfobjects-imfmediabuffer-lock),
[ConvertToContiguousBuffer](https://learn.microsoft.com/en-us/windows/win32/api/mfobjects/nf-mfobjects-imfsample-converttocontiguousbuffer),
[image stride](https://learn.microsoft.com/en-us/windows/win32/medfound/image-stride),
[YUV layouts](https://learn.microsoft.com/en-us/windows/win32/medfound/recommended-8-bit-yuv-formats-for-video-rendering).

## Native CPU shared mappings

- `CreateFileMapping` with paging-file backing and `MapViewOfFile` provide
  application-owned shared storage, not automatic adoption of an MF buffer.
  Start with one explicit copy into a bounded mapped pool unless a verified
  capture allocator can directly fill compatible storage.
- Store versioned metadata, slot/generation, offsets, lengths, and ready/release
  state, not virtual pointers. Validate arithmetic/bounds before every mapping
  and view. Provide process-shared synchronization and an independent reader
  completion protocol; mapping visibility alone does not make publication safe.
- Prefer noninheritable handles and restrictive DACL permissions. `DuplicateHandle`
  creates a handle valid in an authorized target process; transmitting the
  original numeric `HANDLE` does not. Authenticate peers before granting access,
  use minimal rights, and handle partial import/cleanup failures.
- If using named mappings, prevent name collisions/squatting and inspect an
  already-existing object rather than trusting it. Do not request global
  namespaces, elevation, or broad permissions merely to bypass access failures.
- Each process calls `UnmapViewOfFile` and closes owned handles after its work
  completes. Define safe handling of crashed/slow readers, publisher restart,
  and outstanding generations. Separate read-only payload access from any
  writable release bookkeeping; do not give every subscriber frame write access.

Sources: [named shared memory](https://learn.microsoft.com/en-us/windows/win32/memory/creating-named-shared-memory),
[MapViewOfFile](https://learn.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-mapviewoffile),
[DuplicateHandle](https://learn.microsoft.com/en-us/windows/win32/api/handleapi/nf-handleapi-duplicatehandle),
[mapping security](https://learn.microsoft.com/en-us/windows/win32/memory/file-mapping-security-and-access-rights).

## D3D11/DXGI sample and sharing path

1. When supported, initialize `IMFDXGIDeviceManager` with the D3D11 device and
   supply `MF_SOURCE_READER_D3D_MANAGER`. This enables supported D3D/DXVA paths;
   it does not guarantee a camera delivers GPU samples or DMA directly into
   application textures. Verify actual sample interfaces at runtime.
2. Query `IMFDXGIBuffer`, then `GetResource` for `ID3D11Texture2D` and
   `GetSubresourceIndex`. Keep the returned COM interface and correct subresource;
   never assume index zero. `GetResource` fails while the buffer is locked.
3. Inspect adapter identity, texture format, dimensions, usage/bind/misc flags,
   and producer synchronization. For the documented keyed-mutex NT-handle path,
   create compatible resources with `D3D11_RESOURCE_MISC_SHARED_NTHANDLE` and
   `D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX`; export with
   `IDXGIResource1::CreateSharedHandle` and import with
   `ID3D11Device1::OpenSharedResource1`. Close owned NT handles correctly.
   Do not assume an arbitrary MF texture is shareable or add flags after creation.
4. If the sample cannot be shared, use a compatible application-owned texture
   and account for the GPU copy (or conversion). Retain the source sample until
   that work completes. Require matching adapters for the initial fast path;
   treat cross-adapter/device support as a separately validated capability.
5. Define `IDXGIKeyedMutex::AcquireSync`/`ReleaseSync` ownership keys and bounded
   waits for every participant. Check `WAIT_TIMEOUT` and `WAIT_ABANDONED`
   explicitly; `SUCCEEDED` alone is insufficient. A keyed mutex is exclusive,
   not a multi-subscriber broadcast/release protocol. Design fan-out deliberately.
6. A capability-gated D3D11 fence path can provide GPU `Signal`/`Wait` ordering;
   verify the required interfaces and resource/fence sharing separately. A fence
   is not a texture lease and does not supply cooperation from the MF producer.
   Define device-removed/reset recovery and never recycle a pending frame solely
   because a wait timed out.
7. Keep the texture opaque to generic CPU access. A custom ROSIDL storage
   implementation needs native read access plus deep cloning and correct
   `to_cpu()` conversion. CPU fallback may require synchronized staging/readback
   and row/plane repacking; it must produce the same logical image. Verify mixed
   GPU/CPU subscribers, recorder, and remote consumer behavior in the actual RMW.

Sources: [DXGI device manager](https://learn.microsoft.com/en-us/windows/win32/api/mfobjects/nn-mfobjects-imfdxgidevicemanager),
[Source Reader D3D manager](https://learn.microsoft.com/en-us/windows/win32/medfound/mf-source-reader-d3d-manager),
[GetResource](https://learn.microsoft.com/en-us/windows/win32/api/mfobjects/nf-mfobjects-imfdxgibuffer-getresource),
[GetSubresourceIndex](https://learn.microsoft.com/en-us/windows/win32/api/mfobjects/nf-mfobjects-imfdxgibuffer-getsubresourceindex),
[CreateSharedHandle](https://learn.microsoft.com/en-us/windows/win32/api/dxgi1_2/nf-dxgi1_2-idxgiresource1-createsharedhandle),
[OpenSharedResource1](https://learn.microsoft.com/en-us/windows/win32/api/d3d11_1/nf-d3d11_1-id3d11device1-opensharedresource1),
[AcquireSync](https://learn.microsoft.com/en-us/windows/win32/api/dxgi/nf-dxgi-idxgikeyedmutex-acquiresync),
[D3D11 fence ordering](https://learn.microsoft.com/en-us/windows/win32/api/d3d11_3/nn-d3d11_3-id3d11devicecontext4).

## Native Windows acceptance checks

Test padded/negative-pitch CPU frames, NV12 planes, multi-buffer samples, format
changes, permission denial, invalid/closed handles, wrong adapter, slow and
crashed consumers, device removal, and repeated start/stop. Use the D3D debug
layer and native profiling tools when available, without assuming their presence.
Verify COM/handle/pool counts return to baseline and serialized CPU fallback
matches the reference image. Native multi-process tests must prove resource
sharing and release; mocks and successful Linux tests cannot establish that.

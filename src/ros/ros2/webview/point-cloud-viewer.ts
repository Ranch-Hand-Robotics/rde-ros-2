/// <reference types="@webgpu/types" />
// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import { decodePointCloud, DecodedPointCloud } from "./point-cloud-data";
import { orbitFrameRotation, viewProjection, zoomDistance } from "./point-cloud-math";

const shader = /* wgsl */ `
struct Uniforms {
  mvp: mat4x4<f32>,
  centerRadius: vec4<f32>,
  colorRange: vec4<f32>, // near, far, depth mix, point diameter (pixels)
  viewport: vec4<f32>,
};
@group(0) @binding(0) var<uniform> u: Uniforms;
struct VertexOut {
  @builtin(position) position: vec4<f32>,
  @location(0) color: vec3<f32>,
  @location(1) uv: vec2<f32>,
};
fn depthColor(depth: f32) -> vec3<f32> {
  let t = clamp((depth - u.colorRange.x) / max(u.colorRange.y - u.colorRange.x, 0.000001), 0.0, 1.0);
  // Blue -> cyan -> green -> yellow -> red.
  return clamp(vec3<f32>(1.5) - abs(4.0 * t - vec3<f32>(3.0, 2.0, 1.0)), vec3<f32>(0.0), vec3<f32>(1.0));
}
@vertex fn pointVertex(@builtin(vertex_index) index: u32,
  @location(0) position: vec4<f32>, @location(1) color: vec4<f32>) -> VertexOut {
  let corners = array<vec2<f32>, 6>(vec2<f32>(-1, -1), vec2<f32>(1, -1), vec2<f32>(-1, 1),
    vec2<f32>(-1, 1), vec2<f32>(1, -1), vec2<f32>(1, 1));
  var out: VertexOut;
  let local = (position.xyz - u.centerRadius.xyz) / u.centerRadius.w;
  out.position = u.mvp * vec4<f32>(local, 1.0);
  out.position = vec4<f32>(out.position.xy + corners[index] * u.colorRange.w / u.viewport.xy * out.position.w,
    out.position.zw);
  out.color = mix(color.rgb, depthColor(position.w), u.colorRange.z);
  out.uv = corners[index];
  return out;
}
@fragment fn pointFragment(in: VertexOut) -> @location(0) vec4<f32> {
  if (dot(in.uv, in.uv) > 1.0) { discard; }
  return vec4<f32>(in.color, 1.0);
}
@vertex fn axisVertex(@location(0) position: vec4<f32>, @location(1) color: vec4<f32>) -> VertexOut {
  var out: VertexOut;
  out.position = u.mvp * vec4<f32>(position.xyz, 1.0);
  out.color = color.rgb;
  out.uv = vec2<f32>(0.0);
  return out;
}
@fragment fn axisFragment(in: VertexOut) -> @location(0) vec4<f32> {
  return vec4<f32>(in.color, 1.0);
}
`;

export interface PointCloudPreview {
  update(data: unknown): void;
  clear(): void;
  dispose(): void;
}

declare global {
  interface Window {
    createPointCloudViewer: (container: HTMLElement) => PointCloudPreview;
  }
}

/** One GPU device/canvas per topic. Renders on demand, never in an idle animation loop. */
class PointCloudViewer implements PointCloudPreview {
  private readonly canvas: HTMLCanvasElement;
  private readonly status: HTMLElement;
  private readonly metadata: HTMLElement;
  private readonly abort = new AbortController();
  private readonly resize: ResizeObserver;
  private device?: GPUDevice;
  private context?: GPUCanvasContext;
  private pipeline?: GPURenderPipeline;
  private axisPipeline?: GPURenderPipeline;
  private uniform?: GPUBuffer;
  private bindGroup?: GPUBindGroup;
  private points?: GPUBuffer;
  private axes?: GPUBuffer;
  private depth?: GPUTexture;
  private depthWidth = 0;
  private depthHeight = 0;
  private bufferSize = 0;
  private cloud?: DecodedPointCloud;
  private fitCenter: [number, number, number] = [0, 0, 0];
  private fitRadius = 1;
  private fitted = false;
  private readonly yaw = -Math.PI / 4;
  private readonly pitch = Math.PI / 5;
  private distance = 4;
  private frame = 0;
  private disposed = false;
  private gpuError = "";
  private dataError = "";
  private dragging?: { x: number; y: number; id: number };

  constructor(private readonly container: HTMLElement) {
    // Static markup only: ROS-provided strings are always assigned via textContent.
    container.innerHTML = `
      <div class="cloud-heading"><h2>Point cloud <span class="cloud-badge">WebGPU</span></h2>
        <button type="button" class="control secondary" data-cloud="fit">Fit view</button>
        <button type="button" class="control secondary" data-cloud="reset">Reset</button></div>
      <div class="cloud-controls">
        <label>Color <select data-cloud="color"><option value="depth">Depth</option><option value="rgb">RGB</option><option value="mix">RGB + depth</option></select></label>
        <label>Depth mix <input data-cloud="mix" type="range" min="0" max="100" value="50"><output data-cloud="mix-value">50%</output></label>
        <label>Point size <input data-cloud="size" type="range" min="1" max="10" value="3"><output data-cloud="size-value">3 px</output></label>
        <label><input data-cloud="auto" type="checkbox" checked>Auto depth</label>
        <label>Near (m) <input data-cloud="near" type="number" min="0" step="any" value="0" disabled></label>
        <label>Far (m) <input data-cloud="far" type="number" min="0" step="any" value="10" disabled></label>
      </div>
      <div class="cloud-range-error" data-cloud="range-error" role="alert" hidden>Far must be greater than near.</div>
      <fieldset class="cloud-rotation"><legend>Frame rotation (degrees, X → Y → Z)</legend>
        <label>X <input data-cloud="rx" type="number" min="-180" max="180" value="0" step="any"></label>
        <label>Y <input data-cloud="ry" type="number" min="-180" max="180" value="0" step="any"></label>
        <label>Z <input data-cloud="rz" type="number" min="-180" max="180" value="0" step="any"></label>
        <label><input data-cloud="axes" type="checkbox" checked>Frame axes at cloud center</label>
      </fieldset>
      <div class="cloud-viewport"><canvas data-cloud="canvas" tabindex="0" aria-label="3D point cloud. Drag or use arrow keys to orbit; scroll or use plus and minus to zoom; F to fit."></canvas>
        <div class="cloud-status" data-cloud="status" role="status">Initializing WebGPU…</div>
        <div class="cloud-hint">Drag to orbit · Scroll to zoom · F to fit <span class="cloud-axis-x">X</span> <span class="cloud-axis-y">Y</span> <span class="cloud-axis-z">Z</span></div>
      </div>
      <div class="cloud-footer"><span data-cloud="metadata">Waiting for PointCloud2 messages</span>
        <span class="cloud-legend" data-cloud="legend"><span data-cloud="legend-near">Near</span><span class="cloud-ramp" aria-hidden="true"></span><span data-cloud="legend-far">Far</span></span></div>`;
    this.canvas = this.element<HTMLCanvasElement>("canvas");
    this.status = this.element("status");
    this.metadata = this.element("metadata");
    this.resize = new ResizeObserver(() => this.schedule());
    this.resize.observe(this.canvas);
    this.addControls();
    this.syncControls();
    window.addEventListener("pagehide", () => this.dispose(), { signal: this.abort.signal });
    document.addEventListener("visibilitychange", () => this.schedule(), { signal: this.abort.signal });
    void this.initialize();
  }

  private element<T extends HTMLElement = HTMLElement>(name: string): T {
    return this.container.querySelector<T>(`[data-cloud="${name}"]`)!;
  }

  private number(name: string, fallback: number, min: number, max: number): number {
    const input = this.element<HTMLInputElement>(name);
    const value = input.value === "" ? NaN : Number(input.value);
    return Number.isFinite(value) ? Math.min(max, Math.max(min, value)) : fallback;
  }

  private async initialize(): Promise<void> {
    try {
      if (!navigator.gpu) {
        throw new Error("WebGPU is unavailable. Use a recent VS Code with hardware acceleration and a supported GPU/driver.");
      }
      const adapter = await navigator.gpu.requestAdapter();
      if (!adapter) { throw new Error("No WebGPU adapter is available. Check GPU drivers and VS Code hardware acceleration."); }
      if (this.disposed) { return; }
      const device = await adapter.requestDevice();
      if (this.disposed) { device.destroy(); return; }
      this.device = device;
      void device.lost.then(info => {
        if (!this.disposed && this.device === device) { this.fail(`WebGPU device lost: ${info.message || info.reason}. Close and reopen this topic to retry.`); }
      });
      device.addEventListener("uncapturederror", event => this.fail(`WebGPU: ${event.error.message}`), { signal: this.abort.signal });
      this.context = this.canvas.getContext("webgpu") ?? undefined;
      if (!this.context) { throw new Error("A WebGPU canvas could not be created."); }
      const format = navigator.gpu.getPreferredCanvasFormat();
      this.context.configure({ device, format, alphaMode: "opaque" });
      const module = device.createShaderModule({ label: "PointCloud2 shaders", code: shader });
      const compilation = await module.getCompilationInfo();
      if (this.disposed || this.device !== device) { return; }
      const errors = compilation.messages.filter(message => message.type === "error");
      if (errors.length) { throw new Error(errors.map(error => error.message).join("; ")); }
      const layout = device.createBindGroupLayout({ entries: [{ binding: 0, visibility: GPUShaderStage.VERTEX,
        buffer: { type: "uniform" } }] });
      const pipelineLayout = device.createPipelineLayout({ bindGroupLayouts: [layout] });
      const attributes: GPUVertexAttribute[] = [
        { shaderLocation: 0, offset: 0, format: "float32x4" },
        { shaderLocation: 1, offset: 16, format: "float32x4" }
      ];
      this.pipeline = await device.createRenderPipelineAsync({
        layout: pipelineLayout,
        vertex: { module, entryPoint: "pointVertex", buffers: [{ arrayStride: 32, stepMode: "instance", attributes }] },
        fragment: { module, entryPoint: "pointFragment", targets: [{ format }] },
        primitive: { topology: "triangle-list" },
        depthStencil: { format: "depth24plus", depthWriteEnabled: true, depthCompare: "less" }
      });
      if (this.disposed || this.device !== device) { return; }
      this.axisPipeline = await device.createRenderPipelineAsync({
        layout: pipelineLayout,
        vertex: { module, entryPoint: "axisVertex", buffers: [{ arrayStride: 32, attributes }] },
        fragment: { module, entryPoint: "axisFragment", targets: [{ format }] },
        primitive: { topology: "line-list" },
        depthStencil: { format: "depth24plus", depthWriteEnabled: false, depthCompare: "always" }
      });
      if (this.disposed || this.device !== device) { return; }
      this.uniform = device.createBuffer({ size: 112, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
      this.bindGroup = device.createBindGroup({ layout, entries: [{ binding: 0, resource: { buffer: this.uniform } }] });
      const axisData = new Float32Array([
        0, 0, 0, 1, 1, 0.25, 0.3, 1, 0.6, 0, 0, 1, 1, 0.25, 0.3, 1,
        0, 0, 0, 1, 0.35, 1, 0.45, 1, 0, 0.6, 0, 1, 0.35, 1, 0.45, 1,
        0, 0, 0, 1, 0.3, 0.6, 1, 1, 0, 0, 0.6, 1, 0.3, 0.6, 1, 1
      ]);
      this.axes = device.createBuffer({ size: axisData.byteLength, usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
      device.queue.writeBuffer(this.axes, 0, axisData);
      this.upload();
      this.showStatus(this.cloud ? (this.cloud.count ? "" : "No finite points in this cloud.") : "Waiting for PointCloud2 messages");
      this.schedule();
    } catch (error) {
      if (!this.disposed) { this.fail(error instanceof Error ? error.message : String(error)); }
    }
  }

  private showStatus(message: string): void {
    this.status.textContent = this.gpuError || this.dataError || message;
    this.status.hidden = !this.status.textContent;
  }

  private fail(message: string): void {
    this.gpuError = message;
    this.showStatus(message);
    cancelAnimationFrame(this.frame);
    this.frame = 0;
    this.destroyResources();
  }

  public update(data: unknown): void {
    if (this.disposed) { return; }
    this.dataError = "";
    try {
      if (data && typeof data === "object" && "previewError" in data) {
        throw new Error(String(data.previewError));
      }
      this.cloud = decodePointCloud(data);
      const cloud = this.cloud;
      if (cloud.count && !this.fitted) { this.fit(); }
      this.upload();
      this.metadata.textContent = `${cloud.count.toLocaleString()} / ${cloud.totalPoints.toLocaleString()} points · Frame: ${cloud.frameId || "unspecified"}`
        + (cloud.sampledPoints < cloud.totalPoints ? " · sampled" : "")
        + (cloud.skippedPoints ? ` · ${cloud.skippedPoints.toLocaleString()} invalid samples skipped` : "")
        + (!cloud.hasColor ? " · no RGB field; using depth" : "");
      this.syncControls();
      this.showStatus(cloud.count ? "" : "No finite points in this cloud.");
    } catch (error) {
      this.cloud = undefined;
      this.metadata.textContent = "PointCloud2 could not be decoded";
      this.dataError = error instanceof Error ? error.message : String(error);
      this.showStatus(this.dataError);
    }
    this.schedule();
  }

  private upload(): void {
    if (!this.device || !this.cloud?.count || this.gpuError) { return; }
    const data = this.cloud.vertices;
    if (data.byteLength > this.bufferSize) {
      this.points?.destroy();
      this.bufferSize = data.byteLength;
      this.points = this.device.createBuffer({ size: this.bufferSize, usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
    }
    this.device.queue.writeBuffer(this.points!, 0, data.buffer, data.byteOffset, data.byteLength);
  }

  private fit(): void {
    if (!this.cloud?.count) { return; }
    this.fitCenter = this.cloud.center;
    this.fitRadius = Math.max(0.000001, this.cloud.radius);
    const aspect = Math.max(0.1, this.canvas.clientWidth / Math.max(1, this.canvas.clientHeight));
    this.distance = 1.1 / Math.sin(Math.atan(Math.tan(Math.PI / 8) * Math.min(1, aspect)));
    this.fitted = true;
    this.schedule();
  }

  private syncControls(): void {
    const mode = this.element<HTMLSelectElement>("color").value;
    this.element<HTMLInputElement>("mix").disabled = mode !== "mix" || !this.cloud?.hasColor;
    this.element("mix-value").textContent = `${this.number("mix", 50, 0, 100)}%`;
    this.element("size-value").textContent = `${this.number("size", 3, 1, 10)} px`;
    const auto = this.element<HTMLInputElement>("auto").checked;
    for (const name of ["near", "far"]) { this.element<HTMLInputElement>(name).disabled = auto; }
    if (auto && this.cloud) {
      this.element<HTMLInputElement>("near").value = String(Number(this.cloud.minDepth.toPrecision(6)));
      this.element<HTMLInputElement>("far").value = String(Number(Math.max(this.cloud.minDepth + 0.001, this.cloud.maxDepth).toPrecision(6)));
    }
    const near = this.number("near", 0, 0, 1e38), far = this.number("far", 10, 0, 1e38);
    this.element("legend-near").textContent = `${near.toFixed(2)} m`;
    this.element("legend-far").textContent = `${far.toFixed(2)} m`;
    this.element("legend").hidden = mode === "rgb" && !!this.cloud?.hasColor;
    this.element("range-error").hidden = auto || far > near;
    this.element<HTMLInputElement>("far").setCustomValidity(far > near ? "" : "Far must be greater than near.");
    this.schedule();
  }

  private addControls(): void {
    const options = { signal: this.abort.signal };
    this.container.addEventListener("input", () => this.syncControls(), options);
    this.element("fit").addEventListener("click", () => this.fit(), options);
    this.element("reset").addEventListener("click", () => {
      for (const name of ["rx", "ry", "rz"]) { this.element<HTMLInputElement>(name).value = "0"; }
      this.fit();
      this.schedule();
    }, options);
    this.canvas.addEventListener("pointerdown", event => {
      if (event.button !== 0) { return; }
      this.dragging = { x: event.clientX, y: event.clientY, id: event.pointerId };
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.focus();
    }, options);
    this.canvas.addEventListener("pointermove", event => {
      if (!this.dragging || this.dragging.id !== event.pointerId) { return; }
      this.orbit((this.dragging.x - event.clientX) * 0.008, (event.clientY - this.dragging.y) * 0.008);
      this.dragging.x = event.clientX;
      this.dragging.y = event.clientY;
    }, options);
    for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) {
      this.canvas.addEventListener(event, () => { this.dragging = undefined; }, options);
    }
    this.canvas.addEventListener("wheel", event => {
      event.preventDefault();
      this.zoom(Math.exp(Math.max(-1, Math.min(1, event.deltaY * 0.001))));
    }, { ...options, passive: false });
    this.canvas.addEventListener("keydown", event => {
      switch (event.key) {
        case "ArrowLeft": this.orbit(0.1, 0); break;
        case "ArrowRight": this.orbit(-0.1, 0); break;
        case "ArrowUp": this.orbit(0, 0.1); break;
        case "ArrowDown": this.orbit(0, -0.1); break;
        case "+": case "=": this.zoom(0.9); break;
        case "-": this.zoom(1.1); break;
        case "f": case "F": this.fit(); break;
        default: return;
      }
      event.preventDefault();
    }, options);
  }

  private rotation(): [number, number, number] {
    return ["rx", "ry", "rz"].map(name => this.number(name, 0, -180, 180)) as [number, number, number];
  }

  private orbit(yaw: number, pitch: number): void {
    const rotation = orbitFrameRotation(this.rotation(), this.yaw, this.pitch, yaw, pitch);
    ["rx", "ry", "rz"].forEach((name, index) => {
      this.element<HTMLInputElement>(name).value = String(Number(rotation[index].toFixed(4)));
    });
    this.schedule();
  }

  private zoom(factor: number): void {
    this.distance = zoomDistance(this.distance, factor, this.fitRadius);
    this.schedule();
  }

  private schedule(): void {
    if (this.frame || this.disposed || this.gpuError || document.hidden) { return; }
    this.frame = requestAnimationFrame(() => {
      this.frame = 0;
      try { this.draw(); } catch (error) { this.fail(String(error)); }
    });
  }

  private draw(): void {
    if (!this.device || !this.pipeline || !this.axisPipeline || !this.bindGroup || !this.context) { return; }
    const scale = Math.min(window.devicePixelRatio || 1, 2);
    const limit = Math.min(this.device.limits.maxTextureDimension2D, 4096);
    const width = Math.max(1, Math.min(limit, Math.round(this.canvas.clientWidth * scale)));
    const height = Math.max(1, Math.min(limit, Math.round(this.canvas.clientHeight * scale)));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    if (!this.depth || width !== this.depthWidth || height !== this.depthHeight) {
      this.depth?.destroy();
      this.depth = this.device.createTexture({ size: [width, height], format: "depth24plus", usage: GPUTextureUsage.RENDER_ATTACHMENT });
      this.depthWidth = width; this.depthHeight = height;
    }
    const values = new Float32Array(28);
    values.set(viewProjection(this.yaw, this.pitch, this.distance, width / height, this.rotation()));
    values.set([...this.fitCenter, this.fitRadius], 16);
    const mode = this.element<HTMLSelectElement>("color").value;
    const mix = !this.cloud?.hasColor || mode === "depth" ? 1 : mode === "rgb" ? 0 : this.number("mix", 50, 0, 100) / 100;
    const near = this.number("near", 0, 0, 1e38);
    const far = Math.max(near + 0.001, this.number("far", 10, 0, 1e38));
    values.set([near, far, mix, this.number("size", 3, 1, 10) * scale], 20);
    values.set([width, height, 0, 0], 24);
    this.device.queue.writeBuffer(this.uniform!, 0, values);
    const encoder = this.device.createCommandEncoder();
    const pass = encoder.beginRenderPass({
      colorAttachments: [{ view: this.context.getCurrentTexture().createView(), clearValue: { r: 0.025, g: 0.035, b: 0.055, a: 1 }, loadOp: "clear", storeOp: "store" }],
      depthStencilAttachment: { view: this.depth.createView(), depthClearValue: 1, depthLoadOp: "clear", depthStoreOp: "store" }
    });
    pass.setBindGroup(0, this.bindGroup);
    if (this.cloud?.count && this.points) {
      pass.setPipeline(this.pipeline);
      pass.setVertexBuffer(0, this.points);
      pass.draw(6, this.cloud.count);
      if (this.axes && this.element<HTMLInputElement>("axes").checked) {
        pass.setPipeline(this.axisPipeline);
        pass.setVertexBuffer(0, this.axes);
        pass.draw(6);
      }
    }
    pass.end();
    this.device.queue.submit([encoder.finish()]);
  }

  public clear(): void {
    this.cloud = undefined;
    this.dataError = "";
    this.fitted = false;
    this.points?.destroy(); this.points = undefined; this.bufferSize = 0;
    this.metadata.textContent = "Waiting for PointCloud2 messages";
    this.showStatus("Waiting for PointCloud2 messages");
    this.schedule();
  }

  private destroyResources(): void {
    this.points?.destroy(); this.points = undefined; this.bufferSize = 0;
    this.axes?.destroy(); this.axes = undefined;
    this.uniform?.destroy(); this.uniform = undefined;
    this.depth?.destroy(); this.depth = undefined;
    this.context?.unconfigure(); this.context = undefined;
    const device = this.device;
    this.device = undefined;
    device?.destroy();
  }

  public dispose(): void {
    this.disposed = true;
    cancelAnimationFrame(this.frame);
    this.resize.disconnect();
    this.abort.abort();
    this.cloud = undefined;
    this.destroyResources();
  }
}

window.createPointCloudViewer = container => new PointCloudViewer(container);
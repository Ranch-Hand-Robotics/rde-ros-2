// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as assert from "assert";
import { frameRotation, frameRotationDegrees, multiplyMatrices, orbitFrameRotation, viewProjection, zoomDistance } from "../../src/ros/ros2/webview/point-cloud-math";

function transform(matrix: ArrayLike<number>, point: number[]): number[] {
  return [0, 1, 2, 3].map(row => point.reduce((sum, value, column) => sum + matrix[column * 4 + row] * value, 0));
}

function close(actual: number[], expected: number[]): void {
  actual.forEach((value, index) => assert.ok(Math.abs(value - expected[index]) < 0.00001, `${actual} != ${expected}`));
}

describe("Point cloud camera math", () => {
  it("zooms below a millimeter independently of cloud radius", () => {
    for (const radius of [0.001, 1, 1000]) {
      let distance = 4;
      while (distance * radius >= 0.001) {
        const next = zoomDistance(distance, 0.9, radius);
        assert.ok(next < distance, "Zoom must not stop at the cloud boundary");
        distance = next;
      }
      assert.ok(distance * radius < 0.001);
      for (let i = 0; i < 1000; i++) { distance = zoomDistance(distance, 0.9, radius); }
      assert.ok(Math.abs(distance * radius - 1e-6) < 1e-12, "Only a micrometer-scale lower safeguard remains");
      assert.ok(zoomDistance(distance, 1.1, radius) > distance, "Can zoom back out from the safeguard");
    }
  });

  it("keeps millimeter details visible and depth ordered when zoomed inside a large cloud", () => {
    for (const radius of [0.001, 1, 1000]) {
      // Camera 1 mm from the target; points separated by 0.1 mm in screen space.
      const distance = 0.001 / radius;
      const matrix = viewProjection(0, 0, distance, 1, [0, 0, 0]);
      const center = transform(matrix, [0, 0, 0, 1]);
      const detail = transform(matrix, [0, 0.0001 / radius, 0, 1]);
      const closer = transform(matrix, [0.0005 / radius, 0, 0, 1]);
      const behind = transform(matrix, [0.002 / radius, 0, 0, 1]);
      for (const point of [center, detail, closer]) {
        assert.ok(point[3] > 0 && point[2] > 0 && point[2] < point[3], "Detail must be inside the clipping planes");
      }
      assert.ok(closer[2] / closer[3] < center[2] / center[3], "Depth buffer must distinguish nearby points");
      close([detail[0] / detail[3]], [0.1 / Math.tan(Math.PI / 8)]);
      assert.ok(behind[3] < 0, "Points behind the camera remain clipped");
    }
  });

  it("retains finite, visible projections at both zoom safeguards", () => {
    for (const distance of [zoomDistance(4, 1e-20, 1000), zoomDistance(100, 2, 1), zoomDistance(1e30, 2, 1)]) {
      const matrix = viewProjection(0, 0, distance, 1, [0, 0, 0]);
      assert.ok(Array.from(matrix).every(Number.isFinite));
      const point = transform(matrix, [0, 0, 0, 1]);
      assert.ok(point[3] > 0 && point[2] > 0 && point[2] < point[3]);
    }
    assert.strictEqual(zoomDistance(100, 2, 1), 200, "Zoom out is no longer capped at 100 cloud radii");
  });

  it("round-trips frame angles, including gimbal lock and wrapped representations", () => {
    for (const angles of [[0, 0, 0], [23, -48, 71], [35, 90, 70], [-25, -90, 15], [179, 120, -179], [12, 89.99, 45]]) {
      const matrix = frameRotation(angles[0], angles[1], angles[2]);
      const degrees = frameRotationDegrees(matrix);
      assert.ok(degrees.every(value => Number.isFinite(value) && Math.abs(value) <= 180));
      close(Array.from(frameRotation(...degrees)), Array.from(matrix));
    }
  });

  it("makes displayed angles reproduce the orbit without a separate camera rotation", () => {
    const yaw = -Math.PI / 4, pitch = Math.PI / 5;
    for (const rotation of [[0, 0, 0], [20, 35, -70], [35, 90, 70]] as [number, number, number][]) {
      for (const [dx, dy] of [[0.1, 0], [0, 0.2], [-0.3, 0.4]]) {
        const degrees = orbitFrameRotation(rotation, yaw, pitch, dx, dy);
        assert.notDeepStrictEqual(degrees, rotation);
        close(Array.from(viewProjection(yaw, pitch, 4, 1.5, degrees)),
          Array.from(viewProjection(yaw + dx, pitch + dy, 4, 1.5, rotation)));
      }
    }
  });

  it("keeps repeated orbit rotations finite and returns on reverse single-axis drags", () => {
    const yaw = -Math.PI / 4, pitch = Math.PI / 5;
    const original: [number, number, number] = [25, -15, 40];
    for (const [dx, dy] of [[0.5, 0], [0, 0.5]]) {
      const moved = orbitFrameRotation(original, yaw, pitch, dx, dy);
      const reversed = orbitFrameRotation(moved, yaw, pitch, -dx, -dy);
      close(Array.from(frameRotation(...reversed)), Array.from(frameRotation(...original)));
    }
    let rotation = original;
    for (let i = 0; i < 1000; i++) {
      rotation = orbitFrameRotation(rotation, yaw, pitch, 0.1, 0.05);
      assert.ok(rotation.every(value => Number.isFinite(value) && Math.abs(value) <= 180));
    }
    const point = transform(frameRotation(...rotation), [1, 2, 3, 1]);
    assert.ok(Math.abs(Math.hypot(...point.slice(0, 3)) - Math.sqrt(14)) < 0.00001);
  });

  it("uses right-handed X, Y and Z rotations", () => {
    close(transform(frameRotation(90, 0, 0), [0, 1, 0, 1]), [0, 0, 1, 1]);
    close(transform(frameRotation(0, 90, 0), [0, 0, 1, 1]), [1, 0, 0, 1]);
    close(transform(frameRotation(0, 0, 90), [1, 0, 0, 1]), [0, 1, 0, 1]);
  });

  it("applies local X then Y then Z and preserves lengths", () => {
    const rotation = frameRotation(23, -48, 71);
    const point = [1, 2, 3, 1];
    const composed = transform(frameRotation(0, 0, 71), transform(frameRotation(0, -48, 0), transform(frameRotation(23, 0, 0), point)));
    close(transform(rotation, point), composed);
    const rotated = transform(rotation, point);
    assert.ok(Math.abs(Math.hypot(...rotated.slice(0, 3)) - Math.sqrt(14)) < 0.00001);
    close(Array.from(multiplyMatrices(frameRotation(0, 0, 0), rotation)), Array.from(rotation));
  });

  it("maps the target to screen center with WebGPU depth in 0..1", () => {
    for (const aspect of [0.5, 1, 2]) {
      const mvp = viewProjection(-0.8, 0.6, 4, aspect, [0, 0, 0]);
      const point = transform(mvp, [0, 0, 0, 1]);
      close([point[0], point[1], point[3]], [0, 0, 4]);
      assert.ok(point[2] / point[3] > 0 && point[2] / point[3] < 1);
      assert.ok(Array.from(mvp).every(Number.isFinite));
    }
  });

  it("keeps ROS +Z up and accounts for viewport aspect", () => {
    const square = viewProjection(0, 0, 4, 1, [0, 0, 0]);
    const wide = viewProjection(0, 0, 4, 2, [0, 0, 0]);
    assert.ok(transform(square, [0, 0, 1, 1])[1] > 0);
    const point = [0, 1, 0, 1];
    assert.ok(transform(square, point)[0] > 0);
    close([transform(wide, point)[0]], [transform(square, point)[0] / 2]);
    assert.ok(transform(square, [5, 0, 0, 1])[3] < 0, "Points behind the camera must be clipped");
  });
});
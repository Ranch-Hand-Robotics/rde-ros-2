// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

/** Column-major matrices, right-handed ROS coordinates (+Z up), WebGPU depth 0..1. */
export function multiplyMatrices(a: ArrayLike<number>, b: ArrayLike<number>): Float32Array {
  const out = new Float32Array(16);
  for (let column = 0; column < 4; column++) {
    for (let row = 0; row < 4; row++) {
      for (let k = 0; k < 4; k++) {
        out[column * 4 + row] += a[k * 4 + row] * b[column * 4 + k];
      }
    }
  }
  return out;
}

/** Local X, then Y, then Z rotation, in degrees. */
export function frameRotation(x: number, y: number, z: number): Float32Array {
  const [sx, sy, sz] = [x, y, z].map(value => Math.sin(value * Math.PI / 180));
  const [cx, cy, cz] = [x, y, z].map(value => Math.cos(value * Math.PI / 180));
  return multiplyMatrices(
    [cz, sz, 0, 0, -sz, cz, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    multiplyMatrices(
      [cy, 0, -sy, 0, 0, 1, 0, 0, sy, 0, cy, 0, 0, 0, 0, 1],
      [1, 0, 0, 0, 0, cx, sx, 0, 0, -sx, cx, 0, 0, 0, 0, 1]
    )
  );
}

function cameraOrientation(yaw: number, pitch: number): number[] {
  const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
  // Camera basis: right = up cross backward; up = backward cross right.
  return [-sy, -cy * sp, cy * cp, 0, cy, -sy * sp, sy * cp, 0,
    0, cp, sp, 0, 0, 0, 0, 1];
}

/** Recover X → Y → Z degrees; at gimbal lock choose the equivalent Z = 0 representation. */
export function frameRotationDegrees(matrix: ArrayLike<number>): [number, number, number] {
  const cosY = Math.hypot(matrix[0], matrix[1]);
  const x = cosY > 1e-6 ? Math.atan2(matrix[6], matrix[10]) : Math.atan2(-matrix[9], matrix[5]);
  const y = Math.atan2(-matrix[2], cosY);
  const z = cosY > 1e-6 ? Math.atan2(matrix[1], matrix[0]) : 0;
  return [x, y, z].map(value => value * 180 / Math.PI) as [number, number, number];
}

/** Fold a screen-relative camera orbit into the same frame rotation used by the degree inputs. */
export function orbitFrameRotation(rotation: [number, number, number], yaw: number, pitch: number,
  deltaYaw: number, deltaPitch: number): [number, number, number] {
  const view = cameraOrientation(yaw, pitch);
  // The inverse of the camera's orthonormal rotation is its transpose.
  const inverse = view.map((_value, index) => view[(index % 4) * 4 + Math.floor(index / 4)]);
  const delta = multiplyMatrices(inverse, cameraOrientation(yaw + deltaYaw, pitch + deltaPitch));
  return frameRotationDegrees(multiplyMatrices(delta, frameRotation(...rotation)));
}

/** Distances are in cloud-radius units; allow close-ups down to one micrometer. */
export function zoomDistance(distance: number, factor: number, radius: number): number {
  // Keep GPU matrix values representable without imposing a cloud-sized zoom stop.
  return Math.max(1e-6 / radius, Math.min(1e30, distance * factor));
}

export function viewProjection(yaw: number, pitch: number, distance: number,
  aspect: number, rotation: [number, number, number]): Float32Array {
  const view = cameraOrientation(yaw, pitch);
  view[14] = -distance;
  // Follow the camera into the cloud instead of clipping small details at a fixed plane.
  const near = distance * 0.001, far = Math.max(1000, distance * 2), f = 1 / Math.tan(Math.PI / 8);
  const projection = [f / aspect, 0, 0, 0, 0, f, 0, 0,
    0, 0, far / (near - far), -1, 0, 0, near * far / (near - far), 0];
  return multiplyMatrices(multiplyMatrices(projection, view), frameRotation(...rotation));
}
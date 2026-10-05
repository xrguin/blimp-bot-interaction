import test from 'node:test';
import assert from 'node:assert/strict';
import { buildLensMap, cameraPose, cameraRotationThree, distortPoint, effectiveIntrinsics,
  matVec, projectOptical, projectionElements, undistortPoint } from '../camera-math.js';

const profile = { width: 640, height: 480, reference_width: 640, reference_height: 480,
  K: [[554.256258422, 0, 319.5], [0, 554.256258422, 239.5], [0, 0, 1]], D: [0, 0, 0, 0, 0],
  translation_body_m: [0, 0, 0.312], rotation_optical_to_body: [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
  near_m: 0.001, far_m: 50 };
const near = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);

test('scaled intrinsics retain the pixel-center convention and reject aspect changes', () => {
  const k = effectiveIntrinsics({ ...profile, width: 320, height: 240 });
  near(k.cx, 159.5); near(k.cy, 119.5); near(k.fx, profile.K[0][0] / 2);
  const offset = effectiveIntrinsics({ ...profile, K: [[570, 0, 300], [0, 590, 220], [0, 0, 1]], width: 320, height: 240 });
  near(offset.cx, 149.75); near(offset.cy, 109.75);
  assert.throws(() => effectiveIntrinsics({ ...profile, width: 320 }));
});

test('the downward camera has body-right at image right and body-forward at image top', () => {
  const pose = cameraPose([0, 0, -1.325, 0, 0, 0], profile);
  near(pose.position_ned_m[2], -1.013);
  assert.deepEqual(matVec(pose.rotation_optical_to_ned, [0, 0, 1]), [0, 0, 1]);
  assert.deepEqual(matVec(pose.rotation_optical_to_ned, [1, 0, 0]), [0, 1, 0]);
  assert.deepEqual(matVec(pose.rotation_optical_to_ned, [0, -1, 0]), [1, 0, 0]);
  const turned = cameraPose([1, 2, -2, 0, 0, Math.PI / 2], profile);
  const right = matVec(turned.rotation_optical_to_ned, [1, 0, 0]);
  near(right[0], -1); near(right[1], 0);
  const pitched = cameraPose([0, 0, -1, 0, Math.PI / 2, 0], profile);
  near(pitched.position_ned_m[0], 0.312); near(pitched.position_ned_m[2], -1);
});

test('Three projection matches a nonsymmetric calibrated pinhole at zero-indexed pixels', () => {
  const k = { fx: 520, fy: 590, cx: 307.25, cy: 219.75 };
  const p = [0.2, -0.15, 1.7];
  const pixels = projectOptical(p, k);
  const matrix = projectionElements(k, 640, 480, 0.001, 50);
  const glPoint = [p[0], -p[1], -p[2], 1];
  const clip = Array.from({ length: 4 }, (_, i) => matrix.slice(i * 4, i * 4 + 4)
    .reduce((s, x, j) => s + x * glPoint[j], 0));
  near((clip[0] / clip[3] + 1) * 640 / 2 - 0.5, pixels[0]);
  near((1 - clip[1] / clip[3]) * 480 / 2 - 0.5, pixels[1]);
  const rotation = cameraRotationThree(profile.rotation_optical_to_body);
  assert.deepEqual(matVec(rotation, [0, 0, -1]), [0, -1, 0]);
  assert.deepEqual(matVec(rotation, [0, 1, 0]), [1, 0, 0]);
});

test('Brown-Conrady inversion recovers radial and tangential distortion and rejects folds', () => {
  const d = [-0.2, 0.025, 0.003, -0.002, 0.001];
  for (const [x, y] of [[0, 0], [0.6, 0.45], [-0.57, 0.4], [0.3, -0.4]]) {
    const distorted = distortPoint(x, y, d);
    const inverse = undistortPoint(...distorted, d);
    assert.ok(inverse); near(inverse[0], x, 1e-8); near(inverse[1], y, 1e-8);
  }
  assert.equal(undistortPoint(1, 1, [-1, 0, 0, 0, 0]), null);
});

test('expanded lens map covers barrel-distorted edges and uses bottom-up GPU rows', () => {
  const small = { ...profile, width: 64, height: 48 };
  const plain = buildLensMap(small);
  const barrel = buildLensMap({ ...small, D: [-0.2, 0.025, 0.003, -0.002, 0.001] });
  assert.equal(barrel.validPixelFraction, 1);
  assert.ok(barrel.sourceWidth > plain.sourceWidth);
  assert.ok(barrel.sourceHeight > plain.sourceHeight);
  for (let i = 0; i < barrel.data.length; i += 4) {
    assert.ok(barrel.data[i] > 0 && barrel.data[i] < 1);
    assert.ok(barrel.data[i + 1] > 0 && barrel.data[i + 1] < 1);
  }
  assert.ok(plain.data[1] < plain.data[(47 * 64) * 4 + 1]);
});

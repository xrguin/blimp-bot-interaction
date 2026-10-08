// Camera coordinates: optical x right, y down, z forward; robot/world are NED.
export function effectiveIntrinsics(profile) {
  const k = profile.K.flat();
  const sx = profile.width / profile.reference_width;
  const sy = profile.height / profile.reference_height;
  if (k.length !== 9 || !k.every(Number.isFinite) || Math.abs(sx - sy) > 1e-10) {
    throw new Error('Camera intrinsics require a finite 3 × 3 K and matching image aspect ratio.');
  }
  return { fx: k[0] * sx, fy: k[4] * sy,
    cx: (k[2] + 0.5) * sx - 0.5, cy: (k[5] + 0.5) * sy - 0.5 };
}

export function distortPoint(x, y, d) {
  const [k1, k2, p1, p2, k3] = d;
  const r2 = x * x + y * y;
  const radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
  return [x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
    y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y];
}

// Damped Newton inversion, rejecting singular/folded mappings and nonconvergence.
export function undistortPoint(xd, yd, d) {
  const [k1, k2, p1, p2, k3] = d;
  let x = xd, y = yd;
  for (let i = 0; i < 30; i += 1) {
    const [u, v] = distortPoint(x, y, d);
    const ex = u - xd, ey = v - yd;
    const err = ex * ex + ey * ey;
    const r2 = x * x + y * y;
    const radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
    const slope = k1 + r2 * (2 * k2 + 3 * r2 * k3);
    const a = radial + 2 * x * x * slope + 2 * p1 * y + 6 * p2 * x;
    const b = 2 * x * y * slope + 2 * p1 * x + 2 * p2 * y;
    const c = radial + 2 * y * y * slope + 6 * p1 * y + 2 * p2 * x;
    const det = a * c - b * b;
    if (!Number.isFinite(det) || det <= 1e-10 || a <= 0 || c <= 0 || r2 > 100) return null;
    if (err < 1e-20) return [x, y];
    const dx = (c * ex - b * ey) / det;
    const dy = (a * ey - b * ex) / det;
    let scale = 1, accepted = false;
    for (let j = 0; j < 12; j += 1) {
      const nx = x - scale * dx, ny = y - scale * dy;
      const [nu, nv] = distortPoint(nx, ny, d);
      if ((nu - xd) ** 2 + (nv - yd) ** 2 < err) {
        x = nx; y = ny; accepted = true; break;
      }
      scale *= 0.5;
    }
    if (!accepted) return null;
  }
  return null;
}

export function rotationBodyToNed([phi, theta, psi]) {
  const cp = Math.cos(phi), sp = Math.sin(phi), ct = Math.cos(theta), st = Math.sin(theta);
  const cs = Math.cos(psi), ss = Math.sin(psi);
  return [[cs * ct, cs * st * sp - ss * cp, cs * st * cp + ss * sp],
    [ss * ct, ss * st * sp + cs * cp, ss * st * cp - cs * sp],
    [-st, ct * sp, ct * cp]];
}

export function matVec(m, v) { return m.map((row) => row.reduce((s, x, i) => s + x * v[i], 0)); }
export function matMul(a, b) {
  return a.map((row) => b[0].map((_, j) => row.reduce((s, x, k) => s + x * b[k][j], 0)));
}

export function cameraPose(eta, profile) {
  const body = rotationBodyToNed(eta.slice(3, 6));
  const offset = matVec(body, profile.translation_body_m);
  return { position_ned_m: eta.slice(0, 3).map((x, i) => x + offset[i]),
    rotation_optical_to_ned: matMul(body, profile.rotation_optical_to_body) };
}

export function projectOptical(point, intrinsics, distortion = [0, 0, 0, 0, 0]) {
  if (point[2] <= 0) return null;
  const [x, y] = distortPoint(point[0] / point[2], point[1] / point[2], distortion);
  return [intrinsics.fx * x + intrinsics.cx, intrinsics.fy * y + intrinsics.cy];
}

// THREE's camera looks along -z with +y up; the optical convention is +z/+y down.
export function cameraRotationThree(opticalToNed) {
  return matMul(matMul([[1, 0, 0], [0, 0, -1], [0, 1, 0]], opticalToNed),
    [[1, 0, 0], [0, -1, 0], [0, 0, -1]]);
}

export function projectionElements({ fx, fy, cx, cy }, width, height, near, far) {
  return [2 * fx / width, 0, 1 - 2 * (cx + 0.5) / width, 0,
    0, 2 * fy / height, 2 * (cy + 0.5) / height - 1, 0,
    0, 0, -(far + near) / (far - near), -2 * far * near / (far - near),
    0, 0, -1, 0];
}

export function buildLensMap(profile, maxTextureSize = 4096) {
  const { width, height } = profile;
  const k = effectiveIntrinsics(profile);
  const d = profile.D;
  if (![width, height].every((x) => Number.isInteger(x) && x > 0 && x <= maxTextureSize)
      || ![k.fx, k.fy].every((x) => Number.isFinite(x) && x > 0)
      || !Array.isArray(d) || d.length !== 5 || !d.every(Number.isFinite)) {
    throw new Error('Invalid camera image size, focal length, or distortion coefficients.');
  }
  const map = new Float32Array(width * height * 4);
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity, valid = 0;
  for (let row = 0; row < height; row += 1) {
    const v = height - 1 - row; // Texture rows start at the bottom; image pixels start at the top.
    for (let u = 0; u < width; u += 1) {
      const index = (row * width + u) * 4;
      const xy = undistortPoint((u - k.cx) / k.fx, (v - k.cy) / k.fy, d);
      if (!xy) continue;
      const x = xy[0] * k.fx + k.cx, y = xy[1] * k.fy + k.cy;
      map[index] = x; map[index + 1] = y; map[index + 2] = 1; map[index + 3] = 1;
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y); valid += 1;
    }
  }
  if (!valid) throw new Error('This lens calibration has no valid image pixels.');
  // Padding includes a full bilinear footprint, even for barrel-distorted image corners.
  const left = Math.floor(minX) - 2, top = Math.floor(minY) - 2;
  const sourceWidth = Math.ceil(maxX) - left + 3, sourceHeight = Math.ceil(maxY) - top + 3;
  if (sourceWidth > maxTextureSize || sourceHeight > maxTextureSize) {
    throw new Error('This lens calibration requires an unsupported field of view.');
  }
  for (let i = 0; i < map.length; i += 4) {
    map[i] = (map[i] - left + 0.5) / sourceWidth;
    map[i + 1] = 1 - (map[i + 1] - top + 0.5) / sourceHeight;
  }
  return { data: map, width, height, sourceWidth, sourceHeight, intrinsics: k,
    sourceIntrinsics: { ...k, cx: k.cx - left, cy: k.cy - top }, validPixels: valid,
    validPixelFraction: valid / (width * height) };
}

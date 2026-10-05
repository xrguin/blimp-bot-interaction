import * as THREE from './vendor/three/three.module.js';
import { buildLensMap, cameraPose, cameraRotationThree, projectionElements } from './camera-math.js';

// A calibrated pinhole camera plus OpenCV Brown-Conrady distortion. The inverse
// lookup is calculated once per profile, then used for an inexpensive GPU warp.
export class SensorCamera {
  constructor(sceneView, canvas) {
    this.sceneView = sceneView;
    this.canvas = canvas;
    this.context = canvas.getContext('2d', { alpha: false });
    if (!this.context) throw new Error('Camera preview requires a 2D canvas.');
    this.camera = new THREE.PerspectiveCamera();
    this.warpScene = new THREE.Scene();
    this.warpCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this.warpMaterial = new THREE.ShaderMaterial({
      uniforms: { sourceImage: { value: null }, inverseLens: { value: null } },
      vertexShader: `varying vec2 sampleUv;
        void main() { sampleUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }`,
      fragmentShader: `uniform sampler2D sourceImage;
        uniform sampler2D inverseLens;
        varying vec2 sampleUv;
        void main() {
          vec4 mapping = texture2D(inverseLens, sampleUv);
          gl_FragColor = mapping.b > 0.5 ? texture2D(sourceImage, mapping.rg) : vec4(0.0,0.0,0.0,1.0);
          #include <colorspace_fragment>
        }`,
      depthTest: false, depthWrite: false, toneMapped: false,
    });
    this.quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), this.warpMaterial);
    this.quad.frustumCulled = false;
    this.warpScene.add(this.quad);
  }

  configure(profile) {
    const renderer = this.sceneView.renderer;
    if (!renderer) throw new Error('The 3D scene must be initialized before its camera.');
    const map = buildLensMap(profile, Math.min(renderer.capabilities.maxTextureSize, 4096));
    const texture = new THREE.DataTexture(map.data, map.width, map.height, THREE.RGBAFormat, THREE.FloatType);
    texture.minFilter = THREE.NearestFilter; texture.magFilter = THREE.NearestFilter;
    texture.generateMipmaps = false; texture.needsUpdate = true;
    const targetOptions = { minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter,
      format: THREE.RGBAFormat, type: THREE.UnsignedByteType, colorSpace: THREE.SRGBColorSpace,
      depthBuffer: true, stencilBuffer: false };
    const sourceTarget = new THREE.WebGLRenderTarget(map.sourceWidth, map.sourceHeight, targetOptions);
    sourceTarget.samples = Math.min(4, renderer.capabilities.maxSamples);
    const outputTarget = new THREE.WebGLRenderTarget(map.width, map.height, { ...targetOptions, depthBuffer: false });
    this._disposeTargets();
    this.profile = JSON.parse(JSON.stringify(profile));
    this.map = map;
    this.lookup = texture;
    this.sourceTarget = sourceTarget;
    this.outputTarget = outputTarget;
    this.warpMaterial.uniforms.sourceImage.value = sourceTarget.texture;
    this.warpMaterial.uniforms.inverseLens.value = texture;
    this.canvas.width = map.width; this.canvas.height = map.height;
    this.pixels = new Uint8Array(map.width * map.height * 4);
    this.imageData = this.context.createImageData(map.width, map.height);
    this.camera.near = profile.near_m;
    this.camera.far = profile.far_m;
    this.camera.projectionMatrix.set(...projectionElements(map.sourceIntrinsics, map.sourceWidth,
      map.sourceHeight, profile.near_m, profile.far_m));
    this.camera.projectionMatrixInverse.copy(this.camera.projectionMatrix).invert();
    return { valid_pixel_fraction: map.validPixelFraction, width: map.width, height: map.height };
  }

  render(state, config) {
    if (!this.sceneView.ready || !this.profile || !state?.eta) return null;
    const pose = state.camera?.position_ned_m && state.camera?.rotation_optical_to_ned
      ? state.camera : cameraPose(state.eta, this.profile);
    const p = pose.position_ned_m;
    this.camera.position.set(p[0], -p[2], p[1]);
    const r = cameraRotationThree(pose.rotation_optical_to_ned);
    this.camera.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(
      ...r[0], 0, ...r[1], 0, ...r[2], 0, 0, 0, 0, 1,
    ));
    this.camera.updateMatrixWorld(true);
    this.sceneView.update(state, config, { appendTrails: false });
    const renderer = this.sceneView.renderer;
    const oldTarget = renderer.getRenderTarget();
    const oldCubeFace = renderer.getActiveCubeFace();
    const oldMipmapLevel = renderer.getActiveMipmapLevel();
    const oldViewport = renderer.getViewport(new THREE.Vector4());
    const oldScissor = renderer.getScissor(new THREE.Vector4());
    const oldScissorTest = renderer.getScissorTest();
    try {
      renderer.setScissorTest(false);
      this.sceneView.withSensorScene(() => {
        renderer.setRenderTarget(this.sourceTarget);
        renderer.clear();
        renderer.render(this.sceneView.scene, this.camera);
      });
      renderer.setRenderTarget(this.outputTarget);
      renderer.clear();
      renderer.render(this.warpScene, this.warpCamera);
      renderer.readRenderTargetPixels(this.outputTarget, 0, 0, this.map.width, this.map.height, this.pixels);
    } finally {
      renderer.setRenderTarget(oldTarget, oldCubeFace, oldMipmapLevel);
      renderer.setViewport(oldViewport);
      renderer.setScissor(oldScissor);
      renderer.setScissorTest(oldScissorTest);
    }
    const stride = this.map.width * 4;
    for (let row = 0; row < this.map.height; row += 1) {
      const offset = (this.map.height - 1 - row) * stride;
      this.imageData.data.set(this.pixels.subarray(offset, offset + stride), row * stride);
    }
    this.context.putImageData(this.imageData, 0, 0);
    return { position_ned_m: [...pose.position_ned_m],
      rotation_optical_to_ned: pose.rotation_optical_to_ned.map((row) => [...row]),
      K_pixels: [[this.map.intrinsics.fx, 0, this.map.intrinsics.cx],
        [0, this.map.intrinsics.fy, this.map.intrinsics.cy], [0, 0, 1]],
      valid_pixel_fraction: this.map.validPixelFraction,
      width: this.map.width, height: this.map.height };
  }

  capturePNG() {
    return new Promise((resolve, reject) => this.canvas.toBlob(
      (blob) => blob ? resolve(blob) : reject(new Error('Could not encode the camera image.')), 'image/png',
    ));
  }

  _disposeTargets() {
    this.sourceTarget?.dispose(); this.outputTarget?.dispose(); this.lookup?.dispose();
  }

  dispose() {
    this._disposeTargets();
    this.quad.geometry.dispose(); this.warpMaterial.dispose();
    this.profile = null;
  }
}

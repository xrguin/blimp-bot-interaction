import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

const ROVER_COLORS = [0xff8a52, 0x50d5a1, 0xffc65a, 0xe889c6, 0xa694ff, 0xff6f72, 0x4fc4f4, 0xc6db68];
const MAX_TRAIL_POINTS = 500;

function worldPoint(ned) {
  return new THREE.Vector3(ned[0], -ned[2], ned[1]);
}

function localPoint(ned) {
  return new THREE.Vector3(ned[0], -ned[2], ned[1]);
}

function nedAttitudeMatrix(phi, theta, psi) {
  const cph = Math.cos(phi), sph = Math.sin(phi);
  const cth = Math.cos(theta), sth = Math.sin(theta);
  const cps = Math.cos(psi), sps = Math.sin(psi);
  return new THREE.Matrix4().set(
    cps * cth, -sps * cph + cps * sth * sph, sps * sph + cps * cph * sth, 0,
    sps * cth, cps * cph + sph * sth * sps, -cps * sph + sth * sps * cph, 0,
    -sth, cth * sph, cth * cph, 0,
    0, 0, 0, 1,
  );
}

class RoverView {
  constructor(scene, length, color) {
    this.group = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(length, 0.07, length * 0.48),
      new THREE.MeshStandardMaterial({ color, roughness: 0.68, metalness: 0.08 }),
    );
    body.position.y = 0.055;
    this.group.add(body);

    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(length * 0.16, length * 0.28, 4),
      new THREE.MeshStandardMaterial({ color: 0xf3f8fa, roughness: 0.5 }),
    );
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(length * 0.57, 0.07, 0);
    this.group.add(nose);
    scene.add(this.group);

    this.trailPositions = new Float32Array(MAX_TRAIL_POINTS * 3);
    this.trailGeometry = new THREE.BufferGeometry();
    this.trailGeometry.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
    this.trailGeometry.setDrawRange(0, 0);
    this.trail = new THREE.Line(
      this.trailGeometry,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.56 }),
    );
    this.trail.frustumCulled = false;
    scene.add(this.trail);
    this.points = [];
  }

  update(q, appendTrail) {
    this.group.position.set(q[0], 0.055, q[1]);
    this.group.rotation.set(0, -q[2], 0);
    if (appendTrail) {
      const last = this.points[this.points.length - 1];
      if (!last || Math.hypot(last[0] - q[0], last[1] - q[1]) > 0.004) {
        this.points.push([q[0], q[1]]);
        if (this.points.length > MAX_TRAIL_POINTS) this.points.shift();
        for (let i = 0; i < this.points.length; i += 1) {
          this.trailPositions[i * 3] = this.points[i][0];
          this.trailPositions[i * 3 + 1] = 0.012;
          this.trailPositions[i * 3 + 2] = this.points[i][1];
        }
        this.trailGeometry.attributes.position.needsUpdate = true;
        this.trailGeometry.setDrawRange(0, this.points.length);
      }
    }
  }

  resetTrail() {
    this.points.length = 0;
    this.trailGeometry.setDrawRange(0, 0);
  }

  dispose(scene) {
    scene.remove(this.group, this.trail);
    this.group.traverse((obj) => {
      obj.geometry?.dispose();
      obj.material?.dispose();
    });
    this.trailGeometry.dispose();
    this.trail.material.dispose();
  }
}

export class SimulationScene {
  constructor(canvas, onUnavailable) {
    this.canvas = canvas;
    this.onUnavailable = onUnavailable;
    this.ready = false;
    this.lastGeneration = null;
    this.rovers = [];
    this.thrusterLines = [];
    this.defaultCamera = { position: new THREE.Vector3(5.6, 4.2, 6.3), target: new THREE.Vector3(0, 0.75, 0) };
  }

  initialize(config) {
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: false, preserveDrawingBuffer: true });
    } catch (error) {
      this.onUnavailable?.(error);
      return false;
    }
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x09151d, 1);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(0x09151d, 0.035);
    this.camera = new THREE.PerspectiveCamera(42, 1, 0.03, 120);
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.07;
    this.controls.minDistance = 2;
    this.controls.maxDistance = 18;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.resetCamera();

    this.scene.add(new THREE.HemisphereLight(0xc8efff, 0x18232a, 1.65));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
    keyLight.position.set(-3, 8, 4);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.set(1024, 1024);
    this.scene.add(keyLight);

    const arena = Number(config.arena) || 6;
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(arena, arena),
      new THREE.MeshStandardMaterial({ color: 0x0e2029, roughness: 0.96, metalness: 0 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);
    const grid = new THREE.GridHelper(arena, Math.max(2, Math.round(arena)), 0x45616d, 0x243a44);
    grid.position.y = 0.006;
    grid.material.transparent = true;
    grid.material.opacity = 0.62;
    this.scene.add(grid);

    const circlePts = [];
    for (let i = 0; i < 128; i += 1) {
      const a = i / 128 * Math.PI * 2;
      circlePts.push(new THREE.Vector3(1.5 * Math.cos(a), 0.018, 1.5 * Math.sin(a)));
    }
    const reference = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(circlePts),
      new THREE.LineDashedMaterial({ color: 0x6f8994, dashSize: 0.10, gapSize: 0.08, transparent: true, opacity: 0.65 }),
    );
    reference.computeLineDistances();
    this.referenceCircle = reference;
    this.scene.add(reference);

    this._buildBlimp(config.geometry || {});
    this._ensureRovers(config.n_rovers || 0, config.geometry?.rover_length || 0.28);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.canvas.parentElement);
    this.resize();
    this.ready = true;
    return true;
  }

  _buildBlimp(geometry) {
    this.blimp = new THREE.Group();
    const r = Number(geometry.r_env) || 0.45;
    const h = Number(geometry.h_env) || 0.95;
    const envelope = new THREE.Mesh(
      new THREE.SphereGeometry(1, 28, 18),
      new THREE.MeshPhysicalMaterial({ color: 0x55aaff, roughness: 0.28, metalness: 0.08, transparent: true, opacity: 0.86, clearcoat: 0.35 }),
    );
    envelope.scale.set(r, h / 2, r);
    envelope.castShadow = true;
    this.blimp.add(envelope);

    const rings = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.SphereGeometry(1.008, 14, 9), 18),
      new THREE.LineBasicMaterial({ color: 0x9bd5ff, transparent: true, opacity: 0.22 }),
    );
    rings.scale.copy(envelope.scale);
    this.blimp.add(rings);

    const dVT = Number(geometry.d_VT) || 0.26;
    const gondolaSize = Array.isArray(geometry.gondola_size) ? geometry.gondola_size.map(Number) : [r * 0.82, r * 0.46, 0.10];
    const gondola = new THREE.Mesh(
      new THREE.BoxGeometry(gondolaSize[0], gondolaSize[2], gondolaSize[1]),
      new THREE.MeshStandardMaterial({ color: 0x263640, roughness: 0.56, metalness: 0.28 }),
    );
    gondola.position.y = -dVT;
    gondola.castShadow = true;
    this.blimp.add(gondola);
    const mastGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, -dVT, 0)]);
    this.blimp.add(new THREE.Line(mastGeometry, new THREE.LineBasicMaterial({ color: 0xd4e0e4 })));

    const positions = geometry.thruster_positions || [];
    const axes = geometry.thruster_axes || [];
    const thrusterLength = Number(geometry.thruster_length) || 0.13;
    const thrusterRadius = Number(geometry.thruster_radius) || 0.025;
    for (let i = 0; i < 6; i += 1) {
      const pos = localPoint(positions[i] || [0, 0, dVT]);
      const axis = localPoint(axes[i] || [1, 0, 0]).normalize();
      const holder = new THREE.Group();
      holder.position.copy(pos);
      const motor = new THREE.Mesh(
        new THREE.CylinderGeometry(thrusterRadius, thrusterRadius, thrusterLength, 10),
        new THREE.MeshStandardMaterial({ color: 0x26343b, roughness: 0.45, metalness: 0.55 }),
      );
      motor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
      holder.add(motor);
      const thrustPositions = new Float32Array(6);
      const thrustGeometry = new THREE.BufferGeometry();
      thrustGeometry.setAttribute('position', new THREE.BufferAttribute(thrustPositions, 3));
      const thrustLine = new THREE.Line(thrustGeometry, new THREE.LineBasicMaterial({ color: 0xff6f72 }));
      holder.add(thrustLine);
      this.thrusterLines.push({ line: thrustLine, positions: thrustPositions, axis });
      this.blimp.add(holder);
    }
    this.scene.add(this.blimp);

    this.forceArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.01, 0x45d9d2, 0.12, 0.07);
    this.forwardArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), r * 1.7, 0xe8f8ff, 0.13, 0.07);
    this.scene.add(this.forceArrow, this.forwardArrow);
  }

  _ensureRovers(count, length) {
    while (this.rovers.length < count) {
      this.rovers.push(new RoverView(this.scene, length, ROVER_COLORS[this.rovers.length % ROVER_COLORS.length]));
    }
    while (this.rovers.length > count) this.rovers.pop().dispose(this.scene);
  }

  update(state, config) {
    if (!this.ready || !state?.eta) return;
    const generationChanged = this.lastGeneration !== null && state.generation !== this.lastGeneration;
    this.lastGeneration = state.generation;
    if (generationChanged) this.rovers.forEach((rover) => rover.resetTrail());

    const eta = state.eta;
    this.blimp.position.copy(worldPoint(eta));
    const A = new THREE.Matrix4().set(
      1, 0, 0, 0,
      0, 0, -1, 0,
      0, 1, 0, 0,
      0, 0, 0, 1,
    );
    const rotation = A.clone().multiply(nedAttitudeMatrix(eta[3], eta[4], eta[5])).multiply(A.clone().invert());
    this.blimp.quaternion.setFromRotationMatrix(rotation);

    const maxThrust = Math.max(Math.abs(state.params?.['blimp.T_max'] || 0.1), 1e-9);
    this.thrusterLines.forEach((entry, i) => {
      const normalized = (state.thrust?.[i] || 0) / maxThrust;
      const length = normalized * 0.32;
      entry.positions[0] = 0; entry.positions[1] = 0; entry.positions[2] = 0;
      entry.positions[3] = entry.axis.x * length;
      entry.positions[4] = entry.axis.y * length;
      entry.positions[5] = entry.axis.z * length;
      entry.line.geometry.attributes.position.needsUpdate = true;
      entry.line.material.color.setHex(normalized >= 0 ? 0xff6f72 : 0x55aaff);
    });

    const force = worldPoint(state.force_world || [0, 0, 0]);
    const forceScale = Math.max(0, Number(state.params?.['view.force_scale']) || 1);
    const forceLength = Math.min(force.length() * forceScale, 2.3);
    this.forceArrow.position.copy(this.blimp.position);
    this.forceArrow.setDirection(forceLength > 1e-6 ? force.normalize() : new THREE.Vector3(1, 0, 0));
    this.forceArrow.setLength(Math.max(forceLength, 0.001), Math.min(0.16, forceLength * 0.3), Math.min(0.09, forceLength * 0.18));
    this.forceArrow.visible = forceLength > 0.005;
    this.forwardArrow.position.copy(this.blimp.position);
    const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(this.blimp.quaternion).normalize();
    this.forwardArrow.setDirection(forward);

    const roverStates = state.rover_q || [];
    this._ensureRovers(roverStates.length, config?.geometry?.rover_length || 0.28);
    this.rovers.forEach((rover, i) => rover.update(roverStates[i], !state.paused));

    const radius = state.params?.['task.circle_radius'];
    if (Number.isFinite(radius)) this.referenceCircle.scale.set(radius / 1.5, 1, radius / 1.5);
  }

  resize() {
    if (!this.renderer) return;
    const box = this.canvas.parentElement.getBoundingClientRect();
    const width = Math.max(1, Math.round(box.width));
    const height = Math.max(1, Math.round(box.height));
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  resetCamera() {
    if (!this.camera || !this.controls) return;
    this.camera.position.copy(this.defaultCamera.position);
    this.controls.target.copy(this.defaultCamera.target);
    this.controls.update();
  }

  render() {
    if (!this.ready) return;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  downloadPng() {
    if (!this.ready) return;
    this.render();
    const link = document.createElement('a');
    link.download = `blimp-scene-${new Date().toISOString().replaceAll(':', '-')}.png`;
    link.href = this.renderer.domElement.toDataURL('image/png');
    link.click();
  }
}

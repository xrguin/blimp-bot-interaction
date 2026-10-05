import { SimulationScene } from './scene.js';
import { SensorCamera } from './sensor-camera.js';
import { FrameZip } from './zip.js';

const $ = (id) => document.getElementById(id);
const clone = (value) => JSON.parse(JSON.stringify(value));
const nextPaint = () => new Promise((resolve) => setTimeout(resolve, 0));

function saveBlob(blob, name) {
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 60000);
}

function png(canvas) {
  return new Promise((resolve, reject) => canvas.toBlob((blob) => blob ? resolve(blob) : reject(new Error('Could not encode camera frame')), 'image/png'));
}

async function sha256(data) {
  const bytes = typeof data === 'string' ? new TextEncoder().encode(data) : await data.arrayBuffer();
  const digest = await crypto.subtle.digest('SHA-256', bytes);
  return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, '0')).join('');
}

export class CameraPanel {
  constructor(scene, { send, getState, getConfig, hasControl, request = (...args) => fetch(...args) }) {
    this.scene = scene;
    this.send = send;
    this.getState = getState;
    this.getConfig = getConfig;
    this.hasControl = hasControl;
    this.request = request;
    this.sessionVersion = 0;
    this.sensor = new SensorCamera(scene, $('sensor-canvas'));
    this.profile = null;
    this.previewProfile = null;
    this.signature = null;
    this.lastFrame = null;
    this.exporting = false;
    this.lastMetadata = null;
    this.pendingCommands = new Map();
    this.wire();
  }

  error(message = '') { $('sensor-error').textContent = message; }

  resetSession() {
    this.sessionVersion = (this.sessionVersion || 0) + 1;
    this.cancelExport = true;
    this.pendingCommands.clear();
    this.signature = null;
    this.profile = null;
    this.previewProfile = null;
    this.lastMetadata = null;
    this.lastFrame = null;
    this.clearDownload();
    const canvas = $('sensor-canvas');
    const context = canvas.getContext('2d');
    context.fillStyle = '#111';
    context.fillRect(0, 0, canvas.width, canvas.height);
    $('sensor-clock').textContent = 'Waiting for the new flight…';
    $('sensor-export-status').textContent = '';
    $('sensor-retry').hidden = true;
    ['sensor-record', 'sensor-stop', 'sensor-clear', 'sensor-download', 'sensor-snapshot', 'sensor-settings-fields']
      .forEach((id) => { $(id).disabled = true; });
    this.error();
  }

  clearDownload() {
    if (this.downloadUrl) URL.revokeObjectURL(this.downloadUrl);
    this.downloadUrl = null;
    $('sensor-zip-link').hidden = true;
    $('sensor-zip-link').removeAttribute('href');
  }

  command(action, payload = {}) {
    const id = this.send(action, payload);
    if (id) this.pendingCommands.set(id, action);
    else this.error('Camera command could not be sent. Reconnect with control and try again.');
  }

  onMessage(message) {
    const action = this.pendingCommands.get(message.id);
    if (!action || !['ack', 'error'].includes(message.type)) return;
    this.pendingCommands.delete(message.id);
    if (message.type === 'error') {
      if (action === 'camera_config') this.populateSettings();
      this.error(message.message || 'Camera command rejected.');
    }
  }

  populateSettings() {
    const profile = this.profile;
    if (!profile) return;
    $('sensor-resolution').value = `${profile.width}x${profile.height}`;
    $('sensor-fps').value = profile.fps;
    $('sensor-reference-size').textContent = `${profile.reference_width} × ${profile.reference_height} pixels`;
    const values = [profile.K[0][0], profile.K[1][1], profile.K[0][2], profile.K[1][2], ...profile.D];
    ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3'].forEach((key, i) => { $(`sensor-${key}`).value = values[i]; });
    ['x', 'y', 'z'].forEach((key, i) => { $(`sensor-mount-${key}`).value = profile.translation_body_m[i]; });
    $('sensor-calibration').textContent = profile.calibration_status === 'calibrated' ? 'User optics' : 'Estimated optics';
    $('sensor-calibration').className = `badge ${profile.calibration_status === 'calibrated' ? 'controller' : 'paused'}`;
  }

  cameraUnavailable(error) {
    this.previewProfile = null;
    this.lastMetadata = null;
    this.lastFrame = null;
    const context = $('sensor-canvas').getContext('2d');
    context.fillStyle = '#111';
    context.fillRect(0, 0, $('sensor-canvas').width, $('sensor-canvas').height);
    $('sensor-clock').textContent = 'Camera unavailable — adjust settings or retry.';
    $('sensor-retry').hidden = false;
    $('sensor-record').disabled = true;
    $('sensor-snapshot').disabled = true;
    this.error(`Camera view unavailable: ${error.message}`);
  }

  wire() {
    $('sensor-settings').addEventListener('submit', (event) => {
      event.preventDefault();
      try {
        const profile = clone(this.profile);
        [profile.width, profile.height] = $('sensor-resolution').value.split('x').map(Number);
        profile.fps = Number($('sensor-fps').value);
        const values = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3'].map((key) => Number($(`sensor-${key}`).value));
        if (values.some((n) => !Number.isFinite(n))) throw new Error('Enter finite camera values.');
        const nextK = [[values[0], 0, values[2]], [0, values[1], values[3]], [0, 0, 1]];
        const nextD = values.slice(4);
        if (JSON.stringify(nextK) !== JSON.stringify(profile.K) || JSON.stringify(nextD) !== JSON.stringify(profile.D)) {
          profile.calibration_status = 'estimated';
        }
        profile.K = nextK;
        profile.D = nextD;
        profile.translation_body_m = ['x', 'y', 'z'].map((key) => Number($(`sensor-mount-${key}`).value));
        this.error();
        this.command('camera_config', { value: profile });
      } catch (error) { this.error(error.message); }
    });
    $('sensor-import').addEventListener('change', async (event) => {
      const file = event.target.files?.[0];
      if (!file) return;
      try {
        if (file.size > 32768) throw new Error('Camera settings must be a small JSON file (at most 32 KB).');
        const data = JSON.parse(await file.text());
        let profile;
        if (data.schema_version === 'xiao-camera-v1') {
          profile = data;
        } else {
          // Explicit OpenCV-style JSON, with calibration size, K and D.
          if (!data.image_width || !data.image_height || !data.camera_matrix || !data.distortion_coefficients) {
            throw new Error('Use exported camera settings, or image_width, image_height, camera_matrix and distortion_coefficients.');
          }
          profile = { ...clone(this.profile), reference_width: data.image_width, reference_height: data.image_height,
            K: data.camera_matrix, D: data.distortion_coefficients, calibration_status: 'calibrated',
            notes: 'User-supplied intrinsics and distortion. Mounting pose retained from current settings; validate it separately.' };
        }
        this.error();
        this.command('camera_config', { value: profile });
      } catch (error) { this.error(`Import failed: ${error.message}`); }
      event.target.value = '';
    });
    $('sensor-save-settings').addEventListener('click', () => {
      if (!this.profile) return;
      saveBlob(new Blob([JSON.stringify(this.profile, null, 2)], { type: 'application/json' }), 'xiao-ov2640-camera.json');
    });
    $('sensor-snapshot').addEventListener('click', async () => {
      try {
        if (!this.lastMetadata) throw new Error('Wait for a camera frame.');
        const frame = this.lastMetadata;
        saveBlob(await png(this.sensor.canvas), `ov2640-preview-g${frame.generation}-step${frame.control_step}-${frame.t.toFixed(3)}s.png`);
      } catch (error) { this.error(error.message); }
    });
    $('sensor-record').addEventListener('click', () => { this.error(); this.command('camera_record_start'); });
    $('sensor-stop').addEventListener('click', () => this.command('camera_record_stop'));
    $('sensor-clear').addEventListener('click', () => this.command('camera_record_clear'));
    $('sensor-retry').addEventListener('click', () => { this.signature = null; this.onState(this.getState()); });
    $('sensor-download').addEventListener('click', () => this.downloadRecording());
    $('sensor-cancel-export').addEventListener('click', () => { this.cancelExport = true; });
  }

  onState(state) {
    const profile = state.camera?.profile || this.getConfig().camera;
    const signature = JSON.stringify(profile);
    if (signature !== this.signature) {
      // Remember failures too, so a rejected renderer profile is not retried at 20 Hz.
      this.signature = signature;
      this.profile = clone(profile);
      this.lastMetadata = null;
      this.lastFrame = null;
      this.populateSettings();
      try {
        const scale = Math.min(1, 640 / profile.width);
        const preview = { ...profile, width: Math.round(profile.width * scale),
          height: Math.round(profile.height * scale), fps: Math.min(10, profile.fps) };
        this.sensor.configure(preview);
        this.previewProfile = preview;
        $('sensor-retry').hidden = true;
        this.error();
      } catch (error) { this.cameraUnavailable(error); }
    }
    const recording = state.camera_recording || { status: 'empty', frames: 0, seconds: 0 };
    const status = recording.status;
    const active = status === 'recording';
    if (status === 'empty' && !this.exporting) {
      this.clearDownload();
      $('sensor-export-status').textContent = '';
    }
    $('sensor-record-status').textContent = active
      ? `Recording · ${recording.seconds.toFixed(1)} s · ${recording.frames} frames`
      : status === 'ready' ? `${recording.frames} frames ready · ${recording.seconds.toFixed(1)} s${recording.reason ? ` · ${recording.reason}` : ''}`
        : 'Record up to 30 seconds of simulation time.';
    $('sensor-record').disabled = !this.hasControl() || !this.previewProfile || status !== 'empty' || this.exporting;
    $('sensor-stop').disabled = !this.hasControl() || !active;
    $('sensor-clear').disabled = !this.hasControl() || status !== 'ready' || this.exporting;
    $('sensor-download').disabled = status !== 'ready' || this.exporting;
    $('sensor-settings-fields').disabled = !this.hasControl() || active || this.exporting;
    $('sensor-save-settings').disabled = !this.profile;
    $('sensor-snapshot').disabled = !this.lastMetadata;
    $('sensor-ground-note').hidden = state.altitude > 0.05;
  }

  render(state) {
    if (!this.previewProfile || !state?.camera) return;
    const stride = Math.max(1, Math.round(1 / (this.previewProfile.fps * this.getConfig().dt_ctrl)));
    const bucket = Math.floor(state.control_step / stride);
    const key = `${state.generation}:${bucket}:${this.signature}`;
    if (key === this.lastFrame) return;
    try {
      const metadata = this.sensor.render(state, this.getConfig());
      if (!metadata) throw new Error('The scene is not ready.');
      this.lastMetadata = { ...metadata, t: state.t, control_step: state.control_step, generation: state.generation };
      this.lastFrame = key;
      $('sensor-clock').textContent = `${state.t.toFixed(2)} s · ${this.previewProfile.width} × ${this.previewProfile.height} preview · up to ${this.previewProfile.fps} Hz`;
    } catch (error) { this.cameraUnavailable(error); }
  }

  async downloadRecording() {
    if (this.exporting) return;
    this.exporting = true;
    this.cancelExport = false;
    const sessionVersion = this.sessionVersion;
    this.clearDownload();
    $('sensor-cancel-export').hidden = false;
    this.error();
    let scene, sensor;
    try {
      this.onState(this.getState() || {});
      const response = await (this.request || fetch)('/api/camera/recording', { cache: 'no-store' });
      if (sessionVersion !== this.sessionVersion) throw new Error('The flight session changed');
      if (!response.ok) throw new Error((await response.json()).detail || 'Recording is unavailable');
      const recording = await response.json();
      if (this.cancelExport || sessionVersion !== this.sessionVersion) throw new Error('Export cancelled');
      const canvas = document.createElement('canvas');
      const config = { ...this.getConfig(), geometry: recording.geometry, arena: recording.arena,
        n_rovers: recording.frames[0]?.rover_q.length || 0, camera: recording.profile };
      scene = new SimulationScene(canvas, () => {}, { interactive: false });
      if (!scene.initialize(config)) throw new Error('Could not create the recording renderer');
      sensor = new SensorCamera(scene, document.createElement('canvas'));
      sensor.configure(recording.profile);
      const archive = new FrameZip();
      const imageRecords = [];
      for (let i = 0; i < recording.frames.length; i += 1) {
        if (this.cancelExport || sessionVersion !== this.sessionVersion) throw new Error('Export cancelled');
        const frame = recording.frames[i];
        const rendered = sensor.render({ ...frame, generation: recording.generation, paused: true }, config);
        const filename = `frames/${String(i).padStart(6, '0')}.png`;
        const image = await png(sensor.canvas);
        await archive.add(filename, image);
        const nextStep = recording.frames[i + 1]?.control_step;
        const actionIndices = recording.transitions.map((transition, index) => ({ transition, index }))
          .filter(({ transition }) => transition.step >= frame.control_step && transition.step < nextStep).map(({ index }) => index);
        imageRecords.push({ frame_index: i, file: filename, sha256: await sha256(image), t: frame.t,
          relative_t: frame.t - recording.frames[0].t, control_step: frame.control_step,
          relative_control_step: frame.control_step - recording.frames[0].control_step,
          transitions_to_next_frame: actionIndices, camera: rendered });
        if (this.cancelExport || sessionVersion !== this.sessionVersion) throw new Error('Export cancelled');
        $('sensor-export-status').textContent = `Preparing images ${i + 1} / ${recording.frames.length}…`;
        await nextPaint();
      }
      const recordingJson = JSON.stringify(recording);
      await archive.add('recording.json', recordingJson);
      await archive.add('manifest.json', JSON.stringify({ schema_version: 'xiao-image-sequence-v1',
        created_utc: new Date().toISOString(), profile: recording.profile, images: imageRecords,
        state_action_file: 'recording.json', state_action_sha256: await sha256(recordingJson),
        profile_sha256: await sha256(JSON.stringify(recording.profile)), encoding: 'PNG', renderer: 'Three.js 0.186.1',
        timing: 'Images rendered from exact recorded simulation states; transitions contain every applied control interval.',
        limitations: ['Estimated optics until user calibration is imported.', 'Stylized scene; no measured exposure, noise, rolling shutter, motion blur, JPEG pipeline or wireless latency.'],
      }, null, 2));
      if (this.cancelExport || sessionVersion !== this.sessionVersion) throw new Error('Export cancelled');
      this.downloadUrl = URL.createObjectURL(archive.finish());
      const link = $('sensor-zip-link');
      link.href = this.downloadUrl;
      link.download = `ov2640-recording-${new Date().toISOString().replaceAll(':', '-')}.zip`;
      link.hidden = false;
      link.click();
      $('sensor-export-status').textContent = `${imageRecords.length} synchronized images ready. Use Save ZIP if the download did not start. Recording retained until discarded.`;
    } catch (error) {
      if (sessionVersion === this.sessionVersion) {
        this.error(`Export failed: ${error.message}. The recording is retained; you can try again.`);
        $('sensor-export-status').textContent = '';
      }
    } finally {
      sensor?.dispose();
      scene?.dispose();
      this.exporting = false;
      $('sensor-cancel-export').hidden = true;
      if (sessionVersion === this.sessionVersion) this.onState(this.getState() || {});
    }
  }
}

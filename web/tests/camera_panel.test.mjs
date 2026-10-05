import test from 'node:test';
import assert from 'node:assert/strict';
import { CameraPanel } from '../camera-panel.js';

function makeProfile(overrides = {}) {
  return { schema_version: 'xiao-camera-v1', sensor: 'OV2640', calibration_status: 'estimated',
    width: 640, height: 480, fps: 10, reference_width: 640, reference_height: 480,
    K: [[554, 0, 319.5], [0, 554, 239.5], [0, 0, 1]], D: [0, 0, 0, 0, 0],
    translation_body_m: [0, 0, 0.312], rotation_optical_to_body: [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
    near_m: 0.001, far_m: 50, ...overrides };
}

// Exercise real controller methods without WebGL, using a small DOM surface and
// a controllable sensor at the controller's existing collaborator boundary.
function fixture(t, profile = makeProfile()) {
  const oldDocument = globalThis.document;
  const elements = new Map();
  const downloads = [];
  const context = { fills: [], fillRect(...args) { this.fills.push(args); } };
  const element = (id) => {
    if (!elements.has(id)) elements.set(id, {
      id, value: '', textContent: '', className: '', hidden: false, disabled: false,
      width: 640, height: 480, listeners: new Map(),
      addEventListener(name, handler) { this.listeners.set(name, handler); },
      removeAttribute(name) { delete this[name]; },
      dispatch(name, event = {}) { return this.listeners.get(name)?.(event); },
      getContext() { return context; },
      toBlob(callback) { callback(new Blob(['PNG test fixture'], { type: 'image/png' })); },
      click() { downloads.push({ href: this.href, name: this.download }); },
    });
    return elements.get(id);
  };
  globalThis.document = { getElementById: element, createElement: (tag) => element(`created-${tag}-${elements.size}`) };
  t.after(() => { globalThis.document = oldDocument; });
  const f = { element, context, downloads, configured: [], rendered: [], sent: [], control: true,
    config: { camera: profile, dt_ctrl: 0.05 },
    state: { camera: { profile }, generation: 3, control_step: 0, t: 0, altitude: 1,
      camera_recording: { status: 'empty', frames: 0, seconds: 0 } } };
  const sensor = {
    canvas: element('sensor-canvas'),
    configure(value) {
      f.configured.push(structuredClone(value));
      if (f.configureError) throw f.configureError;
    },
    render(state) {
      f.rendered.push(state);
      if (f.renderError) throw f.renderError;
      return f.renderNull ? null : { width: f.panel.previewProfile.width, height: f.panel.previewProfile.height,
        position_ned_m: [0, 0, -1], valid_pixel_fraction: 1 };
    },
  };
  const panel = Object.assign(Object.create(CameraPanel.prototype), {
    scene: {}, sensor, getState: () => f.state, getConfig: () => f.config, hasControl: () => f.control,
    send(action, payload) {
      f.sent.push({ action, payload });
      return f.sendFails ? null : `cmd-${f.sent.length}`;
    },
    profile: null, previewProfile: null, signature: null, lastFrame: null,
    lastMetadata: null, exporting: false, pendingCommands: new Map(),
  });
  f.panel = panel;
  panel.wire();
  return f;
}

test('live preview caps size and rate without altering canonical recording/settings optics', (t) => {
  const profile = makeProfile({ width: 1600, height: 1200, fps: 20 });
  const original = structuredClone(profile);
  const f = fixture(t, profile);
  f.panel.onState(f.state);
  assert.equal(f.configured.length, 1);
  assert.equal(f.configured[0].width, 640);
  assert.equal(f.configured[0].height, 480);
  assert.equal(f.configured[0].fps, 10);
  assert.deepEqual(f.panel.profile, original);
  assert.deepEqual(profile, original);
  assert.equal(f.element('sensor-resolution').value, '1600x1200');
  assert.equal(f.element('sensor-fps').value, 20);
  assert.deepEqual(f.configured[0].K, original.K);
  assert.equal(f.configured[0].reference_width, original.reference_width);
  f.panel.onState(f.state);
  assert.equal(f.configured.length, 1);
});

test('preview preserves a smaller/slower source and only renders once per control-time bucket', (t) => {
  const f = fixture(t, makeProfile({ width: 320, height: 240, fps: 5 }));
  f.panel.onState(f.state);
  assert.equal(f.panel.previewProfile.width, 320);
  assert.equal(f.panel.previewProfile.height, 240);
  assert.equal(f.panel.previewProfile.fps, 5);
  for (let step = 0; step < 8; step += 1) {
    f.panel.render({ ...f.state, control_step: step, t: step * 0.05 });
  }
  assert.deepEqual(f.rendered.map((state) => state.control_step), [0, 4]);
  assert.equal(f.panel.lastMetadata.control_step, 4);
  assert.equal(f.panel.lastMetadata.t, 0.2);
  f.panel.render({ ...f.state, generation: 4 });
  assert.equal(f.rendered.length, 3, 'a new simulation generation must produce a fresh frame');
});

test('failed configuration clears stale frames, caches the failure, and recovers only on retry/change', (t) => {
  const f = fixture(t);
  f.panel.onState(f.state);
  f.panel.render(f.state);
  assert.ok(f.panel.lastMetadata);
  f.configureError = new Error('Invalid lens field of view');
  f.state = { ...f.state, camera: { profile: makeProfile({ D: [-0.9, 0, 0, 0, 0] }) } };
  f.panel.onState(f.state);
  assert.equal(f.panel.previewProfile, null);
  assert.equal(f.panel.lastMetadata, null);
  assert.equal(f.panel.lastFrame, null);
  assert.equal(f.context.fills.length, 1);
  assert.equal(f.element('sensor-retry').hidden, false);
  assert.equal(f.element('sensor-record').disabled, true);
  assert.equal(f.element('sensor-snapshot').disabled, true);
  assert.match(f.element('sensor-error').textContent, /Invalid lens field of view/);
  f.panel.onState(f.state);
  f.panel.render(f.state);
  assert.equal(f.configured.length, 2, 'a bad profile must not trigger repeated configure calls');
  assert.equal(f.rendered.length, 1, 'a failed camera must not render an old sensor profile');
  delete f.configureError;
  f.element('sensor-retry').dispatch('click');
  assert.equal(f.configured.length, 3);
  assert.ok(f.panel.previewProfile);
  assert.equal(f.element('sensor-retry').hidden, true);
  assert.equal(f.element('sensor-error').textContent, '');
  assert.equal(f.element('sensor-snapshot').disabled, true, 'retry alone does not create a frame');
  f.panel.render(f.state);
  f.panel.onState(f.state);
  assert.equal(f.element('sensor-snapshot').disabled, false);
});

test('render errors and a not-ready scene discard the last image metadata and require retry', (t) => {
  const f = fixture(t);
  f.panel.onState(f.state);
  f.panel.render(f.state);
  f.renderError = new Error('GPU context unavailable');
  f.state = { ...f.state, control_step: 2, t: 0.1 };
  f.panel.render(f.state);
  assert.equal(f.panel.lastMetadata, null);
  assert.equal(f.panel.previewProfile, null);
  assert.equal(f.panel.lastFrame, null);
  assert.equal(f.context.fills.length, 1);
  assert.match(f.element('sensor-error').textContent, /GPU context unavailable/);
  f.panel.onState(f.state);
  assert.equal(f.element('sensor-snapshot').disabled, true);
  assert.equal(f.configured.length, 1);
  delete f.renderError;
  f.renderNull = true;
  f.element('sensor-retry').dispatch('click');
  f.panel.render(f.state);
  assert.equal(f.panel.previewProfile, null);
  assert.match(f.element('sensor-error').textContent, /scene is not ready/);
});

test('camera command responses restore rejected settings and clear only the matching pending action', (t) => {
  const f = fixture(t);
  f.panel.onState(f.state);
  f.element('sensor-fx').value = 0;
  f.element('sensor-fps').value = 20;
  f.panel.command('camera_config', { value: makeProfile() });
  f.panel.command('camera_record_start');
  f.panel.onMessage({ type: 'error', id: 'unrelated', message: 'Not this camera command' });
  assert.equal(f.panel.pendingCommands.size, 2);
  f.panel.onMessage({ type: 'error', id: 'cmd-1', message: 'Focal length rejected' });
  assert.equal(f.element('sensor-fx').value, 554);
  assert.equal(f.element('sensor-fps').value, 10);
  assert.equal(f.element('sensor-error').textContent, 'Focal length rejected');
  assert.equal(f.panel.pendingCommands.size, 1);
  f.panel.onMessage({ type: 'ack', id: 'cmd-2' });
  assert.equal(f.panel.pendingCommands.size, 0);
  f.sendFails = true;
  f.panel.command('camera_record_start');
  assert.equal(f.panel.pendingCommands.size, 0);
  assert.match(f.element('sensor-error').textContent, /could not be sent/);
});

test('capped preview timestamps and snapshot filenames identify the rendered frame, not the latest state', async (t) => {
  const f = fixture(t, makeProfile({ width: 1600, height: 1200, fps: 20 }));
  f.panel.onState(f.state);
  f.state = { ...f.state, control_step: 42, t: 2.1 };
  f.panel.render(f.state);
  f.state = { ...f.state, control_step: 43, t: 2.15 };
  f.panel.render(f.state);
  assert.equal(f.rendered.length, 1);
  assert.equal(f.panel.lastMetadata.t, 2.1);
  assert.equal(f.panel.lastMetadata.control_step, 42);
  assert.equal(f.panel.lastMetadata.generation, 3);
  assert.match(f.element('sensor-clock').textContent, /2\.10 s.*640 × 480 preview.*10 Hz/);
  const oldTimeout = globalThis.setTimeout;
  const oldCreate = URL.createObjectURL;
  const oldRevoke = URL.revokeObjectURL;
  let savedBlob;
  globalThis.setTimeout = () => 0; // Suppress the download helper's 60 s URL cleanup timer.
  URL.createObjectURL = (blob) => { savedBlob = blob; return 'blob:camera-test'; };
  URL.revokeObjectURL = () => {};
  try {
    await f.element('sensor-snapshot').dispatch('click');
  } finally {
    globalThis.setTimeout = oldTimeout;
    URL.createObjectURL = oldCreate;
    URL.revokeObjectURL = oldRevoke;
  }
  assert.equal(savedBlob.type, 'image/png');
  assert.equal(f.downloads.length, 1);
  assert.equal(f.downloads[0].name, 'ov2640-preview-g3-step42-2.100s.png');
});

test('changed intrinsics invalidate the prior frame even at the same simulation timestamp', (t) => {
  const f = fixture(t);
  f.panel.onState(f.state);
  f.panel.render(f.state);
  f.state = { ...f.state, camera: { profile: makeProfile({ K: [[560, 0, 300], [0, 550, 220], [0, 0, 1]] }) } };
  f.panel.onState(f.state);
  assert.equal(f.panel.lastMetadata, null);
  assert.equal(f.element('sensor-snapshot').disabled, true);
  f.panel.render(f.state);
  assert.equal(f.configured.length, 2);
  assert.equal(f.rendered.length, 2);
  assert.equal(f.panel.lastMetadata.t, 0);
});

test('discarded recordings release the cached download link instead of leaving an obsolete ZIP', (t) => {
  const f = fixture(t);
  f.panel.onState(f.state);
  f.panel.downloadUrl = 'blob:previous-recording';
  f.element('sensor-zip-link').href = f.panel.downloadUrl;
  f.element('sensor-zip-link').hidden = false;
  f.element('sensor-export-status').textContent = 'A previous recording was exported';
  const revoked = [];
  const oldRevoke = URL.revokeObjectURL;
  URL.revokeObjectURL = (url) => revoked.push(url);
  try { f.panel.onState(f.state); }
  finally { URL.revokeObjectURL = oldRevoke; }
  assert.deepEqual(revoked, ['blob:previous-recording']);
  assert.equal(f.panel.downloadUrl, null);
  assert.equal(f.element('sensor-zip-link').hidden, true);
  assert.equal(f.element('sensor-zip-link').href, undefined);
  assert.equal(f.element('sensor-export-status').textContent, '');
});

test('failed recording retrieval restores the controls and retains the recording for retry', async (t) => {
  const f = fixture(t);
  f.state = { ...f.state, camera_recording: { status: 'ready', frames: 11, seconds: 1 } };
  f.panel.onState(f.state);
  const original = structuredClone(f.state.camera_recording);
  const oldFetch = globalThis.fetch;
  globalThis.fetch = async () => ({ ok: false, json: async () => ({ detail: 'Temporary recording failure' }) });
  try { await f.panel.downloadRecording(); }
  finally { globalThis.fetch = oldFetch; }
  assert.deepEqual(f.state.camera_recording, original);
  assert.equal(f.panel.exporting, false);
  assert.equal(f.element('sensor-cancel-export').hidden, true);
  assert.equal(f.element('sensor-download').disabled, false);
  assert.equal(f.element('sensor-settings-fields').disabled, false);
  assert.match(f.element('sensor-error').textContent, /Temporary recording failure.*recording is retained/);
  assert.equal(f.downloads.length, 0);
});

test('recording exports use the injected private-session request transport', async (t) => {
  const f = fixture(t);
  f.state = { ...f.state, camera_recording: { status: 'ready', frames: 3, seconds: 0.2 } };
  const calls = [];
  f.panel.request = async (...args) => {
    calls.push(args);
    return { ok: false, json: async () => ({ detail: 'Deliberate request test' }) };
  };
  await f.panel.downloadRecording();
  assert.equal(calls.length, 1);
  assert.equal(calls[0][0], '/api/camera/recording');
  assert.match(f.element('sensor-error').textContent, /Deliberate request test/);
});

test('session replacement clears old camera data and cancels an in-flight export response', async (t) => {
  const f = fixture(t);
  f.state = { ...f.state, camera_recording: { status: 'ready', frames: 3, seconds: 0.2 } };
  f.panel.onState(f.state);
  f.panel.render(f.state);
  f.panel.pendingCommands.set('old-command', 'camera_config');
  let complete;
  f.panel.request = () => new Promise((resolve) => { complete = resolve; });
  const exporting = f.panel.downloadRecording();
  f.panel.resetSession();
  assert.equal(f.panel.lastMetadata, null);
  assert.equal(f.panel.previewProfile, null);
  assert.equal(f.panel.pendingCommands.size, 0);
  assert.equal(f.element('sensor-snapshot').disabled, true);
  assert.equal(f.element('sensor-download').disabled, true);
  complete({ ok: true, json: async () => { throw new Error('An old-session body must not be consumed'); } });
  await exporting;
  assert.equal(f.panel.exporting, false);
  assert.equal(f.element('sensor-error').textContent, '');
  assert.equal(f.downloads.length, 0);
});

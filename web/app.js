import { SimulationScene } from './scene.js';

const $ = (id) => document.getElementById(id);
const KEY_SET = new Set(['w', 's', 'a', 'd', 'q', 'e', 'r', 'f']);
const app = {
  config: null,
  state: null,
  scene: null,
  socket: null,
  hasControl: false,
  pressed: new Set(),
  paramBindings: new Map(),
  pendingParams: new Map(),
  pendingUi: { gain: null, pid: null },
  commandId: 0,
  lastUiCommand: null,
  lastSceneSeq: null,
  reconnectTimer: null,
};

function commandId() {
  app.commandId += 1;
  return `web-${Date.now().toString(36)}-${app.commandId}`;
}

function focusedEditor() {
  const target = document.activeElement;
  return Boolean(target && (target.matches('input, select, textarea') || target.isContentEditable));
}

function showCommandError(message = '') {
  $('command-error').textContent = message;
}

function setConnection(status, text) {
  $('connection-dot').className = `status-dot ${status}`;
  $('connection-text').textContent = text;
}

function setControlAvailability(hasControl) {
  app.hasControl = Boolean(hasControl);
  const badge = $('control-badge');
  badge.textContent = hasControl ? 'Controller' : 'Spectator';
  badge.className = `badge ${hasControl ? 'controller' : 'spectator'}`;
  document.querySelectorAll('[data-command]').forEach((element) => { element.disabled = !hasControl; });
  if (!hasControl) clearPressed(false);
}

function send(action, payload = {}) {
  if (!app.hasControl || !app.socket || app.socket.readyState !== WebSocket.OPEN) return false;
  const id = commandId();
  if (action !== 'keys' && action !== 'stop') app.lastUiCommand = id;
  app.socket.send(JSON.stringify({ type: 'command', id, action, ...payload }));
  return true;
}

function clearPressed(sendStop = true, notify = true) {
  app.pressed.clear();
  if (notify && app.hasControl && app.socket?.readyState === WebSocket.OPEN) {
    send('keys', { keys: [] });
    if (sendStop) send('stop');
  }
}

function clearPendingParameters() {
  app.pendingParams.forEach((pending) => clearTimeout(pending.timer));
  app.pendingParams.clear();
  app.pendingUi.gain = null;
  app.pendingUi.pid = null;
  app.lastUiCommand = null;
}

function websocketUrl() {
  const scheme = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${scheme}//${location.host}/ws`;
}

function connect() {
  clearTimeout(app.reconnectTimer);
  clearPressed(false, false);
  clearPendingParameters();
  setControlAvailability(false);
  setConnection('offline', 'Connecting…');
  const socket = new WebSocket(websocketUrl());
  app.socket = socket;

  socket.addEventListener('open', () => setConnection('online', 'Connected'));
  socket.addEventListener('message', (event) => {
    let message;
    try { message = JSON.parse(event.data); } catch { return; }
    if (message.type === 'hello') {
      setControlAvailability(message.has_control);
      showCommandError(message.has_control ? '' : 'Another browser has control. This view is read-only.');
      return;
    }
    if (message.type === 'state') {
      const previous = app.state;
      if (previous?.error && !message.error) showCommandError('');
      if (previous && (message.paused && !previous.paused || message.mode !== previous.mode || message.generation !== previous.generation)) {
        clearPressed(false, false);
      }
      app.state = message;
      updateInterface(message);
      return;
    }
    if (message.type === 'error') showCommandError(message.message || 'The command was rejected.');
    if (message.type === 'ack' && message.id === app.lastUiCommand && !app.state?.error) showCommandError('');
  });
  socket.addEventListener('close', () => {
    if (app.socket !== socket) return;
    clearPressed(false, false);
    clearPendingParameters();
    setControlAvailability(false);
    setConnection('offline', 'Reconnecting…');
    app.reconnectTimer = setTimeout(connect, 900);
  });
  socket.addEventListener('error', () => setConnection('offline', 'Connection error'));
}

function formatNumber(value) {
  if (!Number.isFinite(Number(value))) return '—';
  const n = Number(value);
  if (n !== 0 && (Math.abs(n) < 1e-3 || Math.abs(n) >= 1e4)) return n.toExponential(4).replace(/\.0+(?=e)/, '');
  return Number(n.toPrecision(7)).toString();
}

function validateParameter(binding, raw) {
  if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(raw)) {
    return { ok: false, message: 'Enter a decimal number.' };
  }
  const value = Number(raw);
  if (!Number.isFinite(value)) return { ok: false, message: 'Enter a finite number.' };
  if (value < binding.parameter.min || value > binding.parameter.max) {
    return { ok: false, message: `Use ${formatNumber(binding.parameter.min)} to ${formatNumber(binding.parameter.max)}.` };
  }
  return { ok: true, value };
}

function setBindingError(binding, message = '') {
  binding.number.classList.toggle('invalid', Boolean(message));
  binding.error.textContent = message;
}

function queueParameter(key, value, final = false) {
  const now = performance.now();
  const pending = app.pendingParams.get(key) || { value, lastSent: -Infinity, updated: now, timer: null };
  pending.value = value;
  pending.updated = now;
  const transmit = () => {
    pending.timer = null;
    pending.lastSent = performance.now();
    send('param', { key, value: pending.value });
  };
  if (final) {
    clearTimeout(pending.timer);
    transmit();
  } else if (now - pending.lastSent >= 50) {
    clearTimeout(pending.timer);
    transmit();
  } else if (!pending.timer) {
    pending.timer = setTimeout(transmit, Math.max(0, 50 - (now - pending.lastSent)));
  }
  app.pendingParams.set(key, pending);
}

function commitTextParameter(binding) {
  const result = validateParameter(binding, binding.number.value.trim());
  if (!result.ok) {
    setBindingError(binding, result.message);
    return false;
  }
  setBindingError(binding);
  binding.range.value = result.value;
  binding.number.value = formatNumber(result.value);
  queueParameter(binding.parameter.key, result.value, true);
  return true;
}

function createParameterControl(parameter) {
  const row = document.createElement('div');
  row.className = 'parameter-control';
  const label = document.createElement('label');
  label.className = 'parameter-label';
  label.title = parameter.label;
  label.htmlFor = `param-${parameter.key.replaceAll('.', '-')}`;
  label.textContent = parameter.label;
  if (parameter.unit) {
    const unit = document.createElement('span');
    unit.textContent = ` (${parameter.unit})`;
    label.append(unit);
  }
  if (parameter.description) label.title = parameter.description;
  const range = document.createElement('input');
  range.type = 'range';
  range.min = parameter.min;
  range.max = parameter.max;
  range.step = 'any';
  range.value = parameter.value;
  range.dataset.command = '';
  range.setAttribute('aria-label', parameter.label);
  if (parameter.description) range.title = parameter.description;
  const number = document.createElement('input');
  number.id = label.htmlFor;
  number.className = 'parameter-number';
  number.type = 'text';
  number.inputMode = 'decimal';
  number.autocomplete = 'off';
  number.spellcheck = false;
  number.value = formatNumber(parameter.value);
  number.dataset.command = '';
  if (parameter.description) number.title = parameter.description;
  const error = document.createElement('div');
  error.className = 'parameter-error';
  error.setAttribute('aria-live', 'polite');
  const binding = { parameter, range, number, error };

  range.addEventListener('input', () => {
    binding.dragging = true;
    number.value = formatNumber(Number(range.value));
    setBindingError(binding);
    queueParameter(parameter.key, Number(range.value));
  });
  range.addEventListener('change', () => { binding.dragging = false; queueParameter(parameter.key, Number(range.value), true); });
  range.addEventListener('pointerup', () => { binding.dragging = false; });
  range.addEventListener('blur', () => { binding.dragging = false; });
  number.addEventListener('input', () => setBindingError(binding));
  number.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') {
      event.preventDefault();
      if (commitTextParameter(binding)) { binding.skipBlur = true; number.blur(); }
    }
    if (event.key === 'Escape') {
      event.preventDefault();
      number.value = formatNumber(app.state?.params?.[parameter.key] ?? parameter.value);
      setBindingError(binding);
      binding.skipBlur = true;
      number.blur();
    }
  });
  number.addEventListener('blur', () => {
    if (binding.skipBlur) { binding.skipBlur = false; return; }
    commitTextParameter(binding);
  });
  row.append(label, range, number, error);
  app.paramBindings.set(parameter.key, binding);
  return row;
}

function buildParameters(parameters) {
  const container = $('parameter-groups');
  const groups = new Map();
  parameters.forEach((parameter) => {
    if (!groups.has(parameter.group)) groups.set(parameter.group, []);
    groups.get(parameter.group).push(parameter);
  });
  const groupLabels = { blimp: 'Blimp physics', task: 'Controller and task', view: 'View', mujoco: 'Cameras (MuJoCo)' };
  groups.forEach((items, groupName) => {
    const section = document.createElement('section');
    section.className = 'parameter-group';
    const heading = document.createElement('h3');
    heading.textContent = groupLabels[groupName] || groupName;
    section.append(heading, ...items.map(createParameterControl));
    container.append(section);
  });
  $('parameter-count').textContent = `${parameters.length} controls`;
}

function commitDesiredAltitude() {
  const input = $('desired-altitude');
  const binding = app.paramBindings.get('task.blimp_height');
  if (!binding) return;
  const result = validateParameter(binding, input.value.trim());
  input.parentElement.classList.toggle('invalid', !result.ok);
  $('desired-altitude-error').textContent = result.ok ? '' : result.message;
  if (!result.ok) return;
  input.value = formatNumber(result.value);
  binding.range.value = result.value;
  if (document.activeElement !== binding.number) binding.number.value = formatNumber(result.value);
  queueParameter('task.blimp_height', result.value, true);
}

function setRadio(name, value) {
  const input = document.querySelector(`input[name="${name}"][value="${value}"]`);
  if (input) input.checked = true;
}

function signed(value, digits = 2) {
  const n = Number(value) || 0;
  return `${n >= 0 ? '+' : ''}${n.toFixed(digits)}`;
}

function updateCanonicalParameters(params = {}) {
  Object.entries(params).forEach(([key, value]) => {
    const binding = app.paramBindings.get(key);
    if (!binding || !Number.isFinite(Number(value))) return;
    const pending = app.pendingParams.get(key);
    const matchesPending = pending && Math.abs(Number(value) - pending.value) <= Math.max(1e-12, Math.abs(pending.value) * 1e-10);
    if (matchesPending) {
      clearTimeout(pending.timer);
      app.pendingParams.delete(key);
    }
    const locallyActive = binding.dragging || (pending && performance.now() - pending.updated < 1000);
    if (!locallyActive) binding.range.value = value;
    if (document.activeElement !== binding.number && !binding.number.classList.contains('invalid') && !locallyActive) {
      binding.number.value = formatNumber(value);
      setBindingError(binding);
    }
  });
  const desired = $('desired-altitude');
  const altitudeValue = params['task.blimp_height'];
  if (Number.isFinite(Number(altitudeValue)) && document.activeElement !== desired && !desired.parentElement.classList.contains('invalid') && !app.pendingParams.has('task.blimp_height')) {
    desired.value = formatNumber(altitudeValue);
    desired.parentElement.classList.remove('invalid');
    $('desired-altitude-error').textContent = '';
  }
}

function updateInterface(state) {
  $('sim-time').textContent = `${Number(state.t || 0).toFixed(1)} s`;
  $('run-state').textContent = state.paused ? 'Paused' : 'Running';
  $('run-state').className = `badge ${state.paused ? 'paused' : 'running'}`;
  $('pause-toggle').textContent = state.paused ? 'Run' : 'Pause';
  $('pause-toggle').classList.toggle('running', !state.paused);
  $('pause-toggle').setAttribute('aria-pressed', String(!state.paused));
  if (app.pendingUi.pid && Boolean(state.pid_enabled) === app.pendingUi.pid.value) app.pendingUi.pid = null;
  if (!app.pendingUi.pid || performance.now() - app.pendingUi.pid.updated >= 1000) {
    app.pendingUi.pid = null;
    $('pid-toggle').checked = Boolean(state.pid_enabled);
  }
  setRadio('mode', state.mode);
  setRadio('rovers', state.rovers_mode);
  if (app.pendingUi.gain && Math.abs(Number(state.gain) - app.pendingUi.gain.value) < 1e-10) app.pendingUi.gain = null;
  if ((!app.pendingUi.gain || performance.now() - app.pendingUi.gain.updated >= 1000) && document.activeElement !== $('gain-input')) {
    app.pendingUi.gain = null;
    $('gain-input').value = formatNumber(state.gain ?? 0.4);
  }

  const eta = state.eta || [0, 0, 0, 0, 0, 0];
  const nu = state.nu || [0, 0, 0, 0, 0, 0];
  const command = state.command || [0, 0, 0, 0];
  const force = state.force_world || [0, 0, 0];
  const deg = 180 / Math.PI;
  const roll = eta[3] * deg, pitch = eta[4] * deg, yaw = eta[5] * deg;
  $('altitude-value').textContent = Number(state.altitude ?? 0).toFixed(2);
  $('roll-value').textContent = `${signed(roll, 1)}°`;
  $('pitch-value').textContent = `${signed(pitch, 1)}°`;
  $('yaw-value').textContent = `${signed(yaw, 1)}°`;
  $('attitude-horizon').style.transform = `translateY(${Math.max(-32, Math.min(32, pitch * 0.8))}px) rotate(${-roll}deg)`;
  $('position-value').textContent = `${signed(eta[0])}, ${signed(eta[1])}, ${Number(state.altitude ?? 0).toFixed(2)} m`;
  $('velocity-value').textContent = `${signed(nu[0])}, ${signed(nu[1])}, ${signed(nu[2])} m/s`;
  $('command-value').textContent = command.map((v) => signed(v)).join(', ');
  $('lift-value').textContent = `${signed(state.net_lift_g || 0, 2)} g equiv.`;
  $('force-value').textContent = `${force.map((v) => signed(v, 3)).join(', ')} N`;
  $('realtime-value').textContent = Number.isFinite(Number(state.realtime_factor)) ? `${Number(state.realtime_factor).toFixed(2)}×` : '—';
  updateCanonicalParameters(state.params);
  if (Number.isFinite(Number(state.desired_altitude)) && document.activeElement !== $('desired-altitude') && !$('desired-altitude').parentElement.classList.contains('invalid') && !app.pendingParams.has('task.blimp_height')) {
    $('desired-altitude').value = formatNumber(state.desired_altitude);
  }
  if (state.error) showCommandError(state.error);
}

function adjustGain(delta) {
  if (!app.hasControl) return;
  const current = Number($('gain-input').value || app.pendingUi.gain?.value || app.state?.gain || 0.4);
  const value = Math.max(0.1, Math.min(1, Math.round((current + delta) * 1e6) / 1e6));
  $('gain-input').value = formatNumber(value);
  app.pendingUi.gain = { value, updated: performance.now() };
  send('gain', { value });
}

function commitGain() {
  const input = $('gain-input');
  const value = Number(input.value);
  if (!Number.isFinite(value) || value < 0.1 || value > 1) {
    input.value = formatNumber(app.state?.gain ?? 0.4);
    return;
  }
  input.value = formatNumber(value);
  if (value === (app.pendingUi.gain?.value ?? app.state?.gain)) return;
  app.pendingUi.gain = { value, updated: performance.now() };
  send('gain', { value });
}

function requestPid(value) {
  if (!app.hasControl) return;
  const enabled = Boolean(value);
  $('pid-toggle').checked = enabled;
  app.pendingUi.pid = { value: enabled, updated: performance.now() };
  send('pid', { value: enabled });
}

function wireControls() {
  document.querySelectorAll('input[name="mode"]').forEach((input) => {
    input.dataset.command = '';
    input.addEventListener('change', () => { if (input.checked) { clearPressed(true); send('mode', { value: input.value }); } });
  });
  document.querySelectorAll('input[name="rovers"]').forEach((input) => {
    input.dataset.command = '';
    input.addEventListener('change', () => { if (input.checked) send('rovers', { value: input.value }); });
  });
  ['pid-toggle', 'pause-toggle', 'gain-input', 'gain-down', 'gain-up', 'release-button', 'reset-button', 'desired-altitude'].forEach((id) => { $(id).dataset.command = ''; });
  $('pid-toggle').addEventListener('change', (event) => requestPid(event.target.checked));
  $('pause-toggle').addEventListener('click', () => { clearPressed(true); send('pause', { value: !Boolean(app.state?.paused) }); });
  $('gain-input').addEventListener('change', commitGain);
  $('gain-input').addEventListener('blur', commitGain);
  $('gain-input').addEventListener('keydown', (event) => {
    if (event.key === 'Enter') { event.preventDefault(); commitGain(); event.target.blur(); }
    if (event.key === 'Escape') {
      event.preventDefault();
      event.target.value = formatNumber(app.state?.gain ?? 0.4);
      event.target.blur();
    }
  });
  $('gain-down').addEventListener('click', () => adjustGain(-0.1));
  $('gain-up').addEventListener('click', () => adjustGain(0.1));
  $('release-button').addEventListener('click', () => clearPressed(true));
  $('reset-button').addEventListener('click', () => { clearPressed(false); send('reset'); });
  $('desired-altitude').addEventListener('keydown', (event) => {
    if (event.key === 'Enter') {
      event.preventDefault();
      const before = $('desired-altitude-error').textContent;
      commitDesiredAltitude();
      if (!$('desired-altitude-error').textContent) { event.target.dataset.skipBlur = 'true'; event.target.blur(); }
      else if (!before) event.target.select();
    }
    if (event.key === 'Escape') {
      event.target.value = formatNumber(app.state?.desired_altitude);
      event.target.parentElement.classList.remove('invalid');
      $('desired-altitude-error').textContent = '';
      event.target.dataset.skipBlur = 'true';
      event.target.blur();
    }
  });
  $('desired-altitude').addEventListener('input', () => {
    $('desired-altitude').parentElement.classList.remove('invalid');
    $('desired-altitude-error').textContent = '';
  });
  $('desired-altitude').addEventListener('blur', (event) => {
    if (event.target.dataset.skipBlur === 'true') { delete event.target.dataset.skipBlur; return; }
    commitDesiredAltitude();
  });
  $('camera-reset').addEventListener('click', () => app.scene?.resetCamera());
  $('capture-scene').addEventListener('click', () => app.scene?.downloadPng());
}

function wireKeyboard() {
  window.addEventListener('keydown', (event) => {
    if (event.defaultPrevented || event.ctrlKey || event.metaKey || event.altKey || focusedEditor()
        || event.target.matches?.('input, select, textarea') || event.target.isContentEditable) return;
    const key = event.key.toLowerCase();
    if (KEY_SET.has(key)) {
      event.preventDefault();
      if (!event.repeat) {
        app.pressed.add(key);
        send('keys', { keys: [...app.pressed] });
      }
      return;
    }
    if (event.repeat) return;
    if (key === 'h') { event.preventDefault(); requestPid(!$('pid-toggle').checked); }
    else if (key === ' ') { event.preventDefault(); clearPressed(true); }
    else if (key === 'escape') { event.preventDefault(); clearPressed(true); send('pause', { value: true }); }
    else if (key === 'p') { event.preventDefault(); app.scene?.downloadPng(); }
    else if (key === '+' || key === '=') { event.preventDefault(); adjustGain(0.1); }
    else if (key === '-' || key === '_') { event.preventDefault(); adjustGain(-0.1); }
  });
  window.addEventListener('keyup', (event) => {
    const key = event.key.toLowerCase();
    if (!KEY_SET.has(key)) return;
    app.pressed.delete(key);
    send('keys', { keys: [...app.pressed] });
  });
  window.addEventListener('blur', () => clearPressed(true));
  document.addEventListener('focusin', (event) => {
    if (event.target.matches?.('input, select, textarea') || event.target.isContentEditable) clearPressed(true);
  });
  document.addEventListener('visibilitychange', () => { if (document.hidden) clearPressed(true); });
  setInterval(() => {
    if (app.hasControl && app.state?.mode === 'teleop') send('keys', { keys: [...app.pressed] });
  }, 100);
}

const cameraFeed = { names: [], index: 0, etag: null, objectUrl: null, busy: false, timer: null, failures: 0,
  thumbs: new Map(), thumbCursor: 0, thumbBusy: false, drag: null };

function cameraLabel(name) {
  if (name === 'blimp') return 'Blimp · downward camera';
  const match = /^rover(\d+)$/.exec(name);
  return match ? `Rover ${match[1]} · forward camera` : name;
}

function currentCamera() {
  return cameraFeed.names[cameraFeed.index];
}

async function fetchCameraFrame(name, etag) {
  const headers = etag ? { 'If-None-Match': etag } : {};
  const response = await fetch(`/api/camera/${encodeURIComponent(name)}`, { cache: 'no-store', headers });
  if (response.status === 304 || response.status === 503) return null;   // same frame as before, or not rendered yet
  if (!response.ok) throw new Error(`camera request failed (${response.status})`);
  return { blob: await response.blob(), etag: response.headers.get('ETag'), frameId: response.headers.get('X-Frame-Id'), simTime: response.headers.get('X-Sim-Time') };
}

function showFrame(image, frame, holder) {
  const url = URL.createObjectURL(frame.blob);
  const previous = holder.objectUrl;
  image.onload = () => { if (previous) URL.revokeObjectURL(previous); };
  image.src = url;
  holder.objectUrl = url;
  holder.etag = frame.etag;
}

async function pollCamera() {
  const name = currentCamera();
  if (!name || cameraFeed.busy || !$('camera-live').checked || document.hidden) return;
  cameraFeed.busy = true;
  try {
    const frame = await fetchCameraFrame(name, cameraFeed.etag);
    if (frame) {
      showFrame($('camera-image'), frame, cameraFeed);
      $('camera-caption').textContent = `${cameraLabel(name)} · frame ${frame.frameId ?? '?'} · t = ${frame.simTime ?? '?'} s`;
    }
    cameraFeed.failures = 0;
  } catch (error) {
    cameraFeed.failures += 1;
    if (cameraFeed.failures === 3) $('camera-caption').textContent = 'Camera feed unavailable';
  } finally {
    cameraFeed.busy = false;
  }
  // Thumbnails refresh round-robin, one camera per tick, so every preview stays live at a low rate.
  if (cameraFeed.names.length > 1 && !cameraFeed.thumbBusy) {
    cameraFeed.thumbBusy = true;
    const thumbName = cameraFeed.names[cameraFeed.thumbCursor % cameraFeed.names.length];
    cameraFeed.thumbCursor += 1;
    const holder = cameraFeed.thumbs.get(thumbName);
    try {
      const frame = await fetchCameraFrame(thumbName, holder.etag);
      if (frame) showFrame(holder.image, frame, holder);
    } catch (error) { /* main view reports feed problems */ } finally { cameraFeed.thumbBusy = false; }
  }
}

function selectCamera(index, { focusSlider = false } = {}) {
  const count = cameraFeed.names.length;
  if (!count) return;
  cameraFeed.index = ((index % count) + count) % count;
  cameraFeed.etag = null;                                         // force a fresh frame of the new camera
  const name = currentCamera();
  $('camera-title').textContent = cameraLabel(name);
  $('camera-index').textContent = `${cameraFeed.index + 1} / ${count}`;
  $('camera-caption').textContent = `${cameraLabel(name)} · waiting for frame…`;
  $('camera-slider').value = String(cameraFeed.index);
  for (const [thumbName, holder] of cameraFeed.thumbs) {
    const active = thumbName === name;
    holder.button.classList.toggle('active', active);
    holder.button.setAttribute('aria-selected', String(active));
    if (active) holder.button.scrollIntoView({ block: 'nearest', inline: 'nearest', behavior: 'smooth' });
  }
  if (focusSlider) $('camera-slider').focus();
}

function setupCameraLayout() {
  const row = $('view-row');
  const button = $('camera-layout');
  let sideBySide = true;                                          // default: world view and camera next to each other
  try { const saved = localStorage.getItem('blimp.cameraLayout'); if (saved) sideBySide = saved === 'side'; } catch (error) { /* storage unavailable */ }
  const apply = () => {
    row.classList.toggle('side-by-side', sideBySide);
    button.textContent = sideBySide ? 'Stack below' : 'Side by side';
    button.setAttribute('aria-pressed', String(sideBySide));
  };
  button.addEventListener('click', () => {
    sideBySide = !sideBySide;
    try { localStorage.setItem('blimp.cameraLayout', sideBySide ? 'side' : 'stack'); } catch (error) { /* ignore */ }
    apply();
    button.blur();
  });
  apply();
}

function setupCameras(config) {
  const names = Array.isArray(config.cameras) ? config.cameras : [];
  cameraFeed.names = names;
  const panel = $('camera-panel');
  if (!names.length) {                                           // say why there is no stream instead of hiding silently
    panel.classList.add('unavailable');
    $('camera-unavailable').hidden = false;
    panel.hidden = false;
    return;
  }
  panel.classList.remove('unavailable');
  $('camera-unavailable').hidden = true;
  setupCameraLayout();
  const thumbs = $('camera-thumbs');
  thumbs.replaceChildren(...names.map((name, i) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'camera-thumb';
    button.setAttribute('role', 'tab');
    const image = document.createElement('img');
    image.alt = `${cameraLabel(name)} preview`;
    image.draggable = false;
    const label = document.createElement('span');
    label.textContent = cameraLabel(name);
    button.append(image, label);
    button.addEventListener('click', () => { selectCamera(i); button.blur(); });
    cameraFeed.thumbs.set(name, { button, image, etag: null, objectUrl: null });
    return button;
  }));
  const slider = $('camera-slider');
  slider.max = String(names.length - 1);
  slider.addEventListener('input', () => selectCamera(Number(slider.value)));
  $('camera-prev').addEventListener('click', () => { selectCamera(cameraFeed.index - 1); $('camera-prev').blur(); });
  $('camera-next').addEventListener('click', () => { selectCamera(cameraFeed.index + 1); $('camera-next').blur(); });
  $('camera-live').addEventListener('change', () => { if ($('camera-live').checked) cameraFeed.etag = null; });
  // Drag/swipe on the main view: a horizontal pull of 40 px or more switches camera.
  const frame = $('camera-frame');
  frame.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) return;
    cameraFeed.drag = { x: event.clientX, id: event.pointerId };
    frame.classList.add('dragging');
    frame.setPointerCapture(event.pointerId);
  });
  const endDrag = (event) => {
    if (!cameraFeed.drag || event.pointerId !== cameraFeed.drag.id) return;
    const dx = event.clientX - cameraFeed.drag.x;
    cameraFeed.drag = null;
    frame.classList.remove('dragging');
    if (dx <= -40) selectCamera(cameraFeed.index + 1);
    else if (dx >= 40) selectCamera(cameraFeed.index - 1);
  };
  frame.addEventListener('pointerup', endDrag);
  frame.addEventListener('pointercancel', () => { cameraFeed.drag = null; frame.classList.remove('dragging'); });
  frame.addEventListener('wheel', (event) => {
    if (Math.abs(event.deltaX) > Math.abs(event.deltaY) && Math.abs(event.deltaX) > 20) {
      event.preventDefault();
      selectCamera(cameraFeed.index + (event.deltaX > 0 ? 1 : -1));
    }
  }, { passive: false });
  panel.hidden = false;
  selectCamera(Math.max(0, names.indexOf('blimp')));
  if (cameraFeed.timer) clearInterval(cameraFeed.timer);
  cameraFeed.timer = setInterval(pollCamera, 100);              // main view up to 10 frames/s per tab
}

function renderLoop() {
  if (app.state && app.scene?.ready && app.state.seq !== app.lastSceneSeq) {
    app.scene.update(app.state, app.config);
    app.lastSceneSeq = app.state.seq;
  }
  app.scene?.render();
  requestAnimationFrame(renderLoop);
}

async function initialize() {
  wireControls();
  wireKeyboard();
  setControlAvailability(false);
  let response;
  try {
    response = await fetch('/api/config', { cache: 'no-store' });
    if (!response.ok) throw new Error(`Configuration request failed (${response.status})`);
    app.config = await response.json();
  } catch (error) {
    setConnection('offline', 'Service unavailable');
    showCommandError(error.message);
    $('scene-error').hidden = false;
    $('scene-error').querySelector('span').textContent = 'Start the local service, then reload this page.';
    return;
  }
  buildParameters(app.config.parameters || []);
  setupCameras(app.config);
  try {
    const scene = new SimulationScene($('scene-canvas'), () => { $('scene-error').hidden = false; });
    app.scene = scene;
    scene.initialize(app.config);
    window.__blimpConsole = { get state() { return app.state; }, get config() { return app.config; }, scene };
  } catch (error) {
    $('scene-error').hidden = false;
    $('scene-error').querySelector('span').textContent = 'The controls and telemetry remain available.';
    console.error('3D view initialization failed', error);
  }
  connect();
  requestAnimationFrame(renderLoop);
}

initialize();

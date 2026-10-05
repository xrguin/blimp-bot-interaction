import { SimulationScene } from './scene.js';
import { CameraPanel } from './camera-panel.js';
import { SessionTransport } from './session-transport.js';

const $ = (id) => document.getElementById(id);
const KEY_SET = new Set(['w', 's', 'a', 'd', 'q', 'e', 'r', 'f']);
const app = {
  config: null,
  state: null,
  scene: null,
  sensorPanel: null,
  socket: null,
  hasControl: false,
  pressed: new Set(),
  paramBindings: new Map(),
  pendingParams: new Map(),
  pendingUi: { gain: null, pid: null },
  commandId: 0,
  lastUiCommand: null,
  lastSceneSeq: null,
  transport: null,
  initialized: false,
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
  return id;
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

function disconnected() {
  clearPressed(false, false);
  clearPendingParameters();
  setControlAvailability(false);
  app.sensorPanel?.onState(app.state || {});
}

function receiveMessage(message) {
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
      app.sensorPanel?.onState(message);
      return;
    }
    app.sensorPanel?.onMessage(message);
    if (message.type === 'error') showCommandError(message.message || 'The command was rejected.');
    if (message.type === 'ack' && message.id === app.lastUiCommand && !app.state?.error) showCommandError('');
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
  container.replaceChildren();
  app.paramBindings.clear();
  const groups = new Map();
  parameters.forEach((parameter) => {
    if (!groups.has(parameter.group)) groups.set(parameter.group, []);
    groups.get(parameter.group).push(parameter);
  });
  const groupLabels = { blimp: 'Blimp physics', task: 'Controller and task', view: 'View' };
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
  $('flight-limit').textContent = state.limit_message || '';
  $('flight-limit').hidden = !state.limit_message;
  $('pause-toggle').disabled = !app.hasControl || Boolean(state.limit_message);
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
  $('download-log').addEventListener('click', async (event) => {
    event.preventDefault();
    if (!app.transport) return;
    const button = $('download-log');
    button.disabled = true;
    try {
      const response = await app.transport.request('/api/log.npz');
      if (!response.ok) throw new Error((await response.json()).detail || 'Flight log unavailable.');
      const url = URL.createObjectURL(await response.blob());
      const link = document.createElement('a');
      link.href = url;
      link.download = `blimp-flight-${new Date().toISOString().replaceAll(':', '-')}.npz`;
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 60000);
    } catch (error) { showCommandError(error.message); }
    finally { button.disabled = false; }
  });
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

function renderLoop() {
  if (app.state && app.scene?.ready && app.state.seq !== app.lastSceneSeq) {
    app.scene.update(app.state, app.config);
    app.lastSceneSeq = app.state.seq;
  }
  app.scene?.render();
  if (app.state) app.sensorPanel?.render(app.state);
  requestAnimationFrame(renderLoop);
}

function sessionReady(session) {
  $('start-new-flight').hidden = true;
  $('session-notice').hidden = true;
  $('session-notice').textContent = '';
  app.config = session.config;
  app.state = null;
  app.lastSceneSeq = null;
  disconnected();
  $('session-mode').textContent = session.mode === 'isolated' ? 'Online private flight' : 'Local simulation';
  const limits = app.config.public_limits;
  if (session.mode === 'isolated' && limits) {
    const end = new Date(session.expires_at);
    const endText = Number.isFinite(end.getTime()) ? ` Ends by ${end.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}.` : '';
    $('session-details').textContent = `Each tab has its own flight.${endText} Inactive flights end after ${Math.round(limits.idle_seconds / 60)} minutes; disconnected flights after ${limits.disconnected_grace_seconds} seconds. Logs stop after ${Math.round(limits.max_log_steps * app.config.dt_ctrl / 60)} simulated minutes and require Reset. Download data before leaving.`;
  } else $('session-details').textContent = 'Runs on this computer. Other tabs share this flight. Download data before stopping the server.';
  $('flight-limit').hidden = true;
  buildParameters(app.config.parameters || []);
  setControlAvailability(false);
  if (app.initialized) {
    app.sensorPanel?.resetSession();
    app.scene?.rovers.forEach((rover) => rover.resetTrail());
    app.scene?.resetCamera();
    return;
  }
  app.initialized = true;
  try {
    const scene = new SimulationScene($('scene-canvas'), () => { $('scene-error').hidden = false; });
    app.scene = scene;
    scene.initialize(app.config);
    app.sensorPanel = new CameraPanel(scene, { send, getState: () => app.state,
      getConfig: () => app.config, hasControl: () => app.hasControl,
      request: (path, options) => app.transport.request(path, options) });
    window.__blimpConsole = { get state() { return app.state; }, get config() { return app.config; }, scene };
  } catch (error) {
    $('scene-error').hidden = false;
    $('scene-error').querySelector('span').textContent = 'The controls and telemetry remain available.';
    console.error('3D view initialization failed', error);
  }
  requestAnimationFrame(renderLoop);
}

function initialize() {
  wireControls();
  wireKeyboard();
  setControlAvailability(false);
  app.transport = new SessionTransport({ onSession: sessionReady, onStatus: setConnection,
    onSocket: (socket) => { app.socket = socket; }, onMessage: receiveMessage, onDisconnect: disconnected,
    onEnded: (message) => {
      app.state = null;
      app.lastSceneSeq = null;
      app.sensorPanel?.resetSession();
      $('run-state').textContent = 'Flight ended';
      $('run-state').className = 'badge paused';
      $('pause-toggle').textContent = 'Run';
      $('pause-toggle').classList.remove('running');
      $('pause-toggle').setAttribute('aria-pressed', 'false');
      $('session-notice').textContent = message;
      $('session-notice').hidden = false;
      $('start-new-flight').textContent = 'Start new flight';
      $('start-new-flight').hidden = false;
    },
    onCapacity: () => {
      $('start-new-flight').textContent = 'Retry start';
      $('start-new-flight').hidden = false;
    } });
  $('start-new-flight').addEventListener('click', () => {
    $('start-new-flight').hidden = true;
    app.transport.start();
  });
  window.addEventListener('pagehide', () => app.transport.stop());
  window.addEventListener('pageshow', (event) => { if (event.persisted) app.transport.start(); });
  app.transport.start();
}

initialize();

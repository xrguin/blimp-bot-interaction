import test from 'node:test';
import assert from 'node:assert/strict';
import { SessionTransport } from '../session-transport.js';

const turn = () => new Promise((resolve) => setImmediate(resolve));
const response = (body, status = 200, retry = null) => ({ ok: status < 400, status,
  headers: new Headers(retry ? { 'Retry-After': retry } : {}), json: async () => body });
const session = (token = 'private-a') => ({ mode: 'isolated', token,
  config: { dt_ctrl: 0.05, camera: {} }, expires_at: '2026-10-05T12:30:00Z' });

function harness(t, replies) {
  const f = { calls: [], sockets: [], timers: new Map(), sessions: [], messages: [], ended: [], status: [], disconnects: 0 };
  class Socket {
    constructor(url) { this.url = url; this.listeners = new Map(); this.sent = []; f.sockets.push(this); }
    addEventListener(name, handler) { this.listeners.set(name, handler); }
    emit(name, payload = {}) { this.listeners.get(name)?.(payload); }
    send(data) { this.sent.push(JSON.parse(data)); }
    close() { this.emit('close', { code: 1000 }); }
  }
  let nextTimer = 1;
  f.transport = new SessionTransport({ location: { protocol: 'https:', host: 'sim.example' }, WebSocketImpl: Socket,
    fetchImpl: async (path, options) => {
      f.calls.push({ path, options });
      const next = replies.shift();
      if (next instanceof Error) throw next;
      return typeof next === 'function' ? next() : next;
    },
    schedule: (fn, delay) => { const id = nextTimer++; f.timers.set(id, { fn, delay }); return id; },
    cancel: (id) => f.timers.delete(id),
    onSession: (value) => f.sessions.push(value), onMessage: (value) => f.messages.push(value),
    onEnded: (value) => f.ended.push(value), onStatus: (...value) => f.status.push(value),
    onDisconnect: () => { f.disconnects += 1; },
  });
  f.tick = () => {
    const [id, timer] = f.timers.entries().next().value;
    f.timers.delete(id);
    timer.fn();
    return timer.delay;
  };
  t.after(() => f.transport.stop());
  return f;
}

test('public token is sent only in attach/header; reconnect reuses the same flight', async (t) => {
  const f = harness(t, [response(session()), response({ data: true })]);
  f.transport.start(); await turn();
  assert.equal(f.calls[0].path, '/api/session');
  assert.equal(f.calls[0].options.method, 'POST');
  assert.equal(f.calls[0].options.credentials, 'omit');
  assert.equal(f.sockets[0].url, 'wss://sim.example/ws');
  assert.equal('token' in f.sessions[0], false, 'UI callbacks must not receive credentials');
  f.sockets[0].emit('open');
  assert.deepEqual(f.sockets[0].sent, [{ type: 'attach', token: 'private-a' }]);
  await f.transport.request('/api/camera/recording');
  assert.equal(f.calls[1].options.headers.get('Authorization'), 'Bearer private-a');
  assert.equal(f.calls[1].options.redirect, 'error');
  assert.equal(f.calls[1].path.includes('private-a'), false);
  f.sockets[0].emit('close', { code: 1006 });
  assert.equal(f.tick(), 1000);
  f.sockets[1].emit('open');
  assert.deepEqual(f.sockets[1].sent, [{ type: 'attach', token: 'private-a' }]);
  assert.equal(f.calls.length, 2, 'ordinary reconnect must not create a replacement flight');
});

test('local shared mode retains the original no-attach socket and unauthenticated export', async (t) => {
  const f = harness(t, [response({ ...session(null), mode: 'shared', expires_at: null }), response({})]);
  f.transport.start(); await turn();
  f.sockets[0].emit('open');
  assert.deepEqual(f.sockets[0].sent, []);
  f.sockets[0].emit('message', { data: JSON.stringify({ type: 'hello', has_control: false }) });
  assert.deepEqual(f.messages, [{ type: 'hello', has_control: false }]);
  await f.transport.request('/api/log.npz');
  assert.equal(f.calls[1].options.headers.has('Authorization'), false);
});

test('separate page transports create distinct flights without shared browser storage', async (t) => {
  const a = harness(t, [response(session('visitor-a')), response({})]);
  const b = harness(t, [response(session('visitor-b')), response({})]);
  a.transport.start(); b.transport.start(); await turn();
  a.sockets[0].emit('open'); b.sockets[0].emit('open');
  assert.equal(a.sockets[0].sent[0].token, 'visitor-a');
  assert.equal(b.sockets[0].sent[0].token, 'visitor-b');
  await a.transport.request('/api/log.npz'); await b.transport.request('/api/log.npz');
  assert.equal(a.calls[1].options.headers.get('Authorization'), 'Bearer visitor-a');
  assert.equal(b.calls[1].options.headers.get('Authorization'), 'Bearer visitor-b');
});

test('capacity exhaustion displays a friendly message and respects retry backoff', async (t) => {
  const f = harness(t, [response({ detail: 'Full' }, 503, '7'), response(session())]);
  f.transport.start(); await turn();
  assert.match(f.status.at(-1)[1], /slots are busy.*Retrying/);
  assert.equal(f.sockets.length, 0);
  assert.equal(f.tick(), 7000);
  await turn();
  assert.equal(f.calls.length, 2);
  assert.equal(f.sockets.length, 1);
});

test('expired socket clears credentials and waits for an explicit start before allocating another flight', async (t) => {
  const f = harness(t, [response(session('old')), response(session('replacement'))]);
  f.transport.start(); await turn();
  const old = f.sockets[0];
  old.emit('open');
  old.emit('message', { data: JSON.stringify({ type: 'session_error', code: 'session_expired', message: 'Flight expired.' }) });
  assert.equal(f.ended.length, 1);
  assert.match(f.ended[0], /Flight expired.*Start new flight/);
  old.emit('message', { data: JSON.stringify({ type: 'state', t: 42 }) });
  assert.deepEqual(f.messages, []);
  await assert.rejects(f.transport.request('/api/log.npz'), /No active flight/);
  assert.equal(f.timers.size, 0, 'an expired unattended page must not schedule another allocation');
  assert.equal(f.calls.length, 1);
  f.transport.start(); await turn();
  f.sockets[1].emit('open');
  assert.deepEqual(f.sockets[1].sent, [{ type: 'attach', token: 'replacement' }]);
  assert.equal(f.sessions.length, 2);
});

test('expired download requires an explicit restart and credentials cannot be sent to external URLs', async (t) => {
  const f = harness(t, [response(session()), response({ error: 'session_expired' }, 401)]);
  f.transport.start(); await turn();
  await assert.rejects(f.transport.request('https://other.example/api/log.npz'), /this service/);
  await assert.rejects(f.transport.request('//other.example/api/log.npz'), /this service/);
  assert.equal(f.calls.length, 1);
  await assert.rejects(f.transport.request('/api/log.npz'), /session ended/);
  assert.equal(f.ended.length, 1);
  assert.equal(f.timers.size, 0);
});

test('stop cancels retries and late session creation cannot reconnect a closed page', async (t) => {
  let resolve;
  const f = harness(t, [() => new Promise((done) => { resolve = done; })]);
  f.transport.start();
  f.transport.stop();
  resolve(response(session())); await turn();
  assert.equal(f.sessions.length, 0);
  assert.equal(f.sockets.length, 0);
  assert.equal(f.timers.size, 0);
});

test('expiry close codes wait for restart even when the session error message was lost', async (t) => {
  const f = harness(t, [response(session()), response(session('new'))]);
  f.transport.start(); await turn();
  f.sockets[0].emit('close', { code: 4001 });
  assert.equal(f.ended.length, 1);
  assert.equal(f.timers.size, 0);
  f.transport.start(); await turn();
  f.sockets[1].emit('open');
  assert.equal(f.sockets[1].sent[0].token, 'new');
});

test('default native timer adapters retain the browser global receiver', async () => {
  const oldSet = globalThis.setTimeout;
  const oldClear = globalThis.clearTimeout;
  const calls = [];
  globalThis.setTimeout = function (callback, delay) {
    assert.equal(this, globalThis, 'native window timers must not receive the transport as this');
    calls.push(['set', delay]); return 1;
  };
  globalThis.clearTimeout = function (id) {
    assert.equal(this, globalThis, 'native window timers must not receive the transport as this');
    calls.push(['clear', id]);
  };
  try {
    const transport = new SessionTransport({ fetchImpl: async () => response({}, 503), WebSocketImpl: class {},
      location: { protocol: 'https:', host: 'sim.example' } });
    transport.start(); await turn();
    transport.stop();
    assert.ok(calls.some(([kind]) => kind === 'set'));
    assert.ok(calls.some(([kind]) => kind === 'clear'));
  } finally {
    globalThis.setTimeout = oldSet;
    globalThis.clearTimeout = oldClear;
  }
});

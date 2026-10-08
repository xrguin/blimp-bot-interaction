// Credentials live only in this page's memory. Each new page creates its own flight.
export class SessionTransport {
  #session = null;
  #timer = null;
  #attempt = 0;
  #active = false;
  #creating = false;
  #epoch = 0;

  constructor({ fetchImpl = (...args) => fetch(...args), WebSocketImpl = WebSocket,
    location = globalThis.location,
    schedule = (callback, delay) => globalThis.setTimeout(callback, delay),
    cancel = (id) => globalThis.clearTimeout(id),
    onSession = () => {}, onStatus = () => {}, onMessage = () => {},
    onDisconnect = () => {}, onEnded = () => {}, onSocket = () => {}, onCapacity = () => {} } = {}) {
    Object.assign(this, { fetchImpl, WebSocketImpl, location, schedule, cancel,
      onSession, onStatus, onMessage, onDisconnect, onEnded, onSocket, onCapacity });
    this.socket = null;
  }

  start() {
    this.#active = true;
    this.cancel(this.#timer);
    this.#timer = null;
    if (this.#session) this.#connect();
    else this.#create();
  }

  stop() {
    this.#active = false;
    this.#epoch += 1;
    this.#creating = false;
    this.cancel(this.#timer);
    this.#timer = null;
    this.#session = null;
    this.#detach();
  }

  #detach() {
    const old = this.socket;
    this.socket = null;
    this.onSocket(null);
    this.onDisconnect();
    old?.close();
  }

  #retry(create, minimumMs = 0) {
    if (!this.#active || this.#timer !== null) return;
    const delay = Math.max(minimumMs, Math.min(15000, 1000 * 2 ** Math.min(this.#attempt++, 4)));
    this.#timer = this.schedule(() => {
      this.#timer = null;
      if (create) this.#create(); else this.#connect();
    }, delay);
  }

  async #create() {
    if (!this.#active || this.#creating) return;
    this.#creating = true;
    const epoch = ++this.#epoch;
    this.onStatus('offline', 'Starting flight… The service may need a moment to wake.');
    try {
      const response = await this.fetchImpl('/api/session', { method: 'POST', cache: 'no-store', credentials: 'omit',
        headers: { Accept: 'application/json' } });
      if (!this.#active || epoch !== this.#epoch) return;
      if (!response.ok) {
        const retryAfter = Math.min(60, Math.max(0, Number(response.headers?.get('Retry-After')) || 0));
        const busy = response.status === 503 || response.status === 429;
        this.onStatus('offline', busy ? 'All flight slots are busy. Retrying automatically…' : 'Service unavailable. Retrying automatically…');
        if (busy) this.onCapacity();
        this.#retry(true, retryAfter * 1000);
        return;
      }
      const session = await response.json();
      if (!this.#active || epoch !== this.#epoch) return;
      if (!['isolated', 'shared'].includes(session.mode) || !session.config
          || (session.mode === 'isolated' && (typeof session.token !== 'string' || !session.token))) {
        throw new Error('Invalid flight session response');
      }
      this.#session = session;
      this.#attempt = 0;
      // Do not pass the credential to UI code or debug globals.
      this.onSession({ mode: session.mode, config: session.config, expires_at: session.expires_at });
      this.#connect();
    } catch {
      if (this.#active && epoch === this.#epoch) {
        this.onStatus('offline', 'Could not reach the service. Retrying automatically…');
        this.#retry(true);
      }
    } finally { if (epoch === this.#epoch) this.#creating = false; }
  }

  #end(message) {
    if (!this.#session) return;
    // An unattended expired page must not allocate another runtime indefinitely.
    this.#active = false;
    this.#session = null;
    this.#epoch += 1;
    this.cancel(this.#timer);
    this.#timer = null;
    this.#detach();
    this.onStatus('offline', 'Flight ended. Start a new flight when ready.');
    this.onEnded(message || 'This flight session ended. Unsaved data is no longer available. Select Start new flight to begin again.');
  }

  #connect() {
    if (!this.#active || !this.#session || this.socket) return;
    this.onStatus('offline', 'Connecting…');
    const scheme = this.location.protocol === 'https:' ? 'wss:' : 'ws:';
    let socket;
    try { socket = new this.WebSocketImpl(`${scheme}//${this.location.host}/ws`); }
    catch { this.onStatus('offline', 'Connection unavailable. Retrying…'); this.#retry(false); return; }
    this.socket = socket;
    this.onSocket(socket);
    socket.addEventListener('open', () => {
      if (this.socket !== socket) return;
      if (this.#session.mode === 'isolated') socket.send(JSON.stringify({ type: 'attach', token: this.#session.token }));
      this.onStatus('online', 'Connected');
    });
    socket.addEventListener('message', (event) => {
      if (this.socket !== socket) return;
      let message;
      try { message = JSON.parse(event.data); } catch { return; }
      if (message.type === 'session_error') {
        this.#end(`${message.message || 'This flight session ended.'} Unsaved data is no longer available. Select Start new flight to begin again.`);
        return;
      }
      if (message.type === 'hello') this.#attempt = 0;
      this.onMessage(message);
    });
    socket.addEventListener('close', (event) => {
      if (this.socket !== socket) return;
      if (this.#session?.mode === 'isolated' && [4001, 4002, 1008].includes(event.code)) {
        this.#end(); return;
      }
      this.socket = null;
      this.onSocket(null);
      this.onDisconnect();
      this.onStatus('offline', 'Connection lost. Flight paused; reconnecting…');
      this.#retry(false);
    });
    socket.addEventListener('error', () => {
      if (this.socket === socket) this.onStatus('offline', 'Connection error. Waiting to reconnect…');
    });
  }

  async request(path, options = {}) {
    if (!/^\/api\//.test(path) || path.includes('\\')) throw new Error('Session requests must use this service’s API.');
    const session = this.#session;
    if (!session) throw new Error('No active flight. Wait for a connection and try again.');
    const headers = new Headers(options.headers);
    if (session.mode === 'isolated') headers.set('Authorization', `Bearer ${session.token}`);
    const response = await this.fetchImpl(path, { ...options, headers, credentials: 'omit', cache: 'no-store', redirect: 'error' });
    if (session !== this.#session) throw new Error('The flight changed during this request. Try again in the current flight.');
    if (session.mode === 'isolated' && response.status === 401) {
      this.#end();
      throw new Error('This flight session ended. Its unsaved data is no longer available.');
    }
    return response;
  }
}

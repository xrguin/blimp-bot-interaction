# Publishing the simulator as a website (compute stays on your machine)

Visitors need only a browser: physics (NumPy blimp + MuJoCo rovers) and camera rendering run
on the server, which sends JSON state at 20 Hz and JPEG frames. This guide publishes the local
server through **Cloudflare Tunnel** with a **Cloudflare Access** login, so nothing is installed
on visitors' machines, no router port is opened, and every visitor gets a **private simulation**.
Prerequisite: the server itself installed and verified per [INSTALL.md](INSTALL.md).

## How the server behaves in public mode

- **Private session per visitor.** The first `/api/config` request sets an HttpOnly cookie and
  starts a `SimulationRuntime` (own worker thread, seed, rovers, cameras) for that visitor;
  further tabs of the same visitor join it (first tab controls, the others spectate). Visitors can
  choose their setup on first load: `https://blimp.example.org/?n=6&mode=auto&seed=3`
  (`n` ≤ `--max-rovers`, `mode` teleop|auto, `rovers` circle|idle).
- **Limits.** `--max-sessions` (default 8) concurrent simulations; beyond that the page says
  "server full" and retries every 20 s. Sessions with no open connection for `--idle-timeout`
  seconds (default 600) are stopped and removed; the page then starts a fresh one on reconnect.
  On this class of machine one 4-rover MuJoCo session with five 640×480 cameras costs roughly
  5–8 % of a CPU core plus a little GPU; each viewer polls about 0.5 MB/s of camera JPEG.
- **Network.** The server still binds to `127.0.0.1`. `--public-host blimp.example.org` makes it
  accept that hostname (Host header, `wss://` origin) and mark cookies `Secure` for it.
- **Authentication.** Cloudflare Access (below) authenticates people before any request reaches
  the server. `--access-token SECRET` adds a second lock inside the app: visitors must open the
  page once as `https://blimp.example.org/?token=SECRET` (it is stored in a cookie and removed
  from the address bar); without it the API answers 401 and the page explains what to do.
- `--shared` restores the original behaviour: one simulation for everyone, first tab controls.

## 1. Run the server in public mode (test locally first)

```sh
$ .venv/bin/python web_server.py --no-browser --rovers-backend mujoco \
      --public-host blimp.example.org --max-sessions 8 --idle-timeout 600
Blimp control panel: http://127.0.0.1:8000
Public hostnames accepted: blimp.example.org
Sessions: private per visitor, up to 8, idle timeout 600 s
```

Check from a shell: `curl -s http://127.0.0.1:8000/health` → `{"status":"ok","sessions":0,"max_sessions":8}`.
Opening <http://127.0.0.1:8000> locally still works (loopback is always allowed) and shows a
"Session xxxxxxxx · 1/8" badge in the top bar.

## 2. Cloudflare Tunnel with an Access login (recommended)

You need a domain managed by Cloudflare (the free plan is enough) and a Cloudflare account.

```sh
# install cloudflared (Ubuntu/Debian)
$ sudo mkdir -p --mode=0755 /usr/share/keyrings
$ curl -fsSL https://pkg.cloudflare.com/cloudflare-main.gpg | sudo tee /usr/share/keyrings/cloudflare-main.gpg >/dev/null
$ echo "deb [signed-by=/usr/share/keyrings/cloudflare-main.gpg] https://pkg.cloudflare.com/cloudflared $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/cloudflared.list
$ sudo apt-get update && sudo apt-get install -y cloudflared

# authenticate, create the tunnel, route a hostname to it
$ cloudflared tunnel login                       # opens a browser; pick the domain
$ cloudflared tunnel create blimp                # prints the tunnel id and writes ~/.cloudflared/<id>.json
$ cloudflared tunnel route dns blimp blimp.example.org
$ cp deploy/cloudflared-config.example.yml ~/.cloudflared/config.yml
$ sed -i "s/TUNNEL_ID/<the id>/g; s/USER/$USER/; s/blimp.example.org/<your hostname>/" ~/.cloudflared/config.yml
$ cloudflared tunnel run blimp                   # foreground test; Ctrl+C to stop
```

With the server from step 1 running, <https://blimp.example.org> now serves the page over
HTTPS. Before sharing the link, add the login:

1. Cloudflare dashboard → **Zero Trust** → **Access** → **Applications** → *Add an application*
   → *Self-hosted*. Application domain: `blimp.example.org`. Session duration: e.g. 24 h.
2. Add a policy *Allow* with a rule such as *Emails* = the addresses of your collaborators, or
   *Emails ending in* `@your-university.edu`. The default identity provider is the **one-time
   PIN** sent by e-mail; Google/GitHub login can be enabled under *Settings → Authentication*.
3. Save. Visiting the link now shows Cloudflare's login page first; after login the request is
   forwarded to the tunnel and the simulator loads.

Verify from outside your network (phone on mobile data): login page → simulator → press
**Run**; the camera panel streams; a second device gets its own session badge.

### Keep it running after reboots

```sh
$ mkdir -p ~/.config/systemd/user
$ cp deploy/blimp-web.service deploy/cloudflared.service ~/.config/systemd/user/
$ sed -i "s/blimp.example.org/<your hostname>/" ~/.config/systemd/user/blimp-web.service
$ systemctl --user daemon-reload
$ systemctl --user enable --now blimp-web cloudflared
$ loginctl enable-linger $USER                   # user services run without an open login session
$ systemctl --user status blimp-web cloudflared  # both "active (running)"
```

Logs: `journalctl --user -u blimp-web -f`. Edit the unit's `ExecStart` to change rover
count, session cap or to add `--access-token`. `MUJOCO_GL=egl` in the unit lets the cameras
render without a desktop session; on NVIDIA machines the EGL vendor file is selected
automatically (see INSTALL.md §7).

## 3. Quick public link without a domain (demo only)

```sh
$ cloudflared tunnel --url http://127.0.0.1:8000
```

prints a random `https://<words>.trycloudflare.com` URL. There is no Access login on these
"quick tunnels", so run the server with a token and that hostname:

```sh
$ .venv/bin/python web_server.py --no-browser --rovers-backend mujoco \
      --public-host <words>.trycloudflare.com --access-token "$(openssl rand -hex 16)"
```

and share `https://<words>.trycloudflare.com/?token=<the token>`. The URL changes every time
the quick tunnel restarts, so this is for short demos.

## 4. Alternative: private network with Tailscale

If the audience is a small known group, install Tailscale on your machine and theirs, then
share `http://<this-machine-tailscale-name>:8000` — but the server must then listen on the
Tailscale interface, which it does not do by default (loopback only). Prefer the tunnel, or run
a local reverse proxy (Caddy/nginx) on the Tailscale address that forwards to `127.0.0.1:8000`
and pass that hostname with `--public-host` (use an `https://` listener so cookies and the
WebSocket origin check behave as in public mode).

## 5. Operating notes and limits

- Visitors control only their own simulation; parameter changes, resets and NPZ downloads are
  per session. Nothing writes to disk on the server.
- Bandwidth per viewer ≈ 0.5 MB/s (10 frames/s main camera + round-robin thumbnails); on a
  home uplink plan for a handful of simultaneous viewers, or lower the camera resolution in
  `MujocoParams` (320×240 halves it twice over).
- The idle reaper runs every 15 s; a visitor who closes the tab and returns within the idle
  timeout gets the same simulation back (cookie), otherwise a fresh one.
- Availability follows this desktop: suspend, reboot or Wi-Fi loss drop the site until the
  user services restart. A cloud VM with the INSTALL.md recipe is the next step if that matters.
- Not implemented: per-visitor quotas beyond the session cap, recording of visitors' runs,
  multiple controllers inside one session, or a landing page to choose between shared and
  private worlds.

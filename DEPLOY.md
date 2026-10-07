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
- **Limits.** `--max-sessions` (default 8; the unit file uses 4) concurrent simulations; beyond
  that the page says "server full" and retries every 20 s. Sessions with no open connection for
  `--idle-timeout` seconds (default 600) are stopped and removed; the page then starts a fresh
  one on reconnect. Each viewer polls about 0.5 MB/s of camera JPEG, so three viewers need
  roughly 12 Mbit/s of **upload** bandwidth from the host.
- **Measured capacity (i7-13700F, RTX 3060 Ti, 62 GB).** Load test with N private sessions,
  each with 4 MuJoCo rovers, 5 cameras at 640×480 and a simulated viewer polling at browser
  rates: N = 1–6 all kept exactly real time (factor 1.00) at about one CPU core in total, ~20 MB
  RAM per session and ~40 % GPU; at N = 8 every session dropped to 0.38× real time. All sessions
  run in one Python process, so the interpreter lock is the ceiling, not RAM or the GPU.
  Conservative setting for this machine: `--max-sessions 4`; raise to 6 only if the uplink allows.
- **Network.** The server still binds to `127.0.0.1`. `--public-host blimp.example.org` makes it
  accept that hostname (Host header, `wss://` origin) and mark cookies `Secure` for it.
- **Authentication.** Cloudflare Access (below) authenticates people before any request reaches
  the server. `--access-token SECRET` adds a second lock inside the app: visitors must open the
  page once as `https://blimp.example.org/?token=SECRET` (it is stored in a cookie and removed
  from the address bar); without it the API answers 401 and the page explains what to do.
- `--shared` restores the original behaviour: one simulation for everyone, first tab controls.

## 0. Step by step for first-timers

Three words you will meet:

* **Domain** — a name you own, like `yourname.dev`. Cloudflare must manage its DNS (the name →
  address lookup). If you do not own one, Path A below needs none; Path B needs one (~US$10/year
  from Cloudflare's own registrar, or any registrar whose nameservers you can change).
* **Tunnel** — a small program (`cloudflared`) on this PC that keeps an outgoing connection to
  Cloudflare. Visitors reach Cloudflare; Cloudflare hands the request down the tunnel to
  `127.0.0.1:8000`. Nothing is opened on your router.
* **Access** — Cloudflare's login page placed in front of your site. A visitor types an e-mail
  address, receives a 6-digit code, and is let in only if the address is on your allow list.

### Path A — a public link today, no domain (10 minutes, for demos)

A1. Install `cloudflared` (once):

```sh
$ sudo mkdir -p --mode=0755 /usr/share/keyrings
$ curl -fsSL https://pkg.cloudflare.com/cloudflare-main.gpg | sudo tee /usr/share/keyrings/cloudflare-main.gpg >/dev/null
$ echo "deb [signed-by=/usr/share/keyrings/cloudflare-main.gpg] https://pkg.cloudflare.com/cloudflared $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/cloudflared.list
$ sudo apt-get update && sudo apt-get install -y cloudflared
$ cloudflared --version
```

A2. Terminal 1 — start a *quick tunnel* and copy the address it prints:

```sh
$ cloudflared tunnel --url http://127.0.0.1:8000
...  https://some-random-words.trycloudflare.com  ...
```

A3. Terminal 2 — make a secret and start the server with the address from A2 (no `https://`):

```sh
$ TOKEN=$(openssl rand -hex 16); echo "$TOKEN"
$ .venv/bin/python web_server.py --no-browser --rovers-backend mujoco \
      --public-host some-random-words.trycloudflare.com --access-token "$TOKEN"
```

A4. Share `https://some-random-words.trycloudflare.com/?token=<the token>`. Test it on your phone
with Wi-Fi off. Each visitor gets a private simulation; the page shows a "Session … · 1/8" badge.
Ctrl+C in both terminals stops it. The address changes every time A2 is restarted, so send the
new link each time.

### Path B — a permanent address with an e-mail login (30 minutes, needs a domain)

B1. **Cloudflare account.** <https://dash.cloudflare.com/sign-up>, free plan, confirm the e-mail.

B2. **Put your domain on Cloudflare.**
   * Already own one elsewhere: dashboard → *Add a site* → type the domain → choose *Free* →
     Cloudflare shows two nameserver names → log in at your registrar, replace its nameservers
     with those two → wait until Cloudflare e-mails "… is now active on Cloudflare" (minutes to a
     few hours).
   * Do not own one: dashboard → *Domain Registration* → *Register Domains* → buy one; it is on
     Cloudflare automatically.

B3. Install `cloudflared` exactly as in A1.

B4. **Connect this PC to your account** (opens a browser tab; pick the domain and press *Authorize*):

```sh
$ cloudflared tunnel login
You have successfully logged in. ... cert.pem
```

B5. **Create the tunnel** and note the long id it prints:

```sh
$ cloudflared tunnel create blimp
Created tunnel blimp with id 6ff42ae2-765d-4adf-8112-31c55c1551ef
```

B6. **Give it a name under your domain** (choose any subdomain; `blimp` here):

```sh
$ cloudflared tunnel route dns blimp blimp.yourdomain.com
```

B7. **Tell cloudflared what to forward** — copy the example and fill in the three placeholders
(`TUNNEL_ID` = the id from B5, `USER` = your Linux user name, the hostname from B6):

```sh
$ mkdir -p ~/.cloudflared && cp deploy/cloudflared-config.example.yml ~/.cloudflared/config.yml
$ nano ~/.cloudflared/config.yml        # or: sed -i "s/TUNNEL_ID/6ff4.../g; s/USER/$USER/; s/blimp.example.org/blimp.yourdomain.com/" ~/.cloudflared/config.yml
```

B8. **Start the server** (Terminal 1). No token needed: the login comes from Access.

```sh
$ .venv/bin/python web_server.py --no-browser --rovers-backend mujoco --public-host blimp.yourdomain.com
```

B9. **Start the tunnel** (Terminal 2) and wait for four "Registered tunnel connection" lines:

```sh
$ cloudflared tunnel run blimp
```

B10. Open `https://blimp.yourdomain.com` — the simulator appears **without any login yet**. Do not
share the link before the next step.

B11. **Add the login page (Access).**
   1. Open <https://one.dash.cloudflare.com>. The first time it asks for a *team name* (e.g.
      `yourname-lab`; your login page becomes `yourname-lab.cloudflareaccess.com`) and a plan —
      choose **Free** (up to 50 users; Cloudflare may ask for a payment method but does not
      charge on the free plan).
   2. Left menu *Access* → *Applications* → *Add an application* → **Self-hosted**.
   3. *Application name*: Blimp simulator. *Session duration*: 24 hours. *Application domain*:
      subdomain `blimp`, domain `yourdomain.com`. Press *Next*.
   4. *Add a policy*: name `Collaborators`, action **Allow**. Under *Include* choose the selector
      **Emails** and list the addresses allowed (one per line), or **Emails ending in** with
      `@your-university.edu` to allow a whole organisation. *Next*, then *Add application*.
   5. The default sign-in method, **One-time PIN**, is already enabled (check under *Settings →
      Authentication → Login methods* if you want Google or GitHub as well).

B12. **Test like a visitor.** Open a private/incognito window at `https://blimp.yourdomain.com`:
   Cloudflare's page asks for an e-mail → *Send me a code* → the 6-digit code arrives by e-mail
   (valid 10 minutes) → the simulator loads. An address not on the list sees "That account does
   not have access". Add or remove people later under *Access → Applications → Blimp simulator →
   Policies* — no server restart needed.

B13. **Make it survive reboots** (then close the two terminals):

```sh
$ mkdir -p ~/.config/systemd/user
$ cp deploy/blimp-web.service deploy/cloudflared.service ~/.config/systemd/user/
$ sed -i "s/blimp.example.org/blimp.yourdomain.com/" ~/.config/systemd/user/blimp-web.service
$ systemctl --user daemon-reload && systemctl --user enable --now blimp-web cloudflared
$ loginctl enable-linger $USER
$ systemctl --user status blimp-web cloudflared      # both: active (running)
```

### If something does not look right

| You see | Meaning | Fix |
| --- | --- | --- |
| Cloudflare error 1033 / "tunnel not found" | the tunnel program is not running | start B9 / A2 again (or `systemctl --user restart cloudflared`) |
| Cloudflare error 502 Bad Gateway | tunnel is up, server is down | start B8 / A3 again (`systemctl --user restart blimp-web`) |
| Browser shows "Invalid host header" or a bare 400 | `--public-host` missing or misspelled | restart the server with the exact hostname (no `https://`, no path) |
| Page says "This server needs an access token" | Path A token missing/wrong | open the full link with `?token=` once |
| Site opens with **no** login page | the Access application domain does not match | B11.3: subdomain and domain must equal the hostname from B6 |
| Login e-mail does not arrive | spam folder, or address not on the policy | check spam; B11.4 |
| "Server full. Retrying…" | more visitors than `--max-sessions` | wait, or raise the cap in the unit file |

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
count, session cap or to add `--access-token`. The web server always renders through EGL
(`MUJOCO_GL=egl` is its default even with a display): GLFW's X11 layer is not thread-safe and
aborted the process when several sessions created renderers at once. On NVIDIA machines the
EGL vendor file is selected automatically (see INSTALL.md §7).

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

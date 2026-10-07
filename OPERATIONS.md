# Day-to-day operation of blimp-bot-simulator.com

The public site runs on this machine (`xzha-roar`) as two user services: `blimp-web` (the
simulator server, loopback port 8000) and `cloudflared` (the tunnel that publishes it as
<https://blimp-bot-simulator.com>). Both start automatically at boot and restart on failure.
Nothing below needs `sudo`.

## Normal days: nothing to do

Leave the machine on and awake. Visitors get a private simulation each (up to 4 at once); a
slot frees 3 minutes after a visitor closes the page.

Keep the PC from sleeping: *Settings → Power → Automatic Suspend → Off* (and *Power Button
Behaviour → Nothing* if it is a laptop lid). While the machine sleeps, the site is down.

## Check that it is up (30 seconds)

```sh
systemctl --user status blimp-web cloudflared --no-pager | grep -E "●|Active"
curl -s http://127.0.0.1:8000/health
```

Expected: both `Active: active (running)`, and `{"status":"ok","sessions":N,"reserved":M,"max_sessions":4}`
where `sessions` = simulations running right now, `reserved` = pages loaded that have not
connected yet (bots, previews). From a phone on mobile data, <https://blimp-bot-simulator.com>
should load and show a `Session … · 1/4` badge after a few seconds.

Note: on the campus/office network the domain may show "Web Page Blocked" — that is the
institution's filter, not the site. Check from mobile data or ask IT to allow the domain.

## See who is using it

```sh
journalctl --user -u blimp-web -f            # live log; Ctrl+C to leave
journalctl --user -u blimp-web --since "1 hour ago" | grep -c '"GET /api/config'   # page loads in the last hour
```

Requests show Cloudflare-forwarded client addresses. Paths like `/.env` or `/config.json`
with `404` are internet scanners probing every new domain; they get nothing.

## Take the site offline / back online

```sh
systemctl --user stop blimp-web        # visitors see "Service unavailable"; the tunnel stays up
systemctl --user start blimp-web
systemctl --user stop cloudflared      # also hides the domain completely (Cloudflare error 1033)
systemctl --user start cloudflared
```

Stopping `blimp-web` ends every running simulation; visitors' browsers reconnect by themselves
when it is back and start fresh sessions.

## Restart after changing code or settings

```sh
cd ~/Documents/blimp-bot-interaction
git pull                                 # branch: mujoco
~/miniconda3/envs/mujoco/bin/python -m unittest discover -s sim/tests -t .   # should end with OK
systemctl --user restart blimp-web
```

Server settings (rover count, session cap, camera size, idle timeout) live in
`~/.config/systemd/user/blimp-web.service` on the `ExecStart` line. After editing it:

```sh
systemctl --user daemon-reload && systemctl --user restart blimp-web
```

Current line:
`web_server.py --no-browser --port 8000 --rovers-backend mujoco --max-sessions 4 --idle-timeout 180 --public-host blimp-bot-simulator.com`

Useful additions: `--camera-size 320x240` (quarter the bandwidth), `--max-rovers 4`
(visitors may not request more than 4 rovers), `--access-token SECRET` (only people with the
`/?token=SECRET` link can use it), `--mode auto` (visitors start in autopilot mode).

## If something is wrong

| Symptom | Check | Fix |
| --- | --- | --- |
| Site shows Cloudflare error **1033** | `systemctl --user status cloudflared` | `systemctl --user restart cloudflared` |
| Site shows Cloudflare error **502** | `systemctl --user status blimp-web`; `journalctl --user -u blimp-web -n 50` | `systemctl --user restart blimp-web`; read the last lines of the log for a Python error |
| Page says **"Server full"** for everyone | `curl -s http://127.0.0.1:8000/health` shows `sessions: 4` | wait 3 minutes (idle sessions expire), or restart `blimp-web` to drop all sessions |
| Simulations feel slow for everyone | `top` shows the `python` process near 150–200 % CPU | too many simultaneous users for one process; see DEPLOY.md §2b to add a second server process |
| Cameras missing, page note says "No camera streams" | `ExecStart` lost `--rovers-backend mujoco` | restore the flag, `daemon-reload`, restart |
| After a reboot nothing runs | `loginctl show-user $USER -p Linger` should print `Linger=yes` | `loginctl enable-linger $USER`; `systemctl --user start blimp-web cloudflared` |
| Internet is back after an outage but the site is not | — | `cloudflared` reconnects on its own within a minute; otherwise restart it |

## Files that matter (back them up if you move machines)

| Path | What it is |
| --- | --- |
| `~/.cloudflared/config.yml` | which hostname forwards to which local port |
| `~/.cloudflared/99bc59f9-40e1-4d49-9c67-8f57b8d8386e.json` | the tunnel's credentials — **secret**, never commit or share |
| `~/.cloudflared/cert.pem` | your Cloudflare authorisation for creating/routing tunnels |
| `~/.config/systemd/user/blimp-web.service`, `cloudflared.service` | the two services (templates in `deploy/`) |
| `~/Documents/blimp-bot-interaction` | the code, branch `mujoco` |
| `~/miniconda3/envs/mujoco` | the Python environment the service uses |

The site state is entirely in memory: there is nothing to back up for visitors' sessions.

## Yearly

Cloudflare renews `blimp-bot-simulator.com` automatically (about US$10.46/year, card on file,
e-mail notice beforehand). If the card changes, update it at dash.cloudflare.com → *Billing*
before the renewal date, otherwise the link stops working when the domain expires.

## Later options (documented, not set up)

- Add a login page so only listed e-mails can enter: DEPLOY.md §2, step B11.
- Serve more than 4 people at once: a second server process behind a sticky proxy, DEPLOY.md §2b.
- Move the site to another machine: install the code there (INSTALL.md), copy the three
  `~/.cloudflared` files and the two unit files, start the services there and stop them here.
  Run on one machine at a time: a visitor's session lives in one server's memory.

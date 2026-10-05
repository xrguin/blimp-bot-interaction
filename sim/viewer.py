"""3D matplotlib viewer with live sliders/numeric fields or headless video rendering.

Layout (GUI, 16 x 10 in):

    +------------------------------+-----------------+----------------------------+
    |                              |  attitude HUD   |  [ PID / HOLD : ON ]       |
    |        3D scene              |  heading tape   |  [ RESET ]  [ PAUSE ]      |
    |   blimp, rovers, forces      |  stick boxes    |  -- blimp physics --       |
    |                              |                 |  sliders ...               |
    |                              |                 |  -- controller / task --   |
    |                              |                 |  sliders + number fields   |
    |                              |                 |  -- view --                |
    +------------------------------+-----------------+----------------------------+
    | vehicle information: altitude / target, motion, forces | controls continued |
    | teleop command and keyboard hints (when enabled)       |                    |
    | legend / status bar                                                         |
    +-----------------------------------------------------------------------------+

Display frame is ENU-like for readability: X = x (north), Y = y (east), Z = CV height = -z.
Altitude readouts report the lowest gondola-assembly clearance above the ground.
"""
from __future__ import annotations

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .blimp import R_zyx
from .params import G, SLIDERS
from .sim import TeamSim

PAL = dict(blimp="#2a78d6", gond="#0b0b0b", rover=["#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7", "#e34948", "#008300", "#2a78d6"],
           ref="#8a8984", grid="#e6e5e1", panel="#f4f3ef", ink="#0b0b0b", dim="#52514e",
           on="#1baf7a", off="#c3c2b7", warn="#eb6834")

# slider groups shown in the control panel, in this order (names refer to SLIDERS entries)
SLIDER_GROUPS = [
    ("blimp physics", "blimp"),
    ("controller / task", "task"),
    ("view", "view"),
]


def _ellipsoid(a, b, c, n=14):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n // 2 + 1)
    x = a * np.outer(np.cos(u), np.sin(v)); y = b * np.outer(np.sin(u), np.sin(v)); z = c * np.outer(np.ones_like(u), np.cos(v))
    return np.stack([x, y, z], axis=-1)      # (n, m, 3) body frame


class HUD:
    """Aircraft-style attitude indicator (artificial horizon + pitch ladder + roll scale), a
    heading tape for yaw, a numeric readout and two "stick" boxes for the 4-axis command,
    stacked vertically so the whole thing fits a narrow column.

    Conventions (NED / Fossen): roll phi > 0 = right side down, pitch theta > 0 = nose up,
    yaw psi measured from north (x) toward east (y)."""

    DEG_PER_UNIT = 30.0        # pitch ladder scale: 30 deg per unit of HUD height
    TAPE_HALF_DEG = 60.0       # heading tape shows +-60 deg around the current yaw

    def __init__(self, fig, rect):
        from matplotlib.patches import Circle, Polygon
        self.Polygon = Polygon
        self.ax = fig.add_axes(rect)
        ax = self.ax
        ax.set_xlim(-1.3, 1.3); ax.set_ylim(-3.15, 1.25); ax.set_aspect("equal"); ax.axis("off")
        # attitude ball
        self.clip = Circle((0, 0), 1.0, transform=ax.transData, facecolor="none", edgecolor=PAL["ink"], lw=1.5)
        ax.add_patch(self.clip)
        self.dyn = []
        # fixed aircraft symbol
        ax.plot([-0.45, -0.15], [0, 0], color="#eda100", lw=3); ax.plot([0.15, 0.45], [0, 0], color="#eda100", lw=3)
        ax.plot([-0.15, 0, 0.15], [0, -0.08, 0], color="#eda100", lw=3)
        # roll scale (fixed ticks on the top arc)
        for d in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            a = np.deg2rad(90 - d); L = 0.1 if d % 30 == 0 else 0.06
            ax.plot([np.cos(a) * 1.0, np.cos(a) * (1.0 + L)], [np.sin(a) * 1.0, np.sin(a) * (1.0 + L)], color=PAL["ink"], lw=1)
        # heading tape frame + fixed index
        ax.plot([-1.15, 1.15], [-1.25, -1.25], color=PAL["ink"], lw=1)
        ax.plot([-1.15, 1.15], [-1.55, -1.55], color=PAL["ink"], lw=1)
        ax.add_patch(Polygon([[0, -1.22], [-0.06, -1.12], [0.06, -1.12]], closed=True, color="#eda100"))
        self.readout = ax.text(0, -1.66, "", ha="center", va="top", fontsize=9, color=PAL["ink"], family="monospace")
        # two "stick" boxes under the readout: left = (sway, surge), right = (yaw, heave), command in [-1, 1]
        self.stick_c = [(-0.62, -2.6), (0.62, -2.6)]
        for (cx_, cy_), lab in zip(self.stick_c, ("sway / surge", "yaw / heave")):
            ax.add_patch(Polygon([[cx_ - 0.45, cy_ - 0.45], [cx_ + 0.45, cy_ - 0.45], [cx_ + 0.45, cy_ + 0.45], [cx_ - 0.45, cy_ + 0.45]],
                                 closed=True, facecolor=PAL["panel"], edgecolor=PAL["ink"], lw=1))
            ax.plot([cx_ - 0.45, cx_ + 0.45], [cy_, cy_], color="#c3c2b7", lw=0.8); ax.plot([cx_, cx_], [cy_ - 0.45, cy_ + 0.45], color="#c3c2b7", lw=0.8)
            ax.text(cx_, cy_ + 0.49, lab, ha="center", va="bottom", fontsize=7, color=PAL["dim"])
        self.stick_dots = [ax.plot([cx_], [cy_], "o", color="#eb6834", ms=7)[0] for cx_, cy_ in self.stick_c]
        self.cmd4 = np.zeros(4)

    def update(self, roll, pitch, yaw):
        import matplotlib.transforms as mt
        ax = self.ax
        for a in self.dyn:
            a.remove()
        self.dyn = []
        r_deg, p_deg, y_deg = np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
        # horizon transform: rotate by -roll about the centre, shift down by pitch
        tr = mt.Affine2D().translate(0, -p_deg / self.DEG_PER_UNIT).rotate_deg(-r_deg) + ax.transData
        sky = self.Polygon([[-3, 0], [3, 0], [3, 3], [-3, 3]], closed=True, color="#cfe3fb", transform=tr, zorder=0)
        gnd = self.Polygon([[-3, 0], [3, 0], [3, -3], [-3, -3]], closed=True, color="#d9c7a3", transform=tr, zorder=0)
        for p_ in (sky, gnd):
            ax.add_patch(p_); p_.set_clip_path(self.clip); self.dyn.append(p_)
        hz, = ax.plot([-3, 3], [0, 0], color=PAL["ink"], lw=1.5, transform=tr, zorder=1); hz.set_clip_path(self.clip); self.dyn.append(hz)
        for d in range(-30, 31, 10):
            if d == 0:
                continue
            yy = d / self.DEG_PER_UNIT; w = 0.35 if d % 20 == 0 else 0.2
            ln, = ax.plot([-w, w], [yy, yy], color=PAL["ink"], lw=1, transform=tr, zorder=1); ln.set_clip_path(self.clip); self.dyn.append(ln)
            t1 = ax.text(w + 0.05, yy, f"{d:+d}", fontsize=7, va="center", transform=tr, zorder=1, clip_on=True); t1.set_clip_path(self.clip); self.dyn.append(t1)
        # roll pointer (moves with roll; angle measured from the top)
        a = np.deg2rad(90 - r_deg)
        ptr = self.Polygon([[np.cos(a) * 0.98, np.sin(a) * 0.98], [np.cos(a + 0.05) * 0.86, np.sin(a + 0.05) * 0.86], [np.cos(a - 0.05) * 0.86, np.sin(a - 0.05) * 0.86]], closed=True, color="#eda100", zorder=2)
        ax.add_patch(ptr); self.dyn.append(ptr)
        # heading tape
        for h in range(-180, 541, 10):
            dx = ((h - y_deg + 180) % 360 - 180)
            if abs(dx) > self.TAPE_HALF_DEG:
                continue
            x = dx / self.TAPE_HALF_DEG * 1.1
            hh = h % 360
            L = 0.12 if hh % 30 == 0 else 0.06
            ln, = ax.plot([x, x], [-1.55, -1.55 + L], color=PAL["ink"], lw=1); self.dyn.append(ln)
            if hh % 30 == 0:
                lab = {0: "N", 90: "E", 180: "S", 270: "W"}.get(hh, f"{hh}")
                self.dyn.append(ax.text(x, -1.5 + L, lab, ha="center", va="bottom", fontsize=7))
        self.readout.set_text(f"R {r_deg:+6.1f}°  P {p_deg:+6.1f}°  Y {y_deg:+6.1f}°")
        s, w, h, y = self.cmd4                      # surge, sway, heave, yaw
        (lx, ly), (rx, ry) = self.stick_c
        self.stick_dots[0].set_data([lx - 0.42 * w], [ly + 0.42 * s])     # +sway = left on the box
        self.stick_dots[1].set_data([rx + 0.42 * y], [ry + 0.42 * h])


class Viewer:
    """3D scene + HUD + telemetry + (GUI) control panel.

    Hooks for entry points:
        viewer.extra_lines : callable -> list[str], appended to the telemetry block each frame
        viewer.on_reset    : list of callables run after a Reset (button or viewer.reset())
        viewer.reset_eta0  : blimp pose used by Reset / run_gui (None = TeamSim default)
    """

    # figure-fraction geometry (GUI): 3D | HUD column | control panel
    GUI_FIG = (16, 10)
    GUI_3D = [-0.035, 0.33, 0.515, 0.67]
    GUI_HUD = [0.505, 0.36, 0.20, 0.62]
    GUI_PANEL_X0, GUI_PANEL_X1 = 0.715, 0.995
    VID_FIG = (12, 7.2)
    VID_3D = [-0.02, 0.05, 0.67, 0.94]
    VID_HUD = [0.66, 0.36, 0.33, 0.62]
    VID_TELEM = (0.67, 0.335)

    def __init__(self, sim: TeamSim, gui: bool = True, trail_s: float = 15.0):
        self.sim, self.gui = sim, gui
        self.trail_n = int(trail_s / sim.P.dt_ctrl)
        self.fig = plt.figure(figsize=self.GUI_FIG if gui else self.VID_FIG)
        self.fig.patch.set_facecolor("white")
        self.ax = self.fig.add_axes(self.GUI_3D if gui else self.VID_3D, projection="3d")
        p = sim.P.blimp
        self.mesh0 = _ellipsoid(p.r_env, p.r_env, p.h_env / 2)
        self.trails = [[] for _ in sim.rovers]
        self.artists = []
        self.sliders = []
        self.value_inputs = []
        self.paused = False
        self.extra_lines = None
        self.on_reset = []
        self.reset_eta0 = None
        self._static()
        self.hud = HUD(self.fig, self.GUI_HUD if gui else self.VID_HUD)
        if gui:
            self._build_panel()
            self._build_vehicle_info()
        else:
            tx, ty = self.VID_TELEM
            self.telemetry = self.fig.text(tx, ty, "", ha="left", va="top", fontsize=8.5,
                                           family="monospace", color=PAL["ink"], linespacing=1.35)

    # ------------------------------------------------------------- static scene
    def _static(self):
        ax, A = self.ax, self.sim.P.arena / 2
        ax.set_xlim(-A, A); ax.set_ylim(-A, A); ax.set_zlim(0, 3.0)
        ax.set_box_aspect((1, 1, 0.5), zoom=1.06)      # slight zoom; matplotlib pads 3D axes heavily
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("height above ground (m)")
        ax.tick_params(labelsize=8)
        tk = self.sim.P.task
        ang = np.linspace(0, 2 * np.pi, 100)
        self.ref_line, = ax.plot(tk.circle_center[0] + tk.circle_radius * np.cos(ang),
                                 tk.circle_center[1] + tk.circle_radius * np.sin(ang), 0 * ang, color=PAL["ref"], lw=1, ls="--")
        g = np.arange(-A, A + 0.01, 1.0)
        for gx in g:
            ax.plot([gx, gx], [-A, A], [0, 0], color=PAL["grid"], lw=0.6)
            ax.plot([-A, A], [gx, gx], [0, 0], color=PAL["grid"], lw=0.6)
        # legend / status bar along the bottom of the figure (nothing is drawn over the 3D axes)
        self.fig.text(0.012, 0.018,
                      "force at CV:  x red   y green   z blue   net black        cyan = blimp front        "
                      "red ticks = thruster output        dashed = rover reference circle",
                      fontsize=8.5, color=PAL["dim"])
        self.fig.add_artist(matplotlib.lines.Line2D([0.01, 0.99], [0.052, 0.052], transform=self.fig.transFigure, color="#dddcd6", lw=0.8))

    # ------------------------------------------------------------- control panel
    def _panel_header(self, y, text):
        self.fig.text(self.GUI_PANEL_X0, y, text.upper(), fontsize=8, fontweight="bold", color=PAL["dim"], va="center")
        self.fig.add_artist(matplotlib.lines.Line2D([self.GUI_PANEL_X0, self.GUI_PANEL_X1], [y - 0.012, y - 0.012],
                                                    transform=self.fig.transFigure, color="#dddcd6", lw=0.8))

    def _build_panel(self):
        x0, x1 = self.GUI_PANEL_X0, self.GUI_PANEL_X1
        # --- buttons: the PID toggle is the first, largest control on the panel
        y_top = 0.975
        self.pid_btn = Button(self.fig.add_axes([x0, y_top - 0.055, x1 - x0, 0.055]), "", color=PAL["on"], hovercolor="#8fd6bb")
        self.pid_btn.label.set_fontsize(12); self.pid_btn.label.set_fontweight("bold")
        self.pid_btn.on_clicked(lambda _ev: self.set_pid(not self.sim.P.task.pid_enabled))
        y_btn = y_top - 0.055 - 0.012 - 0.04
        half = (x1 - x0 - 0.01) / 2
        self.reset_btn = Button(self.fig.add_axes([x0, y_btn, half, 0.04]), "RESET  (sim to t = 0)", color=PAL["panel"], hovercolor="#e6e5e1")
        self.reset_btn.label.set_fontsize(9)
        self.reset_btn.on_clicked(lambda _ev: self.reset())
        self.pause_btn = Button(self.fig.add_axes([x0 + half + 0.01, y_btn, half, 0.04]), "PAUSE", color=PAL["panel"], hovercolor="#e6e5e1")
        self.pause_btn.label.set_fontsize(9)
        self.pause_btn.on_clicked(lambda _ev: self.set_paused(not self.paused))
        self._refresh_buttons()
        # --- grouped sliders with exact numeric entry
        label_w = 0.085                      # room for the slider name on the left
        input_w, input_gap = 0.065, 0.010
        sx0, sh, gap = x0 + label_w, 0.015, 0.0295
        input_x = x1 - input_w
        sw = input_x - input_gap - sx0
        y = y_btn - 0.045
        for header, obj in SLIDER_GROUPS:
            self._panel_header(y, header)
            y -= 0.032
            for o, name, lo, hi in SLIDERS:
                if o != obj:
                    continue
                axs = self.fig.add_axes([sx0, y, sw, sh])
                target = getattr(self.sim.P, obj)
                display_name = "net lift (g)" if name == "net_lift_g" else name
                s = Slider(axs, display_name, lo, hi, valinit=getattr(target, name), valfmt="%.3g", color="#2a78d6", track_color="#e6e5e1")
                s.label.set_fontsize(8)
                s.valtext.set_visible(False)
                box = TextBox(self.fig.add_axes([input_x, y - 0.003, input_w, sh + 0.006]), "",
                              initial=f"{s.val:.10g}", color=PAL["panel"], hovercolor="white")
                box.text_disp.set_fontsize(8)
                for spine in box.ax.spines.values():
                    spine.set_edgecolor("#c3c2b7")
                self.value_inputs.append(box)

                def on_change(val, target=target, name=name, box=box):
                    setattr(target, name, float(val))
                    self.sim.blimp.rebuild()
                    self._sync_value_input(box, val)
                s.on_changed(on_change)
                box.on_submit(lambda text, s=s, box=box: self._submit_value(s, box, text))
                self.sliders.append(s)
                y -= gap
            y -= 0.012
        self.fig.text(x0, max(y, 0.065), "Drag or type a value • Enter to apply • Click outside to use keys",
                      fontsize=7.5, color=PAL["dim"], va="bottom")

    def _build_vehicle_info(self):
        """A wide, readable instrument strip below the scene and attitude display."""
        from matplotlib.patches import FancyBboxPatch

        def label(x, y, text, size=10, **kwargs):
            return self.fig.text(x, y, text, fontsize=size, color=PAL["dim"], va="center", **kwargs)

        def value(x, y, size=12):
            return self.fig.text(x, y, "", fontsize=size, color=PAL["ink"], va="center")

        label(0.025, 0.307, "VEHICLE INFORMATION", 11, fontweight="bold")
        self.vehicle_status = label(0.700, 0.307, "", ha="right")
        for x, width in ((0.025, 0.185), (0.220, 0.232), (0.462, 0.238)):
            self.fig.add_artist(FancyBboxPatch((x, 0.103), width, 0.184,
                                boxstyle="round,pad=0.006,rounding_size=0.008",
                                transform=self.fig.transFigure, facecolor="#f3f5f7",
                                edgecolor="#dddfe2", linewidth=0.8, zorder=-1))

        label(0.036, 0.265, "GONDOLA CLEARANCE", fontweight="bold")
        self.altitude_readout = value(0.036, 0.225, 26)
        label(0.036, 0.172, "Desired clearance (m)")
        self.altitude_input = TextBox(self.fig.add_axes([0.142, 0.151, 0.061, 0.041]), "",
                                      initial=f"{self.sim.P.task.blimp_height:.10g}", color="white",
                                      hovercolor="#e8f1fc")
        self.altitude_input.text_disp.set_fontsize(14)
        for spine in self.altitude_input.ax.spines.values():
            spine.set_edgecolor(PAL["blimp"])
        self.value_inputs.append(self.altitude_input)
        self.height_slider = next(s for s in self.sliders if s.label.get_text() == "blimp_height")
        self.height_input = self.value_inputs[self.sliders.index(self.height_slider)]
        self.height_slider.on_changed(lambda val: self._sync_value_input(self.altitude_input, val))
        self.height_slider.on_changed(self._set_altitude_target)
        self.altitude_input.on_submit(lambda text: self._submit_value(self.height_slider, self.altitude_input, text))
        label(0.036, 0.123, f"{self.height_slider.valmin:g}–{self.height_slider.valmax:g} m  •  Enter to apply", 9)

        self.vehicle_values = {}
        for key, y, heading in (("position", 0.265, "POSITION: X, Y, HEIGHT (m)"),
                                ("velocity", 0.209, "BODY VELOCITY (m/s)"),
                                ("attitude", 0.153, "ATTITUDE (deg)")):
            label(0.231, y, heading, fontweight="bold")
            self.vehicle_values[key] = value(0.231, y - 0.027, 12)
        label(0.473, 0.265, "THRUST / LIFT", fontweight="bold")
        for key, y in (("lift", 0.235), ("thrust", 0.207), ("force_xy", 0.179), ("force_up", 0.151)):
            self.vehicle_values[key] = value(0.473, y, 12)
        self.vehicle_values["force_scale"] = value(0.473, 0.122, 10)
        self.telemetry_extra = self.fig.text(0.025, 0.090, "", fontsize=10, color=PAL["ink"],
                                             va="top", linespacing=1.35)

    def _set_altitude_target(self, value):
        controller = self.sim.blimp_ctrl
        if hasattr(controller, "set_altitude_target"):
            controller.set_altitude_target(float(value))

    def _update_vehicle_info(self, force):
        b, tk = self.sim.blimp, self.sim.P.task
        self._refresh_altitude_controls()
        roll, pitch, yaw = np.degrees(b.eta[3:6])
        self.altitude_readout.set_text(f"{b.altitude:.2f} m")
        mode = "PAUSED" if self.paused else "RUNNING"
        self.vehicle_status.set_text(f"{self.sim.t:.1f} s   •   {mode}   •   HOLD {'ON' if tk.pid_enabled else 'OFF'}")
        self.vehicle_status.set_color(PAL["on"] if tk.pid_enabled else PAL["dim"])
        values = {
            "position": f"{b.eta[0]:+.2f}    {b.eta[1]:+.2f}    {b.altitude:.2f}",
            "velocity": f"u  {b.nu[0]:+.2f}    v  {b.nu[1]:+.2f}    w  {b.nu[2]:+.2f}",
            "attitude": f"Roll {roll:+.1f}   Pitch {pitch:+.1f}   Yaw {yaw:+.1f}",
            "lift": f"Net lift (+ up)  {1000 * b.lift / G:+.2f} g equiv.",
            "thrust": f"Max per thruster    {b.p.T_max:.3f} N",
            "force_xy": f"Thruster Fx  {force[0]:+.3f}   Fy  {force[1]:+.3f} N",
            "force_up": f"Thruster upward      {-force[2]:+.3f} N",
            "force_scale": f"Arrow scale  {self.sim.P.view.force_scale:g} m/N",
        }
        for name, text in values.items():
            self.vehicle_values[name].set_text(text)
        self.telemetry_extra.set_text("\n".join(self.extra_lines()) if self.extra_lines else "")

    def _refresh_altitude_controls(self):
        # Teleop can capture a new hold height after manual flight or a hold toggle.
        value = float(self.sim.P.task.blimp_height)
        if self.altitude_input.capturekeystrokes or self.height_input.capturekeystrokes:
            return
        slider = self.height_slider
        if value == slider.val:
            return
        eventson = slider.eventson
        slider.eventson = False
        try:
            # Captured manual heights may lie beyond the limits for typed targets.
            # Keep the handle on its track, while the fields report the true target.
            slider.set_val(np.clip(value, slider.valmin, slider.valmax))
            slider.val = value
        finally:
            slider.eventson = eventson
        self._sync_value_input(self.height_input, value)
        self._sync_value_input(self.altitude_input, value)

    @property
    def text_input_active(self):
        """Entry points must leave keyboard events to a focused numeric field."""
        return any(box.capturekeystrokes for box in self.value_inputs)

    def _sync_value_input(self, box, value):
        # TextBox.set_val also submits: suppress that callback during slider sync.
        eventson = box.eventson
        box.eventson = False
        try:
            if box.capturekeystrokes:
                box.set_val(f"{value:.10g}")
            else:
                box.text_disp.set_text(f"{value:.10g}")
            box.cursor_index = min(box.cursor_index, len(box.text))
            box.cursor.set_visible(box.capturekeystrokes)
        finally:
            box.eventson = eventson
        self.fig.canvas.draw_idle()

    def _submit_value(self, slider, box, text):
        try:
            value = float(text)
        except ValueError:
            value = np.nan
        if np.isfinite(value) and slider.valmin <= value <= slider.valmax:
            # Reuse every existing callback, including teleop thrust-authority updates.
            slider.set_val(value)
        else:
            self._sync_value_input(box, slider.val)

    def _refresh_buttons(self):
        if not hasattr(self, "pid_btn"):
            return
        on = bool(self.sim.P.task.pid_enabled)
        self.pid_btn.label.set_text(f"PID / HOLD :  {'ON' if on else 'OFF'}      (click or H)")
        self.pid_btn.color = PAL["on"] if on else PAL["off"]
        self.pid_btn.hovercolor = "#8fd6bb" if on else "#dddcd6"
        self.pid_btn.ax.set_facecolor(self.pid_btn.color)
        self.pause_btn.label.set_text("RESUME" if self.paused else "PAUSE")
        self.pause_btn.color = PAL["warn"] if self.paused else PAL["panel"]
        self.pause_btn.ax.set_facecolor(self.pause_btn.color)
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------- live controls
    def set_pid(self, enabled: bool):
        self.sim.set_pid(enabled)
        self._refresh_buttons()
        print(f"[viewer] PID {'ON' if enabled else 'OFF'}")

    def set_paused(self, paused: bool):
        self.paused = bool(paused)
        self._refresh_buttons()
        print(f"[viewer] {'paused' if self.paused else 'running'}")

    def on_key(self, ev):
        """Default key bindings for the scenario GUI (teleop installs its own handler instead)."""
        if self.text_input_active:
            return
        k = (ev.key or "").lower()
        if k == "h":
            self.set_pid(not self.sim.P.task.pid_enabled)
        elif k == "r":
            self.reset()
        elif k in (" ", "space"):
            self.set_paused(not self.paused)

    def reset(self):
        self.sim.reset(blimp_eta0=self.reset_eta0)
        self.trails = [[] for _ in self.sim.rovers]
        for cb in self.on_reset:
            cb()
        self.paused = False
        self._refresh_buttons()
        print("[viewer] reset")

    # ------------------------------------------------------------- per frame
    def _telemetry_text(self, F_w):
        sim, b, tk = self.sim, self.sim.blimp, self.sim.P.task
        R_, P_, Y_ = np.degrees(b.eta[3:6])
        lines = [
            f"t     {sim.t:7.1f} s        PID/hold {'ON ' if tk.pid_enabled else 'OFF'}",
            f"clear {b.altitude:6.2f} m   ref {tk.blimp_height:4.2f} m",
            f"pos   x {b.eta[0]:+6.2f}  y {b.eta[1]:+6.2f} m",
            f"vel   u {b.nu[0]:+5.2f}  v {b.nu[1]:+5.2f}  w {b.nu[2]:+5.2f} m/s",
            f"att   R {R_:+6.1f}  P {P_:+6.1f}  Y {Y_:+6.1f} deg",
            f"lift  {1000 * b.lift / G:+.2f} g equiv. (+ up)   T_max {b.p.T_max:.3f} N",
            f"F@CV  Fx {F_w[0]:+.3f}  Fy {F_w[1]:+.3f}  Fz(up) {-F_w[2]:+.3f} N",
            f"arrow scale {self.sim.P.view.force_scale:.0f} m/N",
        ]
        if self.extra_lines is not None:
            lines += list(self.extra_lines())
        if self.paused:
            lines.append("** PAUSED **")
        return "\n".join(lines)

    def draw(self):
        for a in self.artists:
            a.remove()
        self.artists = []
        ax, sim = self.ax, self.sim
        b = sim.blimp
        R = R_zyx(*b.eta[3:6])
        # ellipsoid at CV (display z = -ned z)
        pts = self.mesh0 @ R.T + b.eta[:3]
        X, Y, Z = pts[..., 0], pts[..., 1], -pts[..., 2]
        self.artists.append(ax.plot_wireframe(X, Y, Z, color=PAL["blimp"], lw=0.6, alpha=0.8))
        # Gondola box: shared geometry dimensions are body x length, y width, z height.
        g = b.eta[:3] + R @ np.array([0, 0, b.p.d_VT])
        self.artists += ax.plot([b.eta[0], g[0]], [b.eta[1], g[1]], [-b.eta[2], -g[2]], color=PAL["gond"], lw=2)
        gx, gy, gz = b.p.gondola_size
        corners = np.array([[sx * gx / 2, sy * gy / 2, b.p.d_VT + sz * gz / 2]
                            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners_w = corners @ R.T + b.eta[:3]
        corners_d = np.column_stack((corners_w[:, 0], corners_w[:, 1], -corners_w[:, 2]))
        faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
                 [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
        gondola = Poly3DCollection([corners_d[idx] for idx in faces], facecolor="#303b40",
                                   edgecolor=PAL["gond"], linewidth=0.7, alpha=0.85)
        ax.add_collection3d(gondola)
        self.artists.append(gondola)
        # Six motor housings use the same 0.13 m length and 0.025 m radius as the browser scene.
        for i in range(6):
            pw = b.eta[:3] + R @ b.pos[i]
            axis = b.axes[i] / max(np.linalg.norm(b.axes[i]), 1e-12)
            basis = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
            side1 = np.cross(axis, basis); side1 /= np.linalg.norm(side1)
            side2 = np.cross(axis, side1)
            angles = np.linspace(0, 2 * np.pi, 9)
            rings = []
            half_length = b.p.thruster_length / 2
            for offset in (-half_length, half_length):
                ring_b = (b.pos[i] + offset * axis
                          + b.p.thruster_radius * (np.cos(angles)[:, None] * side1 + np.sin(angles)[:, None] * side2))
                ring_w = ring_b @ R.T + b.eta[:3]
                rings.append(ring_w)
            X = np.vstack([rings[0][:, 0], rings[1][:, 0]])
            Y = np.vstack([rings[0][:, 1], rings[1][:, 1]])
            Z = -np.vstack([rings[0][:, 2], rings[1][:, 2]])
            self.artists.append(ax.plot_wireframe(X, Y, Z, color=PAL["gond"], lw=0.55, alpha=0.9))
            thrust_start = pw + R @ (half_length * np.sign(b.thrust[i] or 1.0) * axis)
            fw = R @ (axis * b.thrust[i] / max(b.p.T_max, 1e-9) * 0.25)
            self.artists += ax.plot([thrust_start[0], thrust_start[0] + fw[0]],
                                    [thrust_start[1], thrust_start[1] + fw[1]],
                                    [-thrust_start[2], -thrust_start[2] - fw[2]], color="#e34948", lw=1.5)
        # net thruster force received by the blimp, at CV, world frame; display z = -ned z
        F_body = b.wrench(b.thrust)[:3]
        F_w = R @ F_body
        s = self.sim.P.view.force_scale
        cx, cy, cz = b.eta[0], b.eta[1], -b.eta[2]
        comps = [(F_w[0], 0, 0, "#e34948"), (0, F_w[1], 0, "#008300"), (0, 0, -F_w[2], "#2a78d6")]
        if self.sim.P.view.show_components:
            for dx, dy, dz, col in comps:
                self.artists.append(ax.quiver(cx, cy, cz, s * dx, s * dy, s * dz, color=col, lw=2, arrow_length_ratio=0.25))
        self.artists.append(ax.quiver(cx, cy, cz, s * F_w[0], s * F_w[1], -s * F_w[2], color="#0b0b0b", lw=2.5, arrow_length_ratio=0.2))
        # front direction (body x axis) in cyan, from the CV
        fwd = R @ np.array([1.0, 0.0, 0.0]) * (1.6 * b.p.r_env)
        self.artists.append(ax.quiver(cx, cy, cz, fwd[0], fwd[1], -fwd[2], color="#00bcd4", lw=3, arrow_length_ratio=0.3))
        # shadow of CV on the floor + heading tick
        self.artists += ax.plot([b.eta[0]], [b.eta[1]], [0], "x", color=PAL["blimp"], ms=6)
        self.artists += ax.plot([b.eta[0], b.eta[0] + 0.5 * np.cos(b.eta[5])], [b.eta[1], b.eta[1] + 0.5 * np.sin(b.eta[5])], [0, 0], color="#00bcd4", lw=2)
        # rovers
        for i, r in enumerate(sim.rovers):
            col = PAL["rover"][i % len(PAL["rover"])]
            x, y, th = r.q
            L = r.p.body_len
            self.artists += ax.plot([x - 0.5 * L * np.cos(th), x + 0.5 * L * np.cos(th)], [y - 0.5 * L * np.sin(th), y + 0.5 * L * np.sin(th)], [0, 0], color=col, lw=4)
            self.artists += ax.plot([x + 0.5 * L * np.cos(th)], [y + 0.5 * L * np.sin(th)], [0], "o", color=col, ms=4)
            if not self.paused:
                self.trails[i].append((x, y))
            if self.trails[i]:
                tr = np.array(self.trails[i][-self.trail_n:])
                self.artists += ax.plot(tr[:, 0], tr[:, 1], 0 * tr[:, 0], color=col, lw=0.8, alpha=0.6)
        self.hud.update(b.eta[3], b.eta[4], b.eta[5])
        if self.gui:
            self._update_vehicle_info(F_w)
        else:
            self.telemetry.set_text(self._telemetry_text(F_w))

    def _frame(self, _):
        if not self.paused:
            self.sim.step()
        self.draw()
        return self.artists

    def run_gui(self):
        self.sim.reset(blimp_eta0=self.reset_eta0)
        self.trails = [[] for _ in self.sim.rovers]
        self._refresh_buttons()
        self.anim = animation.FuncAnimation(self.fig, self._frame, interval=int(1000 * self.sim.P.dt_ctrl), blit=False, cache_frame_data=False)
        plt.show()

    def render_video(self, path, T=None, fps=20, every=1):
        self.sim.reset(blimp_eta0=self.reset_eta0)
        T = T or self.sim.P.T_end
        n = int(round(T / self.sim.P.dt_ctrl))
        writer = animation.FFMpegWriter(fps=fps, bitrate=2400)
        with writer.saving(self.fig, path, dpi=110):
            for k in range(n):
                self.sim.step()
                if k % every == 0:
                    self.draw(); writer.grab_frame()
        return self.sim.logs()

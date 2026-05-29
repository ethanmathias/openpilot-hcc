"""Single pygame window compositing top-down + lead POV + ego POV.

Reads from the shared numpy buffers exposed by MetaDriveWorld
(`topdown_image`, `lead_road_image`, `road_image`). Runs in its own thread
inside the bridge process; quitting the window sets the world's exit_event.
"""
from __future__ import annotations

import threading
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
  from openpilot.tools.sim.bridge.metadrive.metadrive_world import MetaDriveWorld

# Layout: 1600×900 split into a 900×900 top-down pane on the left, and two
# 700×450 camera POV panes stacked vertically on the right (ego on top, lead
# below).
WINDOW_W = 1600
WINDOW_H = 900
TOPDOWN_RECT = (0, 0, 900, 900)
EGO_RECT = (900, 0, 700, 450)
LEAD_RECT = (900, 450, 700, 450)
HUD_FONT_SIZE = 18

_PANE_BG_COLOR = (32, 32, 40)
_HUD_TEXT_COLOR = (220, 220, 220)
_HUD_TEXT_OFFSET = (8, 8)


class HILWindow:
  def __init__(self, world: MetaDriveWorld, title: str = "openpilot HIL — MetaDrive"):
    self.world = world
    self.title = title
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None
    self.ego_telemetry: dict[str, float | str] = {}
    self.lead_telemetry: dict[str, float | str] = {}
    self._pygame = None
    self._font = None

  def start(self) -> None:
    if self._thread is not None:
      return
    self._thread = threading.Thread(target=self._run, name="hil-window", daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=2.0)

  def _run(self) -> None:
    try:
      import pygame
    except ImportError:
      print("[hil-window] pygame not available; window disabled")
      return

    self._pygame = pygame
    pygame.init()
    pygame.display.set_caption(self.title)
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    self._font = pygame.font.SysFont(None, HUD_FONT_SIZE)
    clock = pygame.time.Clock()

    while not self._stop.is_set():
      for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
          self.world.exit_event.set()
          self._stop.set()
          break

      screen.fill((0, 0, 0))
      self._blit_pane(screen, TOPDOWN_RECT, self.world.topdown_image, "Top-down")
      self._blit_pane(screen, EGO_RECT, self.world.road_image, self._hud_text("EGO", self.ego_telemetry))
      self._blit_pane(screen, LEAD_RECT, self.world.lead_road_image, self._hud_text("LEAD", self.lead_telemetry))

      pygame.display.flip()
      clock.tick(20)

    pygame.quit()

  @staticmethod
  def _hud_text(label: str, telemetry: dict[str, float | str]) -> str:
    if not telemetry:
      return label
    parts = [label]
    for key in ("vEgo", "vLead", "dRel", "hccc"):
      if key in telemetry:
        v = telemetry[key]
        parts.append(f"{key}={v:.2f}" if isinstance(v, (int, float)) else f"{key}={v}")
    return "  ".join(parts)

  def _blit_pane(self, screen, rect: tuple, img: np.ndarray | None, label: str) -> None:
    x, y, w, h = rect
    self._pygame.draw.rect(screen, _PANE_BG_COLOR, (x, y, w, h))
    if img is not None and img.size > 0:
      try:
        # pygame surfaces want (W, H, 3); numpy arrays are (H, W, 3).
        surf = self._pygame.surfarray.make_surface(img.swapaxes(0, 1))
        surf = self._pygame.transform.smoothscale(surf, (w, h))
        screen.blit(surf, (x, y))
      except Exception:
        pass
    text = self._font.render(label, True, _HUD_TEXT_COLOR)
    screen.blit(text, (x + _HUD_TEXT_OFFSET[0], y + _HUD_TEXT_OFFSET[1]))


def maybe_start_window(world: MetaDriveWorld) -> HILWindow | None:
  """Convenience: only start a window if the world actually exposes the HIL panes."""
  if not getattr(world, "hil_two_vehicle", False):
    return None
  if world.topdown_image is None or world.lead_road_image is None:
    return None
  win = HILWindow(world)
  win.start()
  return win


def update_telemetry_from_sm(window: HILWindow, ego_sm, lead_sm) -> None:
  """Pull a small set of fields out of each role's SubMaster for the HUD overlay."""
  if window is None:
    return
  if ego_sm is not None and ego_sm.valid.get('carState', False):
    window.ego_telemetry = {
      "vEgo": float(ego_sm['carState'].vEgo),
      "hccc": "active" if ego_sm.valid.get('selfdriveState', False) and ego_sm['selfdriveState'].active else "off",
    }
  if lead_sm is not None and lead_sm.valid.get('carState', False):
    window.lead_telemetry = {
      "vLead": float(lead_sm['carState'].vEgo),
    }

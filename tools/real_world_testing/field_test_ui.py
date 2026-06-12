#!/usr/bin/env python3
"""Tk control panel for real-world (in-car) HC3 tests.

A thin front-end over field_test.py: every button runs the corresponding
subcommand and streams its output into the log pane, so anything the UI does
can also be done (and scripted) from the command line. Run it on the laptop
that is joined to the ego's hotspot:

    ./tools/real_world_testing/launch_ui.sh
"""
from __future__ import annotations

import json
import os
import queue
import shlex
import signal
import subprocess
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from tools.hcc_v2v.launcher_ui import ManagedProcess  # noqa: E402

CONFIG_PATH = Path.home() / ".openpilot_hcc_field_test.json"
FIELD_TEST = "tools/real_world_testing/field_test.py"
RUNS_DIR = Path(__file__).resolve().parent / "runs"


class FieldTestUI:
  def __init__(self, root: tk.Tk):
    self.root = root
    self.root.title("HC3 Field Test")
    self.root.geometry("980x720")

    self.log_queue: "queue.Queue[tuple[str, str]]" = queue.Queue()
    self.run_status = tk.StringVar(value="Idle")

    self.ego_host_var = tk.StringVar(value="10.42.0.1")
    self.lead_host_var = tk.StringVar(value="")
    self.scn_var = tk.StringVar(value="48")
    self.duration_var = tk.StringVar(value="")
    self.start_delay_var = tk.StringVar(value="5")
    self.end_var = tk.StringVar(value="stop")
    self.loop_var = tk.BooleanVar(value=False)
    self.notes_var = tk.StringVar(value="")

    self.proc = ManagedProcess("field_test", self.log_queue, self._set_status)

    self._load_config()
    self._build_layout()
    self._poll_logs()
    self.root.protocol("WM_DELETE_WINDOW", self._on_close)

  # ---------------------------------------------------------------- layout

  def _build_layout(self) -> None:
    self.root.columnconfigure(0, weight=1)
    self.root.rowconfigure(2, weight=1)

    settings = ttk.LabelFrame(self.root, text="Devices & scenario", padding=8)
    settings.grid(row=0, column=0, sticky="ew", padx=12, pady=(12, 6))
    for col in (1, 3, 5):
      settings.columnconfigure(col, weight=1)

    self._entry(settings, 0, 0, "Ego host", self.ego_host_var)
    self._entry(settings, 0, 2, "Lead host", self.lead_host_var)
    self._entry(settings, 0, 4, "Scenario", self.scn_var)
    self._entry(settings, 1, 0, "Duration (s)", self.duration_var)
    self._entry(settings, 1, 2, "Start delay (s)", self.start_delay_var)
    ttk.Label(settings, text="At profile end").grid(row=1, column=4, sticky="w", padx=(12, 6), pady=4)
    end_box = ttk.Combobox(settings, textvariable=self.end_var, state="readonly", values=("stop", "hold"), width=8)
    end_box.grid(row=1, column=5, sticky="w", pady=4)
    ttk.Checkbutton(settings, text="Loop profile", variable=self.loop_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=4)
    self._entry(settings, 2, 2, "Run notes", self.notes_var, span=3)

    actions = ttk.Frame(self.root, padding=(12, 0))
    actions.grid(row=1, column=0, sticky="ew")
    buttons = [
      ("Find lead IP", self.find_lead),
      ("Preflight check", self.preflight),
      ("START RUN", self.start_run),
      ("End run (collect data)", self.end_run),
      ("ABORT (kill remote)", self.abort),
      ("Open runs folder", self.open_runs),
    ]
    for index, (label, callback) in enumerate(buttons):
      ttk.Button(actions, text=label, command=callback).grid(row=0, column=index, sticky="ew", padx=3, pady=4)
      actions.columnconfigure(index, weight=1)

    status_bar = ttk.Frame(self.root, padding=(12, 0))
    status_bar.grid(row=3, column=0, sticky="ew", pady=(0, 4))
    ttk.Label(status_bar, text="Status:").pack(side="left")
    ttk.Label(status_bar, textvariable=self.run_status).pack(side="left", padx=6)

    log_frame = ttk.LabelFrame(self.root, text="Log", padding=8)
    log_frame.grid(row=2, column=0, sticky="nsew", padx=12, pady=(6, 4))
    log_frame.columnconfigure(0, weight=1)
    log_frame.rowconfigure(0, weight=1)
    self.log_text = tk.Text(log_frame, wrap="word", height=22)
    self.log_text.grid(row=0, column=0, sticky="nsew")
    scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
    scrollbar.grid(row=0, column=1, sticky="ns")
    self.log_text.configure(yscrollcommand=scrollbar.set, state="disabled")

  def _entry(self, frame, row: int, column: int, label: str, variable: tk.StringVar, span: int = 1) -> None:
    ttk.Label(frame, text=label).grid(row=row, column=column, sticky="w", padx=(0 if column == 0 else 12, 6), pady=4)
    ttk.Entry(frame, textvariable=variable).grid(row=row, column=column + 1, columnspan=span, sticky="ew", pady=4)

  # ---------------------------------------------------------------- actions

  def _common_flags(self) -> list[str]:
    flags = ["--ego_host", self.ego_host_var.get().strip() or "10.42.0.1"]
    lead = self.lead_host_var.get().strip()
    if lead:
      flags += ["--lead_host", lead]
    return flags

  def _launch(self, subcommand: list[str]) -> None:
    if self.proc.is_running():
      messagebox.showwarning("Busy", "A field_test command is already running — end or abort it first.")
      return
    self._save_config()
    command = " ".join(shlex.quote(part) for part in
                       [sys.executable, FIELD_TEST, *subcommand])
    self.proc.start(command, str(REPO_ROOT))

  def find_lead(self) -> None:
    self._launch(["find-lead", "--ego_host", self.ego_host_var.get().strip() or "10.42.0.1"])

  def preflight(self) -> None:
    if not self._require_lead():
      return
    self._launch(["check", *self._common_flags()])

  def start_run(self) -> None:
    if not self._require_lead():
      return
    scn = self.scn_var.get().strip()
    if not scn.isdigit():
      messagebox.showerror("Invalid scenario", "Scenario must be a positive integer (e.g. 48).")
      return
    if not messagebox.askokcancel(
        "Start run",
        f"Start scenario {scn} on the lead device?\n\n"
        "The ego will follow the virtual lead once engaged.\n"
        "Make sure the driver is ready."):
      return
    cmd = ["run", *self._common_flags(), "--scn", scn,
           "--start_delay", self.start_delay_var.get().strip() or "5",
           "--end", self.end_var.get()]
    duration = self.duration_var.get().strip()
    if duration:
      cmd += ["--duration", duration]
    if self.loop_var.get():
      cmd.append("--loop")
    notes = self.notes_var.get().strip()
    if notes:
      cmd += ["--notes", notes]
    self._launch(cmd)

  def end_run(self) -> None:
    """Graceful end: SIGTERM the run; field_test stops the devices and collects.

    Deliberately NOT ManagedProcess.stop() — that SIGKILLs after 5 s, which
    would interrupt the data collection."""
    if not self.proc.is_running():
      messagebox.showinfo("No run", "No field_test command is running.")
      return
    assert self.proc.process is not None
    os.killpg(self.proc.process.pid, signal.SIGTERM)
    self.log_queue.put(("ui", "end requested — waiting for data collection to finish..."))

  def abort(self) -> None:
    """Emergency: kill remote processes via a separate abort invocation."""
    flags = self._common_flags()
    if "--lead_host" not in flags:
      messagebox.showerror("Missing lead host", "Lead host is required to abort.")
      return
    if self.proc.is_running():
      assert self.proc.process is not None
      os.killpg(self.proc.process.pid, signal.SIGTERM)
    subprocess.Popen([sys.executable, FIELD_TEST, "abort", *flags], cwd=str(REPO_ROOT))
    self.log_queue.put(("ui", "ABORT sent — virtual_lead is being killed; ego disengages via staleness."))

  def open_runs(self) -> None:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    opener = "open" if sys.platform == "darwin" else "xdg-open"
    subprocess.Popen([opener, str(RUNS_DIR)])

  def _require_lead(self) -> bool:
    if not self.lead_host_var.get().strip():
      messagebox.showerror("Missing lead host", "Set the lead device's hotspot IP (use Find lead IP).")
      return False
    return True

  # ---------------------------------------------------------------- plumbing

  def _set_status(self, name: str, running: bool) -> None:
    self.root.after(0, lambda: self.run_status.set("Running" if running else "Idle"))

  def _poll_logs(self) -> None:
    while True:
      try:
        source, message = self.log_queue.get_nowait()
      except queue.Empty:
        break
      self.log_text.configure(state="normal")
      self.log_text.insert("end", f"[{source}] {message}\n")
      self.log_text.see("end")
      self.log_text.configure(state="disabled")
    self.root.after(150, self._poll_logs)

  def _save_config(self) -> None:
    CONFIG_PATH.write_text(json.dumps({
      "ego_host": self.ego_host_var.get(), "lead_host": self.lead_host_var.get(),
      "scn": self.scn_var.get(), "duration": self.duration_var.get(),
      "start_delay": self.start_delay_var.get(), "end": self.end_var.get(),
    }, indent=2))

  def _load_config(self) -> None:
    if not CONFIG_PATH.is_file():
      return
    try:
      raw = json.loads(CONFIG_PATH.read_text())
    except (json.JSONDecodeError, OSError):
      return
    for var, key in ((self.ego_host_var, "ego_host"), (self.lead_host_var, "lead_host"),
                     (self.scn_var, "scn"), (self.duration_var, "duration"),
                     (self.start_delay_var, "start_delay"), (self.end_var, "end")):
      if raw.get(key):
        var.set(raw[key])

  def _on_close(self) -> None:
    if self.proc.is_running():
      if not messagebox.askyesno("Run in progress",
                                 "A run is still active. End it (with data collection) and close?"):
        return
      self.end_run()
    self.root.destroy()


def main() -> int:
  root = tk.Tk()
  style = ttk.Style(root)
  if "clam" in style.theme_names():
    style.theme_use("clam")
  FieldTestUI(root)
  root.mainloop()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import queue
import signal
import subprocess
import sys
import threading
import tkinter as tk
from dataclasses import asdict
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from tools.hcc_v2v.launcher_common import (
  DEFAULT_EGO_PREFIX,
  DEFAULT_LEAD_PREFIX,
  DEFAULT_RELAY_HOST,
  DEFAULT_RELAY_PORT,
  LauncherConfig,
  build_bridge_command,
  build_ego_command,
  build_lead_command,
  build_relay_command,
  build_terminal_command,
  validate_repo,
)


CONFIG_PATH = Path.home() / ".openpilot_hcc_v2v_launcher.json"
PROCESS_STOP_TIMEOUT_S = 5.0


class ManagedProcess:
  def __init__(self, name: str, log_queue: "queue.Queue[tuple[str, str]]", status_callback):
    self.name = name
    self.log_queue = log_queue
    self.status_callback = status_callback
    self.process: subprocess.Popen[str] | None = None
    self.reader_thread: threading.Thread | None = None

  def is_running(self) -> bool:
    return self.process is not None and self.process.poll() is None

  def start(self, command: str, cwd: str) -> None:
    if self.is_running():
      raise RuntimeError(f"{self.name} is already running")

    self.process = subprocess.Popen(
      ["/bin/bash", "-lc", command],
      cwd=cwd,
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      text=True,
      bufsize=1,
      preexec_fn=os.setsid,
    )
    self.status_callback(self.name, True)
    self.log_queue.put((self.name, f"$ {command}"))
    self.reader_thread = threading.Thread(target=self._read_output, daemon=True)
    self.reader_thread.start()

  def stop(self) -> None:
    if not self.is_running():
      self.process = None
      self.status_callback(self.name, False)
      return

    assert self.process is not None
    try:
      os.killpg(self.process.pid, signal.SIGTERM)
      self.process.wait(timeout=PROCESS_STOP_TIMEOUT_S)
    except subprocess.TimeoutExpired:
      os.killpg(self.process.pid, signal.SIGKILL)
      self.process.wait(timeout=PROCESS_STOP_TIMEOUT_S)
    finally:
      self.process = None
      self.status_callback(self.name, False)

  def _read_output(self) -> None:
    assert self.process is not None
    assert self.process.stdout is not None
    for line in self.process.stdout:
      self.log_queue.put((self.name, line.rstrip()))
    rc = self.process.wait()
    self.log_queue.put((self.name, f"[process exited with code {rc}]"))
    self.status_callback(self.name, False)
    self.process = None


class LauncherUI:
  def __init__(self, root: tk.Tk):
    self.root = root
    self.root.title("HCC V2V Sim Launcher")
    self.root.geometry("1080x760")

    self.log_queue: "queue.Queue[tuple[str, str]]" = queue.Queue()
    self.status_vars = {
      "relay": tk.StringVar(value="Stopped"),
      "ego": tk.StringVar(value="Stopped"),
      "lead": tk.StringVar(value="Stopped"),
      "bridge": tk.StringVar(value="Not opened"),
    }

    self.ego_repo_var = tk.StringVar(value=str(Path(__file__).resolve().parents[2]))
    self.lead_repo_var = tk.StringVar(value=str(Path(__file__).resolve().parents[3] / "openpilot-hcc-lead"))
    self.relay_host_var = tk.StringVar(value=DEFAULT_RELAY_HOST)
    self.relay_port_var = tk.StringVar(value=str(DEFAULT_RELAY_PORT))
    self.relay_log_csv_var = tk.StringVar(value=str(Path.home() / "hcc_v2v_relay.csv"))
    self.bridge_output_csv_var = tk.StringVar(value=str(Path.home() / "hcc_dual_sim.csv"))
    self.lead_prefix_var = tk.StringVar(value=DEFAULT_LEAD_PREFIX)
    self.ego_prefix_var = tk.StringVar(value=DEFAULT_EGO_PREFIX)
    self.ego_device_id_var = tk.StringVar(value="ego-sim")
    self.lead_device_id_var = tk.StringVar(value="lead-sim")
    self.scenario_var = tk.StringVar(value="")
    self.input_mode_var = tk.StringVar(value="keyboard")

    self.processes = {
      "relay": ManagedProcess("relay", self.log_queue, self._set_running_status),
      "ego": ManagedProcess("ego", self.log_queue, self._set_running_status),
      "lead": ManagedProcess("lead", self.log_queue, self._set_running_status),
    }

    self._load_config()
    self._build_layout()
    self._poll_logs()
    self.root.protocol("WM_DELETE_WINDOW", self._on_close)

  def _build_layout(self) -> None:
    self.root.columnconfigure(0, weight=1)
    self.root.rowconfigure(1, weight=1)

    controls = ttk.Frame(self.root, padding=12)
    controls.grid(row=0, column=0, sticky="nsew")
    controls.columnconfigure(0, weight=1)
    controls.columnconfigure(1, weight=1)

    self._build_paths_frame(controls)
    self._build_network_frame(controls)
    self._build_actions_frame(controls)
    self._build_status_frame(controls)

    log_frame = ttk.LabelFrame(self.root, text="Logs", padding=8)
    log_frame.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 12))
    log_frame.columnconfigure(0, weight=1)
    log_frame.rowconfigure(0, weight=1)

    self.log_text = tk.Text(log_frame, wrap="word", height=24)
    self.log_text.grid(row=0, column=0, sticky="nsew")
    scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
    scrollbar.grid(row=0, column=1, sticky="ns")
    self.log_text.configure(yscrollcommand=scrollbar.set, state="disabled")

  def _build_paths_frame(self, parent: ttk.Frame) -> None:
    frame = ttk.LabelFrame(parent, text="Repositories", padding=8)
    frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
    frame.columnconfigure(1, weight=1)

    self._add_path_row(frame, 0, "Ego repo", self.ego_repo_var)
    self._add_path_row(frame, 1, "Lead repo", self.lead_repo_var)

  def _build_network_frame(self, parent: ttk.Frame) -> None:
    frame = ttk.LabelFrame(parent, text="Deployment Settings", padding=8)
    frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))
    frame.columnconfigure(1, weight=1)
    frame.columnconfigure(3, weight=1)

    self._add_entry(frame, 0, 0, "Relay host", self.relay_host_var)
    self._add_entry(frame, 0, 2, "Relay port", self.relay_port_var)
    self._add_entry(frame, 1, 0, "Relay log CSV", self.relay_log_csv_var, width=36)
    self._add_browse_button(frame, 1, 2, self.relay_log_csv_var, save=True)
    self._add_entry(frame, 2, 0, "Bridge output CSV", self.bridge_output_csv_var, width=36)
    self._add_browse_button(frame, 2, 2, self.bridge_output_csv_var, save=True)
    self._add_entry(frame, 3, 0, "Ego prefix", self.ego_prefix_var)
    self._add_entry(frame, 3, 2, "Lead prefix", self.lead_prefix_var)
    self._add_entry(frame, 4, 0, "Ego device id", self.ego_device_id_var)
    self._add_entry(frame, 4, 2, "Lead device id", self.lead_device_id_var)
    self._add_entry(frame, 5, 0, "Scenario", self.scenario_var)

    ttk.Label(frame, text="Bridge input").grid(row=5, column=2, sticky="w", padx=(12, 6), pady=4)
    input_box = ttk.Combobox(frame, textvariable=self.input_mode_var, state="readonly", values=("keyboard", "logitech_wheel", "joystick"))
    input_box.grid(row=5, column=3, sticky="ew", pady=4)

  def _build_actions_frame(self, parent: ttk.Frame) -> None:
    frame = ttk.LabelFrame(parent, text="Actions", padding=8)
    frame.grid(row=1, column=0, sticky="nsew", padx=(0, 6))

    buttons = [
      ("Start relay", self.start_relay),
      ("Stop relay", self.stop_relay),
      ("Start ego", self.start_ego),
      ("Stop ego", self.stop_ego),
      ("Start lead", self.start_lead),
      ("Stop lead", self.stop_lead),
      ("Open bridge", self.open_bridge_terminal),
      ("Start all", self.start_all),
      ("Stop all", self.stop_all),
      ("Save config", self._save_config),
    ]

    for index, (label, callback) in enumerate(buttons):
      ttk.Button(frame, text=label, command=callback).grid(row=index // 2, column=index % 2, sticky="ew", padx=4, pady=4)

    for col in range(2):
      frame.columnconfigure(col, weight=1)

  def _build_status_frame(self, parent: ttk.Frame) -> None:
    frame = ttk.LabelFrame(parent, text="Status", padding=8)
    frame.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
    frame.columnconfigure(1, weight=1)

    row = 0
    for name in ("relay", "ego", "lead", "bridge"):
      ttk.Label(frame, text=name.capitalize()).grid(row=row, column=0, sticky="w", pady=4)
      ttk.Label(frame, textvariable=self.status_vars[name]).grid(row=row, column=1, sticky="w", pady=4)
      row += 1

    hint = (
      "Bridge opens in a terminal window so keyboard or wheel input still works.\n"
      "Relay, ego, and lead run in the background and stream logs here."
    )
    ttk.Label(frame, text=hint, justify="left").grid(row=row, column=0, columnspan=2, sticky="w", pady=(8, 0))

  def _add_path_row(self, frame: ttk.LabelFrame, row: int, label: str, variable: tk.StringVar) -> None:
    ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
    ttk.Entry(frame, textvariable=variable).grid(row=row, column=1, sticky="ew", padx=6, pady=4)
    ttk.Button(frame, text="Browse", command=lambda var=variable: self._browse_dir(var)).grid(row=row, column=2, pady=4)

  def _add_entry(self, frame: ttk.LabelFrame, row: int, column: int, label: str, variable: tk.StringVar, width: int = 18) -> None:
    ttk.Label(frame, text=label).grid(row=row, column=column, sticky="w", padx=(0 if column == 0 else 12, 6), pady=4)
    ttk.Entry(frame, textvariable=variable, width=width).grid(row=row, column=column + 1, sticky="ew", pady=4)

  def _add_browse_button(self, frame: ttk.LabelFrame, row: int, column: int, variable: tk.StringVar, save: bool = False) -> None:
    ttk.Button(frame, text="Browse", command=lambda var=variable, save_path=save: self._browse_file(var, save_path)).grid(row=row, column=column, padx=(12, 0), pady=4)

  def _browse_dir(self, variable: tk.StringVar) -> None:
    selected = filedialog.askdirectory(initialdir=variable.get() or str(Path.home()))
    if selected:
      variable.set(selected)

  def _browse_file(self, variable: tk.StringVar, save: bool) -> None:
    if save:
      selected = filedialog.asksaveasfilename(initialfile=Path(variable.get() or "output.csv").name)
    else:
      selected = filedialog.askopenfilename(initialfile=Path(variable.get()).name if variable.get() else "")
    if selected:
      variable.set(selected)

  def _current_config(self) -> LauncherConfig:
    return LauncherConfig(
      ego_repo=self.ego_repo_var.get().strip(),
      lead_repo=self.lead_repo_var.get().strip(),
      relay_host=self.relay_host_var.get().strip() or DEFAULT_RELAY_HOST,
      relay_port=int(self.relay_port_var.get().strip() or DEFAULT_RELAY_PORT),
      relay_log_csv=self.relay_log_csv_var.get().strip(),
      bridge_output_csv=self.bridge_output_csv_var.get().strip(),
      lead_prefix=self.lead_prefix_var.get().strip() or DEFAULT_LEAD_PREFIX,
      ego_prefix=self.ego_prefix_var.get().strip() or DEFAULT_EGO_PREFIX,
      ego_device_id=self.ego_device_id_var.get().strip() or "ego-sim",
      lead_device_id=self.lead_device_id_var.get().strip() or "lead-sim",
      scenario=self.scenario_var.get().strip(),
      input_mode=self.input_mode_var.get().strip() or "keyboard",
    )

  def _validated_config(self) -> LauncherConfig:
    config = self._current_config()
    try:
      validate_repo(config.ego_repo)
      validate_repo(config.lead_repo)
    except ValueError as err:
      raise RuntimeError(str(err)) from err

    if not (1 <= config.relay_port <= 65535):
      raise RuntimeError("relay port must be between 1 and 65535")

    if config.scenario and not config.scenario.isdigit():
      raise RuntimeError("scenario must be blank or a positive integer")

    return config

  def start_relay(self) -> None:
    config = self._require_config()
    if config is None:
      return
    self._start_process("relay", build_relay_command(config), config.ego_repo)

  def stop_relay(self) -> None:
    self.processes["relay"].stop()

  def start_ego(self) -> None:
    config = self._require_config()
    if config is None:
      return
    self._start_process("ego", build_ego_command(config), config.ego_repo)

  def stop_ego(self) -> None:
    self.processes["ego"].stop()

  def start_lead(self) -> None:
    config = self._require_config()
    if config is None:
      return
    self._start_process("lead", build_lead_command(config), config.lead_repo)

  def stop_lead(self) -> None:
    self.processes["lead"].stop()

  def open_bridge_terminal(self) -> None:
    config = self._require_config()
    if config is None:
      return
    try:
      terminal_cmd = build_terminal_command("HCC V2V Bridge", build_bridge_command(config))
      subprocess.Popen(terminal_cmd, cwd=config.ego_repo)
      self.status_vars["bridge"].set("Opened terminal")
      self.log_queue.put(("bridge", f"$ {' '.join(terminal_cmd)}"))
    except Exception as err:
      messagebox.showerror("Bridge launch failed", str(err))

  def start_all(self) -> None:
    self.start_relay()
    self.root.after(400, self.start_ego)
    self.root.after(800, self.start_lead)
    self.root.after(1400, self.open_bridge_terminal)

  def stop_all(self) -> None:
    self.stop_lead()
    self.stop_ego()
    self.stop_relay()
    self.status_vars["bridge"].set("Stop bridge from its terminal")

  def _start_process(self, name: str, command: str, cwd: str) -> None:
    process = self.processes[name]
    try:
      process.start(command, cwd)
    except RuntimeError as err:
      messagebox.showerror(f"Unable to start {name}", str(err))

  def _require_config(self) -> LauncherConfig | None:
    try:
      config = self._validated_config()
      self._save_config()
      return config
    except Exception as err:
      messagebox.showerror("Invalid launcher settings", str(err))
      return None

  def _set_running_status(self, name: str, running: bool) -> None:
    value = "Running" if running else "Stopped"
    self.root.after(0, lambda: self.status_vars[name].set(value))

  def _append_log(self, source: str, message: str) -> None:
    self.log_text.configure(state="normal")
    self.log_text.insert("end", f"[{source}] {message}\n")
    self.log_text.see("end")
    self.log_text.configure(state="disabled")

  def _poll_logs(self) -> None:
    while True:
      try:
        source, message = self.log_queue.get_nowait()
      except queue.Empty:
        break
      self._append_log(source, message)
    self.root.after(150, self._poll_logs)

  def _save_config(self) -> None:
    config = asdict(self._current_config())
    CONFIG_PATH.write_text(json.dumps(config, indent=2))
    self.log_queue.put(("ui", f"saved config to {CONFIG_PATH}"))

  def _load_config(self) -> None:
    if not CONFIG_PATH.is_file():
      return
    try:
      raw = json.loads(CONFIG_PATH.read_text())
    except (json.JSONDecodeError, OSError):
      return

    for attr_name, value in (
      ("ego_repo_var", raw.get("ego_repo")),
      ("lead_repo_var", raw.get("lead_repo")),
      ("relay_host_var", raw.get("relay_host")),
      ("relay_port_var", str(raw.get("relay_port")) if raw.get("relay_port") is not None else None),
      ("relay_log_csv_var", raw.get("relay_log_csv")),
      ("bridge_output_csv_var", raw.get("bridge_output_csv")),
      ("lead_prefix_var", raw.get("lead_prefix")),
      ("ego_prefix_var", raw.get("ego_prefix")),
      ("ego_device_id_var", raw.get("ego_device_id")),
      ("lead_device_id_var", raw.get("lead_device_id")),
      ("scenario_var", raw.get("scenario")),
      ("input_mode_var", raw.get("input_mode")),
    ):
      if value:
        getattr(self, attr_name).set(value)

  def _on_close(self) -> None:
    if any(process.is_running() for process in self.processes.values()):
      should_close = messagebox.askyesno("Close launcher", "Stop relay, ego, and lead before closing?")
      if not should_close:
        return
      self.stop_all()
    self.root.destroy()


def main() -> int:
  root = tk.Tk()
  style = ttk.Style(root)
  if "clam" in style.theme_names():
    style.theme_use("clam")
  LauncherUI(root)
  root.mainloop()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

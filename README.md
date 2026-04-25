openpilot-hcc
=============

This repository contains the `openpilot-hcc` codebase.

This top-level README focuses on installing the repository onto a comma device. For local development and simulator setup, see:

- [tools/README.md](tools/README.md)
- [tools/sim/README.md](tools/sim/README.md)

## Purpose

Use this guide when you want to:

- connect to a comma device over SSH
- replace the device's existing `openpilot` checkout with this repository
- reboot into the updated checkout

## Branch-Specific HCC/V2V Behavior

This `hcc-ego` branch is the ego-vehicle side of the HC3 research stack.

- `V2V-only` mode can be enabled with the persistent `HCCV2VOnly` param or `HCC_V2V_ONLY=1`.
- In `V2V-only` mode, HC3 ignores radar lead input and uses fresh V2V packets as its only lead source.
- Pedal blending remains active in this mode. The final longitudinal command is `HC3 accel + manual pedal accel`, and the manual pedal gains are split between simulation and on-road use in `selfdrive/controls/lib/longcontrol.py`.
- If V2V goes stale or transport health drops, HC3 contribution falls away instead of silently falling back to radar. The branch logs the current V2V mode as one of `manual_only`, `v2v_waiting`, `v2v_active`, `v2v_fault_stale`, or `v2v_fault_transport`.
- `selfdrive/controls/controlsd.py` keeps `longActive` true in `V2V-only` mode even when V2V packets go stale, so the ego car degrades to manual pedal authority instead of resetting out of longitudinal control.

This branch also vendors the shared V2V transport snapshot under `selfdrive/controls/lib/vendor/` and expects the lead car and relay to use the same packet contract.

## Requirements

Before starting:

- create a GitHub account if you do not already have one
- ensure the comma device and your computer are on the same network
- enable SSH on the device at `Settings -> Network -> Advanced -> Enable SSH`

OpenSSH is already available on:

- Windows 10 and newer
- macOS

## 1. Open a Terminal

Use one of the following:

- Windows: PowerShell
- macOS: Terminal

Command Prompt is not recommended for this workflow.

## 2. Create an SSH Key

If you do not already have an SSH key configured for GitHub, generate one:

```bash
ssh-keygen -t ed25519 -f $HOME/.ssh/id_ed25519
```

When prompted for a passphrase, you may press Enter to leave it empty if that matches your preferred setup.

## 3. Copy the Public Key

Windows:

```powershell
Get-Content $HOME\.ssh\id_ed25519.pub | Set-Clipboard
```

macOS:

```bash
cat $HOME/.ssh/id_ed25519.pub | pbcopy
```

## 4. Add the Key to GitHub

Open [GitHub SSH settings](https://github.com/settings/keys), paste the public key into the `Key` field, choose a title, and select **Add SSH key**.

## 5. Verify GitHub SSH Access

Verify that GitHub recognizes your key:

```bash
ssh -T git@github.com
```

Expected result:

```text
Hi <your GitHub username>! You've successfully authenticated, but GitHub does not provide shell access.
```

## 6. Find the Device IP Address

On the device, confirm the current network and obtain the IP address from:

```text
Settings -> Network -> Advanced
```

## 7. Authorize Your GitHub Key on the Device

On the device:

```text
Settings -> Network -> SSH Keys
```

Select **Add**, enter your GitHub username, and confirm.

After a successful refresh:

- your GitHub username should appear in the SSH Keys list
- the **Add** button should change to **Remove**

If a different GitHub username is already present, remove it first if needed.

If you later add or rotate SSH keys on GitHub, repeat this step so the device refreshes the authorized keys.

## 8. Confirm SSH Is Enabled

Verify that:

```text
Settings -> Network -> Enable SSH
```

is enabled and shown in green.

## 9. Connect to the Device

Replace `555.555.555.555` with the device IP address:

```bash
ssh comma@555.555.555.555
```

On first connection, if SSH asks whether the host authenticity can be established, answer:

```text
yes
```

Once connected, you should see a shell prompt in `/data/openpilot`.

## 10. Replace the Device Checkout

If you intend to replace the existing device checkout, remove the current `/data/openpilot` directory and clone this repository in its place.

From the device shell:

```bash
cd /data
rm -rf openpilot
git clone --recurse-submodules -b hcc-ego https://github.com/ethanmathias/openpilot-hcc.git openpilot
```

## 11. Initialize Submodules and Pull LFS Objects

After cloning, initialize submodules and pull Git LFS objects:

```bash
cd /data/openpilot
git submodule update --init --recursive
git lfs pull
```

The `git lfs pull` step downloads neural network models, shared libraries, fonts, and boot binaries — it will take a few minutes. Do not skip it. Running `GIT_LFS_SKIP_SMUDGE=1` as a workaround leaves critical binaries (`updater_magic`, `third_party/*.so`, `*.onnx`) as empty stubs and will prevent openpilot from starting.

## 12. Configure the HCC V2V Relay

The ego device needs a UDP relay that both the ego car and lead car can reach. Run the relay on a laptop, server, or cloud instance with UDP port `19090` open.

On the relay host:

```bash
cd /path/to/openpilot-hcc
python3 tools/hcc_v2v/relay_server.py --host 0.0.0.0 --port 19090 --log_csv ~/hcc_v2v_relay.csv
```

In a second relay-host terminal, watch the relay log:

```bash
tail -f ~/hcc_v2v_relay.csv
```

Set the ego device to contact that relay. Replace `555.555.555.555` with the relay host IP address or DNS name:

```bash
echo -n "1" > /data/params/d/HCCV2VEnabled
echo -n "1" > /data/params/d/HCCV2VOnly
echo -n "hcc-ego" > /data/params/d/HCCV2VDeviceId
echo -n "555.555.555.555" > /data/params/d/HCCV2VRelayHost
echo -n "19090" > /data/params/d/HCCV2VRelayPort
```

Use this one-shot ego registration test before rebooting. It sends the same JSON hello shape the ego subscriber uses:

```bash
python3 - <<'PY'
import json
import socket

RELAY_HOST = "555.555.555.555"
RELAY_PORT = 19090

payload = json.dumps({
  "type": "hello",
  "device_id": "hcc-ego-test",
  "role": "ego",
}, separators=(",", ":"), sort_keys=True).encode("utf-8")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(payload, (RELAY_HOST, RELAY_PORT))
print(payload.decode())
PY
```

The relay CSV should show a `hello` row with role `ego`, device id `hcc-ego-test`, and reason `registered_peer`. After openpilot starts with V2V enabled, the ego side will keep sending periodic hello packets and will listen for lead packets forwarded by the relay.

## 13. Disable Automatic Updates and Clear Staged Updates

The device's built-in updater will attempt to pull from comma.ai's servers on every reboot. Because this is a custom research fork, those updates will always fail and block startup with an update error prompt. Disable the updater before rebooting:

```bash
echo -n "1" > /data/params/d/DisableUpdates
```

Also clear any lingering update failure alert from a previous boot:

```bash
rm -f /data/params/d/Offroad_UpdateFailed
```

**Critically**, clear the update staging directory. On every boot, `launch_chffrplus.sh` checks for a finalized staged update at `/data/safe_staging/finalized/.overlay_consistent`. If that file exists from a previous updater run — even one that fetched upstream comma.ai code — the boot script will swap that staged update back over your custom fork before openpilot starts. That upstream code expects a different AGNOS version, which triggers an AGNOS flash and a reboot loop. Removing the staging directory prevents this.

The `merged` subdirectory is an active OverlayFS mount and must be unmounted before the directory can be removed:

```bash
sudo umount /data/safe_staging/merged
sudo rm -rf /data/safe_staging
```

If `umount` reports that `merged` is not mounted, skip it and run only the `rm` line.

These settings are persistent and survive reboots. You only need to run them once per fresh install.

## 14. Reboot

Reboot the device to start from the updated checkout:

```bash
sudo reboot
```

## Notes

- The clone step above checks out the `hcc-ego` branch directly. This is the ego-vehicle side of the HC3 research stack.
- Removing `/data/openpilot` deletes the existing checkout on the device. Use that step only when you intend to replace it.
- Disabling updates and clearing the staging directory (step 13) is required for any custom fork. Without it, `launch_chffrplus.sh` may swap a previously staged upstream update back over your fork on every boot, which causes an AGNOS version mismatch and a reboot loop.
- For local setup, development tooling, and simulator usage, use the READMEs under `tools/`.

openpilot-hcc
=============

This repository contains the `openpilot-hcc` codebase.

This top-level README focuses on installing the repository onto a comma device. For local development and simulator setup, see:

- [tools/README.md](/Users/ethanmathias/Desktop/UVA/LinkLab/openpilot-hcc/tools/README.md)
- [tools/sim/README.md](/Users/ethanmathias/Desktop/UVA/LinkLab/openpilot-hcc/tools/sim/README.md)

## Purpose

Use this guide when you want to:

- connect to a comma device over SSH
- replace the device's existing `openpilot` checkout with this repository
- reboot into the updated checkout

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
git clone --recurse-submodules https://github.com/ethanmathias/openpilot-hcc.git openpilot
```

## 11. Pull Git LFS Objects

If the repository uses Git LFS, or if you want to ensure LFS-managed files are present, run:

```bash
cd /data/openpilot
git lfs pull
```

## 12. Reboot

Reboot the device to start from the updated checkout:

```bash
sudo reboot
```

## Notes

- The clone step above installs this repository into `/data/openpilot`, which is the expected path on the device.
- Removing `/data/openpilot` deletes the existing checkout on the device. Use that step only when you intend to replace it.
- For local setup, development tooling, and simulator usage, use the READMEs under `tools/`.

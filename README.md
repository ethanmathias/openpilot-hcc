## Before You Start

- You need a GitHub account. Make one if you don't already have one.
- Enable SSH on your comma device: `Settings -> Network -> Advanced -> Enable SSH`.
- Pre-installed OpenSSH Client on Windows 10+ and macOS: Both Windows 10 and newer, and macOS come with a pre-installed SSH client, so no additional software download or installation is required.

## Step 1: Open Terminal or PowerShell

- **Windows:** Open PowerShell by pressing the Start Menu and typing "PowerShell". Command Prompt is not a substitute.
- **macOS:** Open Terminal by opening Spotlight in the top-right corner and typing "Terminal".

## Step 2: Create and Upload SSH Key to GitHub

If you haven't already created an SSH key and uploaded it to GitHub, follow these steps:

1. Generate an SSH key:
```bash
   ssh-keygen -t ed25519 -f $HOME/.ssh/id_ed25519
```

   When prompted to "Enter passphrase (empty for no passphrase):", just press Enter for no passphrase.

2. Copy the public key to your clipboard:

   **Windows:**
```powershell
   Get-Content $HOME\.ssh\id_ed25519.pub | Set-Clipboard
```

   **macOS:**
```bash
   cat $HOME/.ssh/id_ed25519.pub | pbcopy
```

3. Add the SSH key to GitHub:

   Visit [GitHub SSH settings](https://github.com/settings/keys), paste the contents of your clipboard into "Key", give it a name of your choice in "Title", and press **Add SSH Key**.

## Step 3: Verify SSH Key with GitHub

Run the following command to check if GitHub can identify you with the private key on your local system:
```bash
ssh -T git@github.com
```

You should get a message saying: "Hi \<your GitHub username\>! You've successfully authenticated, but GitHub does not provide shell access."

## Step 4: Get the IP Address of Your Device

Make sure your C3 and your connecting device are connected to the same WiFi or network: `Settings [⚙️ icon] > Network > Advanced`.

## Step 5: Add SSH Key to Your Device

Go to `Settings [⚙️ icon] > Network [ > Advanced, if C3] > SSH Keys` and press **Add**.

Enter your GitHub username and press "⏎". You should see the SSH Keys option change to include your GitHub username with the **Add** button changed to **Remove**.

- If a GitHub username is already there, press **Remove** to make **Add** reappear.
- If you change or add new SSH keys on GitHub, repeat this step to refresh the authorized SSH keys data on the device.

## Step 6: Enable SSH

Make sure `Settings [⚙️ icon] > Network [ > Advanced, if C3] > Enable SSH` is enabled. It should be green.

## Step 7: Connect via SSH

Run the following command, replacing `555.555.555.555` with the IP address you discovered earlier:
```bash
ssh comma@555.555.555.555
```

You should see a blue-ish prompt with "/data/openpilot", confirming you are connected.

- If you get an "authenticity of host cannot be determined" message, answer `yes`.

## Clone openpilot-hcc

1. Navigate to the data directory and remove existing openpilot:
```bash
   cd /data
   rm -rf openpilot
```

2. Clone the repository (Default branch assumed; no specific branch specified in the repo):
```bash
   git clone --recurse-submodules https://github.com/ethanmathias/openpilot-hcc.git openpilot
```

## Git LFS

If the repository uses Git LFS (recommended to run just in case):
```bash
cd openpilot
git lfs pull
```

## Reboot
```bash
sudo reboot
```
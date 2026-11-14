#!/usr/bin/env bash
# Copy the Windows build to the handheld over SSH (OpenSSH server on Windows).
#   scripts/deploy_ally.sh user@host [ssh-key]
# Files land in %USERPROFILE%\ArmPi_Ally_Controller\ on the device.
set -euo pipefail
cd "$(dirname "$0")/.."
target="${1:?usage: deploy_ally.sh user@host [ssh-key]}"
opts=(-o BatchMode=yes -o ConnectTimeout=10)
if [[ $# -ge 2 ]]; then opts+=(-i "$2" -o IdentitiesOnly=yes); fi
ssh "${opts[@]}" "$target" 'New-Item -ItemType Directory -Force "$env:USERPROFILE\ArmPi_Ally_Controller" | Out-Null'
scp "${opts[@]}" build-win/armpi-ally.exe build-win/armpi-ally-cli.exe "$target:ArmPi_Ally_Controller/"
echo "deployed to $target:%USERPROFILE%\\ArmPi_Ally_Controller"

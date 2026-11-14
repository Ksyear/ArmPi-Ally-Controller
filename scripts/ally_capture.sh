#!/usr/bin/env bash
# Save the screen of the app that is running on the handheld and copy it here (tests and README pictures).
#   scripts/ally_capture.sh user@host [ssh-key] [out-dir] [shot | burst N SECONDS]
# shot  -> out-dir/ally_<time>.jpg (full size)
# burst -> out-dir/ally_<time>.gif (N half-size frames, one every SECONDS; needs ffmpeg)
# The app must be running: it watches %LOCALAPPDATA%\ArmPi_Ally_Controller\capture.request (also F12 / diag tab).
set -euo pipefail
target="${1:?usage: ally_capture.sh user@host [ssh-key] [out-dir] [shot | burst N SECONDS]}"
key="${2:-}"
out="${3:-captures}"
mode="${4:-shot}"
n="${5:-30}"
secs="${6:-0.2}"
opts=(-o BatchMode=yes -o ConnectTimeout=10)
if [[ -n "$key" ]]; then opts+=(-i "$key" -o IdentitiesOnly=yes); fi
[[ "$n" =~ ^[0-9]+$ ]] || { echo "N must be a number" >&2; exit 2; }
[[ "$secs" =~ ^[0-9.]+$ ]] || { echo "SECONDS must be a number" >&2; exit 2; }
req="shot"
count=1
if [[ "$mode" == "burst" ]]; then
  req="burst $n $secs"
  count="$n"
fi
wait_s=$(python3 -c "print(int($count * $secs + 8))")
mkdir -p "$out"

# Ask the app for a capture and wait until all frames are written; prints the new file names.
names=$(ssh "${opts[@]}" "$target" "\$d = Join-Path \$env:LOCALAPPDATA 'ArmPi_Ally_Controller'; \
if (!(Get-Process -Name 'armpi-ally' -ErrorAction SilentlyContinue)) { 'NOAPP'; exit }; \
\$c = Join-Path \$d 'captures'; \
\$before = @(Get-ChildItem \$c -Filter 'cap_*.bmp' -ErrorAction SilentlyContinue | ForEach-Object Name); \
Set-Content -Path (Join-Path \$d 'capture.request') -Value '$req' -NoNewline; \
\$t0 = Get-Date; \
do { Start-Sleep -Milliseconds 300; \
  \$new = @(Get-ChildItem \$c -Filter 'cap_*.bmp' -ErrorAction SilentlyContinue | Where-Object { \$before -notcontains \$_.Name } | Sort-Object Name) \
} while (\$new.Count -lt $count -and ((Get-Date) - \$t0).TotalSeconds -lt $wait_s); \
Start-Sleep -Milliseconds 500; \$new | ForEach-Object { \$_.Name }" | tr -d '\r')
if [[ "$names" == *NOAPP* ]]; then
  echo "앱이 실행 중이 아님(armpi-ally.exe)" >&2
  exit 1
fi
if [[ -z "$names" ]]; then
  echo "새 화면이 저장되지 않음" >&2
  exit 1
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
for f in $names; do
  scp -q "${opts[@]}" "$target:AppData/Local/ArmPi_Ally_Controller/captures/$f" "$tmp/$f"
done
stamp="$(date +%Y%m%d_%H%M%S)"
if [[ "$mode" == "burst" ]]; then
  fps=$(python3 -c "print(round(1.0 / $secs, 3))")
  ffmpeg -loglevel error -y -framerate "$fps" -pattern_type glob -i "$tmp/cap_*.bmp" \
    -vf "scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer" \
    "$out/ally_$stamp.gif"
  echo "$out/ally_$stamp.gif ($(echo "$names" | wc -w | tr -d ' ') frames)"
else
  first=$(echo "$names" | head -1)
  if command -v sips >/dev/null; then sips -s format jpeg -s formatOptions 85 "$tmp/$first" --out "$out/ally_$stamp.jpg" >/dev/null
  else ffmpeg -loglevel error -y -i "$tmp/$first" -q:v 3 "$out/ally_$stamp.jpg"; fi
  echo "$out/ally_$stamp.jpg"
fi
# The copies are here now: remove them from the handheld.
list=$(echo $names | sed "s/ /','/g")
ssh "${opts[@]}" "$target" "\$c = Join-Path \$env:LOCALAPPDATA 'ArmPi_Ally_Controller\\captures'; foreach (\$f in @('$list')) { Remove-Item (Join-Path \$c \$f) -ErrorAction SilentlyContinue }" >/dev/null

#!/usr/bin/env bash
set -euo pipefail

ozone=/opt/SEGGER/Ozone_V350b/Ozone
jlink=/opt/SEGGER/JLink_V940/libjlinkarm.so.9.40.0

if [[ ! -x "$ozone" || ! -r "$jlink" ]]; then
  echo "找不到 Ozone 或 J-Link V9.40 库，请检查安装路径。" >&2
  exit 1
fi

export LD_PRELOAD="$jlink${LD_PRELOAD:+ $LD_PRELOAD}"
exec "$ozone" "$@"

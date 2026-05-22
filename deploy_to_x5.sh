#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  ./deploy_to_x5.sh whr@X5_IP [--remote-ws ~/tros_ws] [--with-weed-locator]

Examples:
  ./deploy_to_x5.sh whr@192.168.153.10
  ./deploy_to_x5.sh whr@192.168.153.10 --with-weed-locator

This script copies only source packages to the X5/RDK target.
It never copies build/, install/, or log/.
USAGE
}

if [[ $# -gt 0 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"
REMOTE="$1"
REMOTE_WS="~/tros_ws"
WITH_WEED_LOCATOR=0
shift

while [[ $# -gt 0 ]]; do
  case "$1" in
    --remote-ws)
      REMOTE_WS="${2:?missing value for --remote-ws}"
      shift 2
      ;;
    --with-weed-locator)
      WITH_WEED_LOCATOR=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

cd "${WS_DIR}"

packages=(
  "src/gps/handsfree_rtk"
  "src/gps/gps_waypoint_nav"
  "src/base/diff_drive_udp"
)

if [[ "${WITH_WEED_LOCATOR}" == "1" ]]; then
  packages+=("src/weed_locator")
fi

missing=()
for pkg in "${packages[@]}"; do
  if [[ ! -d "${pkg}" ]]; then
    missing+=("${pkg}")
  fi
done

if [[ ${#missing[@]} -gt 0 ]]; then
  echo "Missing source directories:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo "Run this script from the development workspace that contains these packages." >&2
  exit 1
fi

echo "Creating remote src directory on ${REMOTE}:${REMOTE_WS}/src"
ssh "${REMOTE}" "mkdir -p ${REMOTE_WS}/src"

echo "Syncing packages:"
printf '  - %s\n' "${packages[@]}"

rsync -avz --delete \
  --exclude build \
  --exclude install \
  --exclude log \
  "${packages[@]}" \
  "${REMOTE}:${REMOTE_WS}/src/"

echo
echo "Done. On the X5/RDK target, run:"
echo "  cd ${REMOTE_WS}"
echo "  ./run_stack_on_x5.sh --build-only"

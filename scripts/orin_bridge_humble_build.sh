#!/usr/bin/env bash
# Orin (arm64) build of docker/bridge_humble, instrumented for PHB-A0..A5 (13 Oct 2026).
# Predictions: Claude_Research humble_ros1_bridge_check.md §5 -- registered BEFORE this ran.
# Run ON THE ORIN, from the repo root:   scripts/orin_bridge_humble_build.sh
# Needs an uplink (docker pull, apt, git clone). NO ROBOT is involved.
# Writes logs/bridge_humble/orin/<stamp>/: preflight, build log, 5 s memory+disk samples, tegrastats, summary.
# Abort: / free < 3 GB mid-build -> the build is killed, not waited out.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"; cd "$REPO"
STAMP=$(date -u +%Y%m%dT%H%M%SZ); OUT="$REPO/logs/bridge_humble/orin/$STAMP"; mkdir -p "$OUT"
S="$OUT/summary.txt"; log() { echo "$*" | tee -a "$S"; }
avail_gb() { df -BG --output=avail / | tail -1 | tr -dc 0-9; }

log "$(date -u +%FT%TZ) start; host $(hostname) $(uname -m); repo $(git rev-parse --short HEAD) $(git status --porcelain | wc -l) dirty"
# --- preflight, every output to a file, including the empty ones ---
for c in "date -u" "df -h /" "free -h" "zramctl" "swapon --show" "nvpmodel -q" "docker version" "docker images" \
         "cat /etc/nv_tegra_release" "ip -br addr" "ip route" "cat /proc/sys/net/ipv4/ip_forward"; do
  f="$OUT/pre_$(echo "$c" | tr -c 'a-z0-9' _ | sed 's/_*$//').txt"; { echo "\$ $c"; $c; } > "$f" 2>&1; done
log "preflight: / avail $(avail_gb) GB; mem $(free -m | awk '/^Mem:/{print $2" MB total, "$3" MB used"}'); nproc $(nproc); free -g total $(free -g | awk '/^Mem:/{print $2}')"
if [ "$(avail_gb)" -lt 6 ]; then log "REFUSING: / has $(avail_gb) GB free, PHB-A5 says the build needs >= 6 GB"; exit 2; fi
if ! curl -sS -m 10 -o /dev/null https://github.com; then log "REFUSING: no uplink (github.com unreachable)"; exit 2; fi

# --- samplers: bounded (gotcha 15), killed on exit ---
( for i in $(seq 1 2880); do echo "$(date -u +%T) $(free -m | awk '/^Mem:/{print $3}') $(free -m | awk '/^Swap:/{print $3}') $(avail_gb)"; sleep 5; done ) \
  > "$OUT/mem_swap_disk.txt" 2>&1 &   # columns: time, mem used MB, swap used MB, / avail GB  (4 h max)
SAMPLER=$!
TEGRA=""; if command -v tegrastats >/dev/null; then tegrastats --interval 5000 > "$OUT/tegrastats.txt" 2>&1 & TEGRA=$!; fi
cleanup() { kill $SAMPLER $TEGRA 2>/dev/null; }; trap cleanup EXIT

# --- the build (no pipe: the exit code is the build's -- gotcha 9b) ---
T0=$(date +%s)
docker build --network host --progress=plain -t bridge_humble:exp -f docker/bridge_humble/Dockerfile . > "$OUT/build.txt" 2>&1 &
BPID=$!
while kill -0 $BPID 2>/dev/null; do
  if [ "$(avail_gb)" -lt 3 ]; then log "ABORT $(date -u +%T): / free $(avail_gb) GB < 3 -- killing the build"; kill $BPID; break; fi
  sleep 10; done
wait $BPID; RC=$?; T1=$(date +%s)
log "$(date -u +%FT%TZ) docker build rc=$RC wall $(( (T1-T0)/60 ))m$(( (T1-T0)%60 ))s"

# --- read the predictions' quantities out of the artifacts ---
log "PHB-A1 $(grep -m1 -oE 'building ros1_bridge with -j[0-9]+ .*' "$OUT/build.txt")"
log "PHB-A2 $(grep -m1 -oE 'Finished <<< ros1_bridge \[[^]]*\]' "$OUT/build.txt") | $(grep -E '^#[0-9]+ [0-9.]+ (real|user|sys)' "$OUT/build.txt" | awk '{print $3, $4}' | tr '\n' ' ')"
log "PHB-A3 mem used MB: first $(head -1 "$OUT/mem_swap_disk.txt" | awk '{print $2}'), peak $(sort -k2 -n "$OUT/mem_swap_disk.txt" | tail -1 | awk '{print $2" at "$1}'); swap peak MB $(sort -k3 -n "$OUT/mem_swap_disk.txt" | tail -1 | awk '{print $3}'); cc1plus killed: $(grep -c 'Killed signal terminated program' "$OUT/build.txt")"
log "PHB-A4 $(grep -v '\$(' "$OUT/build.txt" | grep -m1 -oE 'PHB-A4 arch=.*') | copied: $(grep -cE "^#[0-9]+ [0-9.]+ '/usr/lib/x86_64-linux-gnu/pkgconfig/" "$OUT/build.txt")"
log "PHB-A5 image $(docker images bridge_humble:exp --format '{{.Size}} {{.CreatedSince}}'); / avail after $(avail_gb) GB; min / avail during $(sort -k4 -n "$OUT/mem_swap_disk.txt" | head -1 | awk '{print $4}') GB"
[ $RC -eq 0 ] && log "PHB-A0 build completed" || log "PHB-A0 build FAILED rc=$RC -- read $OUT/build.txt"
log "next (PHB-A6/A7): GO2_CABLE_IFACE=none scripts/bridge_loopback_test.sh C bridge_humble:exp bridge_humble:exp"

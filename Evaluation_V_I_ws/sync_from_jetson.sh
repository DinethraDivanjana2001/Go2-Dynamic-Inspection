#!/bin/bash
# sync_from_jetson.sh
# Pull ALL captured data from Jetson to local machine.
#
# Usage:  bash sync_from_jetson.sh
#
# Syncs:
#   Jetson: ~/Documents/Visual_Inspection_ws/evaluation/  →  local: eval_dataset/
#   Jetson: ~/Documents/Visual_Inspection_ws/captures/    →  local: data/captures_from_jetson/

JETSON_USER="rgen"
JETSON_IP="192.168.8.181"
JETSON_BASE="~/Documents/Visual_Inspection_ws"

LOCAL_BASE="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_EVAL="${LOCAL_BASE}/Evaluation_V_I_ws/eval_dataset"
LOCAL_CAPTURES="${LOCAL_BASE}/data/captures_from_jetson"

echo ""
echo "══════════════════════════════════════════════════════"
echo "  SYNC FROM JETSON  →  LOCAL"
echo "══════════════════════════════════════════════════════"
echo "  Jetson:  ${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}"
echo "  Local:   ${LOCAL_BASE}"
echo ""

mkdir -p "${LOCAL_EVAL}"
mkdir -p "${LOCAL_CAPTURES}"

# ── 1. Evaluation dataset (collect_dataset.py output) ─────────────────────────
echo "[ 1/2 ]  Syncing evaluation/ ..."
rsync -avz --progress \
    "${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}/evaluation/" \
    "${LOCAL_EVAL}/"
echo ""

# ── 2. Captures (all pipeline + MQTT captures) ────────────────────────────────
echo "[ 2/2 ]  Syncing captures/ ..."
rsync -avz --progress \
    "${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}/captures/" \
    "${LOCAL_CAPTURES}/"
echo ""

# ── Summary ───────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════"
echo "  DONE"
echo ""
echo "  Evaluation dataset:"
find "${LOCAL_EVAL}" -name "*.jpg" 2>/dev/null | wc -l | xargs -I{} echo "    {} images"
echo "    CSV: ${LOCAL_EVAL}/capture_log.csv"
echo ""
echo "  Captures (MQTT/pipeline):"
find "${LOCAL_CAPTURES}" -name "*.jpg" 2>/dev/null | wc -l | xargs -I{} echo "    {} images"
echo "══════════════════════════════════════════════════════"
echo ""

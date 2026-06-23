#!/usr/bin/env bash
# SAP top-phase vs mini-TLAS vs regular vs GPU-top — real default scene
# (Human static, cloth tess 50). Profiler 30 frames.
# Conditions (top-phase mode via YSIM_SAP, sub-object via YSIM_SUBOBJECT):
#   regular   : single-root BVH        (no env)
#   mini_s<s> : sub-object + mini-TLAS (YSIM_SUBOBJECT=s)
#   sap_s<s>  : sub-object + CPU SAP   (YSIM_SUBOBJECT=s YSIM_SAP=1)
#   gpu_s<s>  : sub-object + GPU brute (YSIM_SUBOBJECT=s YSIM_SAP=2)
# INTERLEAVED across repeats so thermal/load drift averages over all modes
# (refit, which the top phase can't affect, must stay flat across modes —
# it's the noise-floor sentinel).
set -uo pipefail
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
EXP="profiles/experiment/sap-topphase-2026-06-23"; BIN=./build/ysim; TPL="$EXP/_template.scene.json"
REPEATS=${REPEATS:-8}; SPLITS=${SPLITS:-"1 2 3"}

one() { # tag, sub, sap, r
  local tag="$1" sub="$2" sap="$3" r="$4"
  local csv="$EXP/${tag}_r${r}.csv" cfg="$EXP/${tag}_r${r}.run.json"
  python3 -c "import json,sys;d=json.load(open('$TPL'));d['profile']['output_path']=sys.argv[1];json.dump(d,open(sys.argv[2],'w'),indent=2)" "$csv" "$cfg"
  if env ${sub:+YSIM_SUBOBJECT=$sub} ${sap:+YSIM_SAP=$sap} "$BIN" --scene "$cfg" >/dev/null 2>&1; then
    echo "  ok   $tag r$r"; else echo "  CRASH $tag r$r"; rm -f "$csv"; fi
  rm -f "$cfg"
}

for r in $(seq 1 "$REPEATS"); do
  echo "[round $r]"
  one regular "" "" "$r"
  for s in $SPLITS; do
    one "mini_s$s" "$s" ""  "$r"
    one "sap_s$s"  "$s" "1" "$r"
    one "gpu_s$s"  "$s" "2" "$r"
  done
done
echo "DONE"

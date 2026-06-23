#!/usr/bin/env bash
# SAP top-phase vs mini-TLAS vs regular BVH — real default scene (Human static,
# cloth tess 50). Drives the profiler (30 frames) over each condition × REPEATS.
#   regular        : single-root BVH         (no env)
#   mini_s<s>      : subobject + mini-TLAS    (YSIM_SUBOBJECT=s)
#   sap_s<s>       : subobject + CPU SAP top  (YSIM_SUBOBJECT=s YSIM_SAP=1)
set -uo pipefail   # NOT -e: tolerate the occasional contact-overflow crash on a
                   # single repeat; median across surviving repeats stays robust.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
EXP="profiles/experiment/sap-topphase-2026-06-23"
BIN=./build/ysim
TPL="$EXP/_template.scene.json"
REPEATS=${REPEATS:-8}
SPLITS=${SPLITS:-"1 2 3"}

gen() { # case_tag, sub(s|""), sap(0|1), csvpath
  python3 -c "
import json,sys
d=json.load(open('$TPL'))
d['profile']['output_path']=sys.argv[1]
json.dump(d,open(sys.argv[2],'w'),indent=2)
" "$1" "$2"
}

run() { # tag, env-sub, env-sap
  local tag="$1" sub="$2" sap="$3"
  for r in $(seq 1 "$REPEATS"); do
    local csv="$EXP/${tag}_r${r}.csv" cfg="$EXP/${tag}_r${r}.run.json"
    gen "$csv" "$cfg"
    if env ${sub:+YSIM_SUBOBJECT=$sub} ${sap:+YSIM_SAP=$sap} "$BIN" --scene "$cfg" >/dev/null 2>&1; then
      echo "  $tag r$r -> $(basename "$csv")"
    else
      echo "  $tag r$r -> CRASHED (skip)"; rm -f "$csv"
    fi
    rm -f "$cfg"
  done
}

echo "[regular]";  run regular "" ""
for s in $SPLITS; do
  echo "[mini_s$s]"; run "mini_s$s" "$s" ""
  echo "[sap_s$s]";  run "sap_s$s"  "$s" "1"
done
echo "DONE"

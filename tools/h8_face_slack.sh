#!/usr/bin/env bash
# **面ごとに「OCC からの離れ」と「ファイル自身のたるみ」を並べます**（4-666。
# 測ったのは 4-664／4-665）。
#
# **なぜ口にするか**: この照合は 3 段あり、**どの段にも罠が 1 つずつ**
# あります。**散文で書くと腐ります**（4-577 で `h8_ports.sh` を置いたのと
# 同じ理屈）。
#
#   ① 点を書き出す   `ZENITH_FACE_POINTS=... read_volume_convergence_probe`
#   ② OCC に聞く     `tools/occ_face_distance.py`
#                    **罠**: `occ_face_area_reference.py` では終わりません
#                    （点 0 個でも 30 分超。37 枚を tessellate(1e-5)）
#   ③ たるみを出す   `ZENITH_ROUGHNESS_ALL=1 roughness_convergence_probe`
#                    **罠**: 既定は「動く面」だけ印字（4-665）
#
# **そして、読むときの罠**——**この測り方には床があります**。
# `distToShape` は 5.4e-10 より細かく返しません。**たるみが 1e-15 台の
# 平らな面は、比べられません**——**「超えた」ではなく「比較が成立しない」**。
#
# 使い方: bash tools/h8_face_slack.sh [出力先ディレクトリ]
#
# **FreeCAD 1.1 が要ります。** **門ではありません**——30 分を超えます。
set -uo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-target/face_slack}"
mkdir -p "$OUT"

PY="${ZENITH_PYTHON:-}"
if [ -z "$PY" ]; then
  for c in py python3 python; do command -v "$c" > /dev/null 2>&1 && { PY="$c"; break; }; done
fi
[ -z "$PY" ] && { echo "Python がありません。"; exit 2; }

EXE=./target/release/examples
for e in read_volume_convergence_probe roughness_convergence_probe; do
  if [ ! -x "$EXE/$e" ] && [ ! -f "$EXE/$e.exe" ]; then
    echo "**$e がありません。** 先に: cargo build --release -p zenith_algo --examples"
    exit 1
  fi
done

echo "== ① 面の上の点を書き出します =="
ZENITH_FACE_POINTS="$OUT/points.txt" "$EXE/read_volume_convergence_probe" > "$OUT/rvc.txt" 2>&1 \
  || { echo "WRONG 1 read_volume_convergence_probe が落ちました"; tail -20 "$OUT/rvc.txt"; exit 1; }
# **間引く口**（既定は間引きません）——**この照合は OCC 側が長く、
# 作りを確かめるだけでも 30 分かかります**。`ZENITH_FACE_SLACK_STRIDE=20`
# で 1/20 に間引くと数分で一周します（4-666）。**本番では立てないこと**
# ——**間引くと最悪値を取りこぼします。**
STRIDE="${ZENITH_FACE_SLACK_STRIDE:-1}"
if [ "$STRIDE" != "1" ]; then
  awk -v s="$STRIDE" '{n[$1]++; if ((n[$1] - 1) % s == 0) print}'     "$OUT/points.txt" > "$OUT/points_thin.txt"
  mv "$OUT/points_thin.txt" "$OUT/points.txt"
  echo "  **1/$STRIDE に間引きました（最悪値を取りこぼします）**"
fi
pts=$(wc -l < "$OUT/points.txt")
[ "$pts" -eq 0 ] && { echo "WRONG 1 点を 1 つも書きませんでした"; exit 1; }
echo "  $pts 点"

echo "== ③ ファイル自身のたるみ（37 枚すべて） =="
ZENITH_ROUGHNESS_ALL=1 "$EXE/roughness_convergence_probe" > "$OUT/rough.txt" 2>&1 \
  || { echo "WRONG 1 roughness_convergence_probe が落ちました"; tail -20 "$OUT/rough.txt"; exit 1; }
awk '/128 点/{h=1;next} h&&/^[0-9]/{print $1, $7} /p-curve/{h=0}' "$OUT/rough.txt" > "$OUT/edge.txt"
awk '/p-curve の粗さ/{p=1} p&&/^[0-9]/{print $1, $6}' "$OUT/rough.txt" > "$OUT/pcurve.txt"
echo "  稜 $(wc -l < "$OUT/edge.txt") 枚 / p-curve $(wc -l < "$OUT/pcurve.txt") 枚"

echo "== ② OCC に距離を聞きます（長いです） =="
# **面ごとに flush するので、切られても続きから回せます**（4-664）。
# **既にある面は飛ばします。**
touch "$OUT/dist.txt"
# **前の回が切られていたら、拾ってから始めます**（4-666）——
# **探りは面ごとに flush しますが、`dist.txt` へ移すのは終わったあと**
# でした。**30 分で切られるこの機械では、それだと 1 枚も残りません**
# （実測: `dist_new.txt` に 27 枚あって `dist.txt` は 0 枚）。
if [ -s "$OUT/dist_new.txt" ]; then
  recovered=$(grep -a '^[0-9]' "$OUT/dist_new.txt" | awk 'NF == 6' | wc -l)
  if [ "$recovered" -gt 0 ]; then
    echo "  **前の回から $recovered 枚ぶん拾いました。**"
    # **切られたファイルを grep がバイナリと判定します**（4-666）——
    # **行の代わりに "Binary file ... matches" を 1 行書き**、
    # **その 1 行が「測り済みの面」として数えられて、
    # 37 枚のはずが 36 枚になりました。** `-a` で文字として読み、
    # **形の揃った行だけ拾います**（最後の 1 行は書きかけのことがあります）。
    grep -a '^[0-9]' "$OUT/dist_new.txt" | awk 'NF == 6' >> "$OUT/dist.txt"
  fi
  rm -f "$OUT/dist_new.txt"
fi
# **同じ面が 2 度入らないように**（拾ったぶんと重なります）。
# **`sort -n -k1 -u` は使いません**（4-666）——**`-n` と `-k1` だと
# 先頭の数字だけで比べる**ので、**同じ面の違う測定を、黙って
# 片方捨てます**。**畳むのは中身まで同じ行だけ**にして、
# **食い違いは下で赤にします。**
sort -u -o "$OUT/dist.txt" "$OUT/dist.txt"
grep -a '^[0-9]' "$OUT/dist.txt" 2>/dev/null | awk '{print $1}' | sort -n -u > "$OUT/done.txt"
# **`NR==FNR` は、1 つ目のファイルが空だと壊れます**（4-666）——
# **2 つ目の 1 行目で `NR==FNR` が成り立ってしまい、未測定なのに
# 「すべて測り済み」と出ました。** **ファイル名で見ます。**
awk -v done="$OUT/done.txt" '
  FILENAME == done { d[$1]; next }
  !($1 in d)
' "$OUT/done.txt" "$OUT/points.txt" > "$OUT/todo.txt"
if [ -s "$OUT/todo.txt" ]; then
  echo "  残り $(awk '{print $1}' "$OUT/todo.txt" | sort -u | wc -l) 枚 / $(wc -l < "$OUT/todo.txt") 点"
  ZENITH_FACE_POINTS="$OUT/todo.txt" "$PY" tools/occ_face_distance.py > "$OUT/dist_new.txt" 2>&1
  grep -a '^[0-9]' "$OUT/dist_new.txt" | awk 'NF == 6' >> "$OUT/dist.txt"
else
  echo "  すべて測り済みです。"
fi
# **同じ面が 2 行以上あったら、黙って直さずに言います**（4-666）——
# **一度だけ、面ごとに 3 行ある状態（37 枚のはずが 68 行）を見ました。**
# **直したあと再現できていません**ので、**起きたら分かるようにだけ
# しておきます**。**中身が同じ重複は畳み、違うものは赤**にします。
before=$(grep -a '^[0-9]' "$OUT/dist.txt" | awk 'NF == 6' | wc -l)
sort -u -o "$OUT/dist.txt" "$OUT/dist.txt"
after=$(grep -a '^[0-9]' "$OUT/dist.txt" | awk 'NF == 6' | wc -l)
[ "$before" -ne "$after" ] && echo "  **同じ行が $((before - after)) 本ありました**（畳みました）"
conflict=$(awk 'NF == 6 {print $1}' "$OUT/dist.txt" | sort -n | uniq -d | tr '
' ' ')
if [ -n "$conflict" ]; then
  echo "WRONG 1 同じ面に違う測定が入っています: $conflict"
  echo "**1 miss(es)**"
  exit 1
fi
faces=$(awk '{print $1}' "$OUT/points.txt" | sort -u | wc -l)
got=$(grep -a '^[0-9]' "$OUT/dist.txt" | awk 'NF == 6' | wc -l)
echo "  $got / $faces 枚"
if [ "$got" -lt "$faces" ]; then
  echo "  **途中で切られました。** **同じ口をもう一度回せば、続きから測ります。**"
fi
# **0 枚でも表が出て rc=0 になっていました**（4-666。4-656 と同じ穴を、
# **その翌日に自分で開けました**）。**測れていないことは、合格ではありません。**
if [ "$got" -eq 0 ]; then
  echo "WRONG 1 面を 1 枚も測れていません（$OUT/dist_new.txt を見てください）"
  [ -f "$OUT/dist_new.txt" ] && tail -20 "$OUT/dist_new.txt"
  exit 1
fi

echo
echo "== 並べます =="
"$PY" - "$OUT" <<'PYEOF'
import io, os, sys
# **この機械の既定は cp932 です**——**em ダッシュで落ちて、
# 表を全部出したあとに偽の赤になりました**（4-666）。
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass
out = sys.argv[1]
def load(name):
    d = {}
    for line in io.open(os.path.join(out, name), encoding="utf-8"):
        a = line.split()
        if len(a) == 2:
            d[int(a[0])] = float(a[1])
    return d
edge, pc = load("edge.txt"), load("pcurve.txt")
# **この測り方の床**（4-664。実測: 最大距離の最小 5.35e-10、中央 1.73e-10）
FLOOR = 1.1e-9
rows = []
for line in io.open(os.path.join(out, "dist.txt"), encoding="utf-8"):
    a = line.split()
    if len(a) >= 6:
        rows.append((int(a[0]), int(a[1]), float(a[4])))
rows.sort(key=lambda r: -r[2])
print(f"{'面':<5}{'点数':>7}{'OCC まで':>13}{'たるみ':>13}  見かた")
print("-" * 58)
inside = over = below = norec = 0
bad = []
for index, count, distance in rows:
    cand = [x for x in (edge.get(index), pc.get(index)) if x is not None]
    if not cand:
        norec += 1
        verdict, slack = "**たるみの記録なし**", float("nan")
    else:
        slack = max(cand)
        if distance <= FLOOR:
            below += 1
            verdict = "床以下（比較が成立しません）"
        elif distance <= slack * (1 + 1e-9):
            inside += 1
            verdict = "内側"
        else:
            over += 1
            bad.append((index, distance, slack))
            verdict = "**たるみを超えています**"
    print(f"{index:<5}{count:>7}{distance:>13.4e}{slack:>13.4e}  {verdict}")
print("-" * 58)
print(f"面 {len(rows)} 枚: 測れた {inside + over} 枚（内側 {inside} / 超過 {over}）、"
      f"床以下 {below} 枚、記録なし {norec} 枚")
if inside + over == 0:
    # **比べられた面が 1 枚も無いなら、何も言えません**（4-666）。
    print("WRONG 1 比べられた面が 1 枚もありません")
    print("**1 miss(es)**")
    raise SystemExit(1)
if over:
    for index, distance, slack in bad:
        print(f"WRONG 1 面{index}: {distance:.4e} > たるみ {slack:.4e}")
    print(f"**{over} miss(es)**")
    raise SystemExit(1)
print()
print("**測れた面はすべて、自分のたるみの内側です**（4-664）。")
print("**これは「こちらが正しい」という意味ではありません**——"
      "たるみの内側なら、どこに面を置いても辻褄は合います。")
PYEOF

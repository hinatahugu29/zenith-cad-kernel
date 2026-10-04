#!/usr/bin/env bash
# **門が「赤になれるか」を数える**（4-642。棚卸しは 4-641）。
#
# **門が赤になる道は 2 つだけ**です（`run_gates.sh`）——
#   ① 終了コード
#   ② 出力に出る 4 語（WRONG [1-9] / PANIC [1-9] / N miss(es) / N over the allowance）
#
# **だから探りごとに `exit(1)`／`exit(2)` と、その 4 語を数えれば、
# 「答えが間違ったときに赤になれるか」が決まります。**
# **`assert!` と `unwrap()` も数えます**——**panic も終了コードを立てる**ので、
# **「組めない／読めない」ときだけは落ちる**ことが分かります。
#
# **4-641 は散文で 15 本と数えました。** **散文は腐ります**ので、
# **数え直せる形**にしておきます（4-577 で `h8_ports.sh` を置いたのと同じ理屈）。
#
# 使い方: bash tools/audit_gate_redness.sh [--gate]
#
# **`--gate` を付けると、門になります**（4-661）——**床の無い門が
# 新しく増えたら赤**。**断りを出している探りだけは許します**が、
# **その断りは下の `ALLOWED` に名指しで書かないと通りません**。
# **「数えるだけ」の監査は、次に誰かが床無しの探りを足したとき、
# 何も言いません**——**それは今夜 13 本ためた作り方そのものです。**
set -uo pipefail
cd "$(dirname "$0")/.."

GATE_MODE=0
for arg in "$@"; do
  case "$arg" in
    --gate) GATE_MODE=1 ;;
    *) echo "知らない引数です: $arg（--gate だけ受けます）"; exit 2 ;;
  esac
done

# **床が無くてよい門**——**理由を自分の出力に書いているものだけ**。
# **ここに足すのは「床を置けない」と確かめたときだけ**にしてください。
ALLOWED="tess_density_probe"

# **門の一覧は `run_gates.sh` から取ります**——**こちらで書き写すと、
# 片方だけ古くなります。**
gates=$(sed -n '/^GATES="/,/"$/p' tools/run_gates.sh \
  | sed 's/^GATES="//; s/"$//' \
  | tr ' ' '\n' | sed '/^$/d')
# **門の輪の外**にあるが、同じ物差しで見ておきたいもの。
# **門の本数には足しません**（足すと「門 N 本」が嘘になります。4-658）
extras="contact_placement_probe"
gates="$gates
$extras"

printf "%-32s %5s %5s %7s %7s  %s\n" "門" "exit" "赤語" "assert" "unwrap" "答えが違うとき"
printf -- "----------------------------------------------------------------------------------\n"

blind=0
blind_list=""
total=0
extra_total=0
extra_blind=0
for g in $gates; do
  f="crates/zenith_algo/examples/$g.rs"
  [ -f "$f" ] || continue
  is_extra=0
  for x in $extras; do [ "$x" = "$g" ] && is_extra=1; done
  if [ "$is_extra" = "1" ]; then
    extra_total=$((extra_total + 1))
  else
    total=$((total + 1))
  fi
  # **コメント行は数えません**（4-661）——**`//exit(1);` を「床がある」と
  # 読むと、床を外したことに気づけません**（実演で引っかかりました）。
  body=$(grep -vE '^[[:space:]]*(//|/\*|\*)' "$f")
  e=$(printf '%s
' "$body" | grep -cE 'exit\([12]\)')
  w=$(printf '%s
' "$body" | grep -cE '"WRONG|PANIC|miss\(es\)|over the allowance')
  a=$(printf '%s
' "$body" | grep -cE 'assert!|assert_eq!|panic!')
  u=$(printf '%s
' "$body" | grep -cE '\.unwrap\(\)|\.expect\(')
  if [ "$e" = "0" ] && [ "$w" = "0" ]; then
    if [ "$is_extra" = "1" ]; then
      extra_blind=$((extra_blind + 1))
    else
      blind=$((blind + 1))
      blind_list="$blind_list $g"
    fi
    if [ "$a" = "0" ] && [ "$u" = "0" ]; then
      verdict="**落ちる道が無い**"
    else
      verdict="**緑のまま**（panic でだけ落ちる）"
    fi
    printf "%-32s %5s %5s %7s %7s  %s\n" "$g" "$e" "$w" "$a" "$u" "$verdict"
  fi
done

printf -- "----------------------------------------------------------------------------------\n"
echo "門 $total 本のうち、**答えが間違っても赤にならないもの $blind 本**。"
echo "（**門の外**で同じ物差しを当てたもの $extra_total 本のうち $extra_blind 本。"
echo "  **上の $total 本には入れていません**）"
echo
echo '**この数は「悪い」とは限りません**——tess_density_probe は'
echo '「報告専用」と自分の出力に書いてあります。**断りの無いものだけが宿題**です。'
echo '**床の足し方と、赤の実演のしかたは 4-639／4-640 に**。'
echo
if [ "$GATE_MODE" = "1" ]; then
  unexplained=""
  for g in $blind_list; do
    ok=0
    for a in $ALLOWED; do [ "$a" = "$g" ] && ok=1; done
    [ "$ok" = "0" ] && unexplained="$unexplained $g"
  done
  if [ -n "$unexplained" ]; then
    echo
    echo "WRONG 1 床の無い門があります（断りも出ていません）:$unexplained"
    echo "**$(echo $unexplained | wc -w) miss(es)**"
    exit 1
  fi
  echo
  echo "**断りの無い「床の無い門」は 0 本です。**"
  exit 0
fi

echo "**門の輪の外**で呼ばれるものは、ここに出ません——export_validation_suite は"
echo "run_gates.sh が門の前に呼びます。**4-659 で || true を外しました**——"
echo "落ちたときと**0 件しか書けなかったとき**は、そこで赤で止まります。"

#!/usr/bin/env bash
# テストを、被覆を1つも減らさずに速く回す。
#
# `cargo test` はテストバイナリを**1本ずつ順に**走らせます（並列になるのは
# 1本の中のテストだけ）。このリポジトリは重いバイナリが数本あるので、
# 全体の壁時計はほぼ「各バイナリの合計」になります。
#
# 実測（2026年8月24日、95バイナリ / 518テスト）:
#
#   tilted_cutter_test              145.7s
#   boolean_crossed_cylinder_test   110.4s
#   boolean_torus_box_test           82.4s
#   boolean_torus_test               55.0s
#   curved_face_chain_split_test     54.4s
#   torus_half_slab_test             46.2s
#   boolean_verification_test        44.1s
#   countersink_range_test           39.1s
#   plane_cone_section_test          30.1s
#   modeling_test                    25.1s
#   （残り85本は合計しても短い）
#
# 合計はおよそ 13 分。**バイナリを並列に走らせれば、いちばん遅い1本
# （146秒）で頭打ちになります。**
#
# 部分集合を選ぶ速い段を作るより、こちらのほうが筋が良いと判断しました。
# 選ぶ側は「何を落としたか」を人が覚えていなければならず、このリポジトリで
# 見つかった欠陥は**まさに測っていなかったところ**から出ているからです。
#
#   bash tools/fast_test.sh          # 既定は CPU 数ぶん並列
#   bash tools/fast_test.sh 4        # 並列数を指定する
#
# **1本でも失敗したら非ゼロで終わります。** 失敗したバイナリの出力は
# そのまま流します。
#
# 注意: テストが同じファイルに書き込むと、並列にしたときだけ壊れます。
# いまのところ該当はありませんが、**新しいテストが `target/` の下に書く
# ようになったら、ここが最初に壊れます。** そのときは書き先をテストごとに
# 分けてください（並列をやめるのではなく）。

set -u

jobs="${1:-0}"
if [ "$jobs" = "0" ]; then
    jobs="$(nproc 2>/dev/null || echo 4)"
fi

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root" || exit 1

echo "== ビルド（テストは走らせない）"
if ! cargo test --release --workspace --exclude zenith_py --no-run 2>&1 | tail -3; then
    echo "ビルドが通りませんでした"
    exit 1
fi

# `--no-run` の出力からバイナリの場所を取ります。名前で拾うと、古い
# ハッシュ付きの実行ファイルが残っていたときにそれも掴んでしまいます。
mapfile -t binaries < <(
    cargo test --release --workspace --exclude zenith_py --no-run --message-format=json 2>/dev/null |
        grep -o '"executable":"[^"]*"' |
        sed 's/"executable":"//; s/"$//' |
        grep -v '^null$' |
        sed 's|\\\\|/|g'
)

if [ "${#binaries[@]}" -eq 0 ]; then
    echo "テストバイナリが1つも見つかりませんでした"
    exit 1
fi

echo "== ${#binaries[@]} バイナリを ${jobs} 並列で走らせます"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

printf '%s\n' "${binaries[@]}" |
    xargs -P "$jobs" -I {} sh -c '
        name=$(basename "{}")
        if "{}" > "'"$out"'/$name.txt" 2>&1; then
            :
        else
            echo "FAILED $name" > "'"$out"'/$name.failed"
        fi
    '

passed=$(cat "$out"/*.txt 2>/dev/null | grep -h '^test result:' | sed 's/[^0-9 ]//g' | awk '{p+=$1} END{print p+0}')
failed=$(cat "$out"/*.txt 2>/dev/null | grep -h '^test result:' | sed 's/[^0-9 ]//g' | awk '{f+=$2} END{print f+0}')
broken=$(ls "$out"/*.failed 2>/dev/null | wc -l)

echo
echo "== ${#binaries[@]} バイナリ / ${passed} 合格 / ${failed} 不合格 / ${broken} バイナリが非ゼロ終了"

if [ "$broken" -gt 0 ] || [ "$failed" -gt 0 ]; then
    for marker in "$out"/*.failed; do
        [ -e "$marker" ] || continue
        name=$(basename "$marker" .failed)
        echo
        echo "---- $name"
        tail -40 "$out/$name.txt"
    done
    exit 1
fi

echo "すべて緑です"

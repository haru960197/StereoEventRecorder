#!/usr/bin/env bash
# ===========================================================================
# calibrate_hot_pixels.sh – ホットピクセルキャリブレーションスクリプト
#
# 概要:
#   metavision_active_pixel_detection を複数回実行し、
#   全測定に共通するホットピクセルを特定して JSON に保存する。
#
# 使用方法:
#   ./scripts/calibrate_hot_pixels.sh
#   ./scripts/calibrate_hot_pixels.sh -s <serial> -r master
#   ./scripts/calibrate_hot_pixels.sh -n 10
# ===========================================================================
set -euo pipefail

# デフォルト設定
NUM_MEASUREMENTS=5
SERIAL_ID=""
ROLE=""
CUSTOM_OUTPUT=""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

ホットピクセルキャリブレーションスクリプト。
metavision_active_pixel_detection を複数回実行し、
全測定に共通するホットピクセルを指定の JSON ファイルに保存します。

OPTIONS:
  -n NUM    測定回数 (default: ${NUM_MEASUREMENTS})
  -s SERIAL カメラのシリアルID
  -r ROLE   カメラの役割 (master または slave)
  -o PATH   出力JSONファイルのパス (デフォルト: config/hot_pixels_<role|serial>.json)
  -h        このヘルプを表示
EOF
    exit 0
}

while getopts "n:s:r:o:h" opt; do
    case "$opt" in
        n) NUM_MEASUREMENTS="$OPTARG" ;;
        s) SERIAL_ID="$OPTARG" ;;
        r) ROLE="$OPTARG" ;;
        o) CUSTOM_OUTPUT="$OPTARG" ;;
        h) usage ;;
        *) usage ;;
    esac
done

# 入力チェック
if ! command -v metavision_active_pixel_detection &>/dev/null; then
    echo "[ERROR] metavision_active_pixel_detection が見つかりません。"
    echo "        Metavision SDK がインストールされていることを確認してください。"
    exit 1
fi

if ! command -v python3 &>/dev/null; then
    echo "[ERROR] python3 が見つかりません。"
    exit 1
fi

# 出力先JSONパスの決定
if [ -n "$CUSTOM_OUTPUT" ]; then
    OUTPUT_JSON="$CUSTOM_OUTPUT"
elif [ -n "$ROLE" ]; then
    OUTPUT_JSON="${PROJECT_ROOT}/config/hot_pixels_${ROLE}.json"
elif [ -n "$SERIAL_ID" ]; then
    OUTPUT_JSON="${PROJECT_ROOT}/config/hot_pixels_${SERIAL_ID}.json"
else
    OUTPUT_JSON="${PROJECT_ROOT}/config/hot_pixels.json"
fi

# 一時ディレクトリの作成
TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hp_calibration_XXXXXX")
trap 'rm -rf "$TMP_DIR"' EXIT

echo "=============================================="
echo " ホットピクセルキャリブレーション (Stereo)"
echo "=============================================="
echo ""
echo "  測定回数     : ${NUM_MEASUREMENTS}"
if [ -n "$SERIAL_ID" ]; then
echo "  シリアルID   : ${SERIAL_ID}"
fi
if [ -n "$ROLE" ]; then
echo "  役割         : ${ROLE}"
fi
echo "  一時保存先   : ${TMP_DIR}/"
echo "  出力先       : ${OUTPUT_JSON}"
echo ""
echo "----------------------------------------------"
echo " ※ 遮光環境を確認してください"
echo " ※ カメラが接続されていることを確認してください"
echo "----------------------------------------------"
echo ""
read -rp "準備ができたら Enter を押して開始 → "
echo ""

# メインループ
for i in $(seq 1 "$NUM_MEASUREMENTS"); do
    echo "━━━ 測定 ${i}/${NUM_MEASUREMENTS} ━━━"

    DEST_NAME="measurement_$(printf '%02d' "$i").txt"
    DEST_PATH="${TMP_DIR}/${DEST_NAME}"

    # metavision_active_pixel_detection を起動
    echo "  🔬 metavision_active_pixel_detection を起動します..."
    echo "     → GUI 上でスペースキーを押して検出を完了してください"
    echo ""

    cmd=(metavision_active_pixel_detection -o "$DEST_PATH")
    if [ -n "$SERIAL_ID" ]; then
        cmd+=(-s "$SERIAL_ID")
    fi

    # 実行
    "${cmd[@]}" || {
        echo "[WARNING] 検出コマンドが異常終了しました (exit code: $?)"
    }

    # 結果ファイルの存在確認
    if [ -f "$DEST_PATH" ]; then
        echo "  📁 保存完了: ${DEST_NAME}"
    else
        echo "  ❌ 測定データが作成されませんでした。"
        echo "     この測定をスキップします。"
    fi

    echo ""

    # 最終回以外は操作待ち
    if [ "$i" -lt "$NUM_MEASUREMENTS" ]; then
        read -rp "  次の測定に進むには Enter を押してください → "
        echo ""
    fi
done

# 解析
echo ""
echo "━━━ 解析 ━━━"

COLLECTED_FILES=("$TMP_DIR"/measurement_*.txt)
NUM_COLLECTED=0
for f in "${COLLECTED_FILES[@]}"; do
    if [ -f "$f" ]; then
        NUM_COLLECTED=$((NUM_COLLECTED + 1))
    fi
done

if [ "$NUM_COLLECTED" -eq 0 ]; then
    echo "[ERROR] 測定データが1件もありません。キャリブレーション失敗。"
    exit 1
fi

echo "  ${NUM_COLLECTED} 件の測定データを解析します..."

python3 "${SCRIPT_DIR}/parse_hot_pixels.py" \
    "${TMP_DIR}" \
    -o "$OUTPUT_JSON"

echo ""
echo "=============================================="
echo " キャリブレーション完了"
echo "=============================================="
echo ""
echo "次のステップ:"
echo "  ホットピクセルマスク付きで録画:"
if [ -n "$ROLE" ]; then
echo "    ./build/stereo_event_recorder --${ROLE}"
else
echo "    ./build/stereo_event_recorder --master (または --slave)"
fi
echo ""

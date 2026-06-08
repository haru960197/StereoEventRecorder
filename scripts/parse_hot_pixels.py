#!/usr/bin/env python3
"""
parse_hot_pixels.py – 複数のactive_pixel_calib.txtを解析し、
全ファイルに共通するホットピクセルをconfig/hot_pixels.jsonに出力する。

Usage:
    python3 parse_hot_pixels.py <txt_dir_or_files...> -o <output.json>

    # ディレクトリ内の全txtファイルを処理
    python3 parse_hot_pixels.py /tmp/hp_calibration/ -o config/hot_pixels.json

    # ファイルを個別指定
    python3 parse_hot_pixels.py file1.txt file2.txt file3.txt -o config/hot_pixels.json
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


def parse_active_pixel_file(filepath: Path) -> set[tuple[int, int]]:
    """
    active_pixel_calib.txt をパースし、ピクセル座標の集合を返す。

    ファイル形式:
      - '%' で始まる行はコメント/メタデータ（スキップ）
      - 空行はスキップ
      - データ行は "x y" の2カラム（スペース区切り）
    """
    pixels = set()
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    x = int(parts[0])
                    y = int(parts[1])
                    pixels.add((x, y))
                except ValueError:
                    pass
    return pixels


def find_common_hot_pixels(pixel_sets: list[set[tuple[int, int]]]) -> set[tuple[int, int]]:
    """全ての集合に共通するピクセルの積集合を求める。"""
    if not pixel_sets:
        return set()
    common = pixel_sets[0]
    for ps in pixel_sets[1:]:
        common = common & ps
    return common


def save_hot_pixels_json(pixels: set[tuple[int, int]], output_path: Path, num_files: int):
    """ホットピクセルをJSON形式で保存する。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "hot_pixels": sorted(
            [{"x": x, "y": y} for x, y in pixels],
            key=lambda p: (p["y"], p["x"]),
        ),
        "metadata": {
            "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
            "num_measurements": num_files,
            "num_hot_pixels": len(pixels),
            "description": f"Hot pixels detected in all {num_files} calibration runs",
        },
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"Saved {len(pixels)} hot pixel(s) to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Parse active pixel detection results and find common hot pixels."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        help="Input txt files or a directory containing txt files.",
    )
    parser.add_argument(
        "-o",
        "--output",
        required=True,
        help="Output path for hot_pixels.json.",
    )
    args = parser.parse_args()

    # Collect input files
    txt_files: list[Path] = []
    for inp in args.inputs:
        p = Path(inp)
        if p.is_dir():
            found = sorted(p.glob("*.txt"))
            txt_files.extend(found)
        elif p.is_file():
            txt_files.append(p)
        else:
            print(f"[WARNING] Skipping non-existent path: {inp}", file=sys.stderr)

    if not txt_files:
        print("[ERROR] No input txt files found.", file=sys.stderr)
        sys.exit(1)

    print(f"Processing {len(txt_files)} file(s):")
    for f in txt_files:
        print(f"  - {f}")

    # Parse each file
    pixel_sets: list[set[tuple[int, int]]] = []
    for tf in txt_files:
        ps = parse_active_pixel_file(tf)
        print(f"  {tf.name}: {len(ps)} pixel(s)")
        pixel_sets.append(ps)

    # Find intersection
    common = find_common_hot_pixels(pixel_sets)
    print(f"\nCommon hot pixels (present in ALL {len(txt_files)} files): {len(common)}")

    # Save result
    output_path = Path(args.output)
    save_hot_pixels_json(common, output_path, len(txt_files))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import array
import json
import math
from pathlib import Path


def load_meta(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        meta = json.load(f)
    meta["_json_path"] = str(path)
    file_path = Path(meta["file"])
    if not file_path.is_absolute():
        file_path = path.parent / file_path.name
    meta["_bin_path"] = str(file_path)
    return meta


def collect_dump(root: Path):
    by_name = {}
    for json_path in sorted(root.glob("*.json")):
        meta = load_meta(json_path)
        name = str(meta.get("output_name", ""))
        frame = int(meta.get("frame_index", 0))
        stage = str(meta.get("stage", ""))
        by_name.setdefault(name, []).append((frame, stage, meta))

    for items in by_name.values():
        items.sort(key=lambda item: (item[0], item[1]))
    return by_name


def choose_meta(by_name, name: str, frame: int | None, preferred_stage: str | None):
    items = by_name.get(name, [])
    if not items:
        return None

    candidates = items
    if frame is not None:
        candidates = [item for item in candidates if item[0] == frame]
    if preferred_stage:
        staged = [item for item in candidates if item[1] == preferred_stage]
        if staged:
            candidates = staged
    if not candidates:
        return None
    return candidates[0][2]


def read_tensor(meta: dict) -> array.array:
    path = Path(meta["_bin_path"])
    arr = array.array("f")
    with path.open("rb") as f:
        arr.frombytes(f.read())
    value_count = int(meta.get("value_count", len(arr)))
    if len(arr) != value_count:
        raise RuntimeError(f"{path}: expected {value_count} float32 values, got {len(arr)}")
    return arr


def stats(arr):
    if len(arr) == 0:
        return {"min": math.nan, "max": math.nan, "mean": math.nan}
    total = 0.0
    mn = math.inf
    mx = -math.inf
    for value in arr:
        v = float(value)
        total += v
        if v < mn:
            mn = v
        if v > mx:
            mx = v
    return {
        "min": mn,
        "max": mx,
        "mean": total / len(arr),
    }


def compare_arrays(a, b):
    max_abs = 0.0
    sum_abs = 0.0
    dot = 0.0
    norm_a2 = 0.0
    norm_b2 = 0.0
    diff_total = 0.0
    diff_min = math.inf
    diff_max = -math.inf

    for av, bv in zip(a, b):
        da = float(av)
        db = float(bv)
        d = da - db
        ad = abs(d)
        if ad > max_abs:
            max_abs = ad
        sum_abs += ad
        dot += da * db
        norm_a2 += da * da
        norm_b2 += db * db
        diff_total += d
        if d < diff_min:
            diff_min = d
        if d > diff_max:
            diff_max = d

    count = len(a)
    denom = math.sqrt(norm_a2) * math.sqrt(norm_b2)
    if denom == 0.0:
        cos = 1.0 if max_abs == 0.0 else math.nan
    else:
        cos = dot / denom

    return {
        "max_abs_diff": max_abs if count else math.nan,
        "mean_abs_diff": (sum_abs / count) if count else math.nan,
        "cosine_similarity": cos,
        "diff": {
            "min": diff_min if count else math.nan,
            "max": diff_max if count else math.nan,
            "mean": (diff_total / count) if count else math.nan,
        },
    }


def compare_one(name: str, board_meta: dict, x86_meta: dict):
    a = read_tensor(board_meta)
    b = read_tensor(x86_meta)
    if len(a) != len(b):
        raise RuntimeError(
            f"{name}: value count mismatch board={len(a)} x86={len(b)}\n"
            f"  board_json={board_meta['_json_path']}\n"
            f"  x86_json={x86_meta['_json_path']}"
        )

    board_stats = stats(a)
    x86_stats = stats(b)
    cmp_stats = compare_arrays(a, b)
    return {
        "name": name,
        "count": int(len(a)),
        "max_abs_diff": cmp_stats["max_abs_diff"],
        "mean_abs_diff": cmp_stats["mean_abs_diff"],
        "cosine_similarity": cmp_stats["cosine_similarity"],
        "board": board_stats,
        "x86": x86_stats,
        "diff": cmp_stats["diff"],
        "board_json": board_meta["_json_path"],
        "x86_json": x86_meta["_json_path"],
    }


def main():
    parser = argparse.ArgumentParser(
        description="Compare TI board ti_output_dump tensors with TI x86 ti_output_dump tensors."
    )
    parser.add_argument("--board", required=True, type=Path, help="Board ti_output_dump directory")
    parser.add_argument("--x86", required=True, type=Path, help="TI x86 ti_output_dump directory")
    parser.add_argument("--frame", type=int, default=0, help="Frame index to compare, default: 0")
    parser.add_argument(
        "--names",
        nargs="*",
        default=None,
        help="Output names to compare. Default: intersection of board/x86 dump names.",
    )
    parser.add_argument(
        "--x86-stage",
        default=None,
        choices=[None, "raw", "normalized"],
        help="Prefer TI x86 dump stage. Default: auto by output name.",
    )
    args = parser.parse_args()

    board = collect_dump(args.board)
    x86 = collect_dump(args.x86)

    if args.names:
        names = args.names
    else:
        names = sorted(set(board.keys()) & set(x86.keys()))

    if not names:
        raise SystemExit(
            "No matching output names found.\n"
            f"board names={sorted(board.keys())}\n"
            f"x86 names={sorted(x86.keys())}"
        )

    print(f"board_dump={args.board}")
    print(f"x86_dump={args.x86}")
    print(f"frame={args.frame}")
    print("")

    for name in names:
        preferred_stage = args.x86_stage
        if preferred_stage is None and name in {"det_cat", "proto", "angle"}:
            preferred_stage = "normalized"

        board_meta = choose_meta(board, name, args.frame, None)
        x86_meta = choose_meta(x86, name, args.frame, preferred_stage)
        if board_meta is None:
            print(f"[SKIP] {name}: missing board tensor")
            continue
        if x86_meta is None:
            print(f"[SKIP] {name}: missing x86 tensor")
            continue

        result = compare_one(name, board_meta, x86_meta)
        print(f"[{name}] count={result['count']}")
        print(f"  max_abs_diff      : {result['max_abs_diff']:.9g}")
        print(f"  mean_abs_diff     : {result['mean_abs_diff']:.9g}")
        print(f"  cosine_similarity : {result['cosine_similarity']:.9g}")
        print(
            "  board min/max/mean: "
            f"{result['board']['min']:.9g} / {result['board']['max']:.9g} / {result['board']['mean']:.9g}"
        )
        print(
            "  x86   min/max/mean: "
            f"{result['x86']['min']:.9g} / {result['x86']['max']:.9g} / {result['x86']['mean']:.9g}"
        )
        print(
            "  diff  min/max/mean: "
            f"{result['diff']['min']:.9g} / {result['diff']['max']:.9g} / {result['diff']['mean']:.9g}"
        )
        print(f"  board_json: {result['board_json']}")
        print(f"  x86_json  : {result['x86_json']}")
        print("")


if __name__ == "__main__":
    main()

"""Run the HSV gem detector on saved images and write annotated copies.

Use it to tune colors offline, without the camera or robot:

    .venv/bin/python tools/check_colors.py data/captures/raw/*.jpg
    .venv/bin/python tools/check_colors.py --colors crimson,violet,gold img.jpg

Annotated images go to data/color_check/. Detection runs on the whole image
(no arena warp), so the red arena wall may also show up as crimson here.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "host"))

import vision  # noqa: E402

DRAW_BGR = {
    "violet": (211, 0, 148), "cyan": (255, 255, 0), "crimson": (60, 20, 220),
    "gold": (0, 215, 255), "blue": (255, 0, 0), "lime": (0, 255, 0),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--colors", default=",".join(vision.PALETTE))
    parser.add_argument("--max-area", type=float, default=4500.0,
                        help="largest blob in px^2; raise it for full-resolution photos")
    parser.add_argument("--out", type=Path, default=ROOT / "data" / "color_check")
    args = parser.parse_args()
    vision.set_palette(tuple(args.colors.split(",")))
    args.out.mkdir(parents=True, exist_ok=True)

    for path in args.images:
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path}: cannot read")
            continue
        gems = vision.detect_gems(image, max_area=args.max_area)
        counts = Counter(gem.color for gem in gems)
        print(f"{path.name}: " + ", ".join(f"{color}={counts[color]}" for color in vision.PALETTE))
        for gem in gems:
            half = max(8, int(gem.area ** 0.5) // 2)
            color = DRAW_BGR[gem.color]
            cv2.rectangle(image, (gem.x - half, gem.y - half), (gem.x + half, gem.y + half), color, 2)
            cv2.putText(image, gem.color, (gem.x - half, gem.y - half - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        cv2.imwrite(str(args.out / path.name), image)
    print(f"Annotated images: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

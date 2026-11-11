#!/usr/bin/env python3
"""Render controls_sheet.html (dark mode, 1920x1080) to web-ui/controls_key.png.

The PNG is the in-headset controls key shown while left X is held.
Needs Firefox and network access (the sheet loads its fonts from Google Fonts).

Usage:  python3 src/alfie_vr/graphics/render_controls_key.py [--out PATH]
"""

import argparse
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SHEET = HERE / 'controls_sheet.html'
DEFAULT_OUT = HERE.parent / 'alfie_vr' / 'web-ui' / 'controls_key.png'
WIDTH, HEIGHT = 1920, 1080

# Force dark theme, drop page chrome, size the SVG to exactly fill the window.
RENDER_CSS = f"""<style>
body{{padding:0!important;margin:0;background:var(--sheet)}}
.scroller{{max-width:none;overflow:visible}}
svg.sheet{{width:{WIDTH}px;height:{HEIGHT}px;min-width:0;border-radius:0}}
</style>"""


def build_page(src: str) -> str:
    marker = '<div class="scroller">'
    head, body = src.split(marker, 1)
    return ('<!doctype html><html data-theme="dark"><head><meta charset="utf-8">'
            + head + RENDER_CSS + '</head><body>' + marker + body + '</body></html>')


def png_size(path: Path):
    return struct.unpack('>II', path.read_bytes()[16:24])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    firefox = shutil.which('firefox')
    if not firefox:
        sys.exit('firefox not found')

    # Snap Firefox has a private /tmp, so work under the home directory.
    work_root = Path.home() / 'snap' / 'firefox' / 'common'
    if not work_root.is_dir():
        work_root = Path.home()
    with tempfile.TemporaryDirectory(dir=work_root, prefix='controls_key_') as tmp:
        tmp = Path(tmp)
        page = tmp / 'sheet_dark.html'
        shot = tmp / 'controls_key.png'
        page.write_text(build_page(SHEET.read_text()))
        subprocess.run([firefox, '--headless', f'--window-size={WIDTH},{HEIGHT}',
                        '--screenshot', str(shot), page.as_uri()],
                       check=True, timeout=120,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not shot.exists():
            sys.exit('firefox did not write a screenshot')
        if png_size(shot) != (WIDTH, HEIGHT):
            sys.exit(f'unexpected size {png_size(shot)}')
        args.out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(shot, args.out)
    print(f'wrote {args.out} ({WIDTH}x{HEIGHT})')


if __name__ == '__main__':
    main()

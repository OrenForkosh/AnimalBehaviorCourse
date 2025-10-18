#!/usr/bin/env python3
"""
Bundle the app into a single self-contained HTML file by inlining:
- Stylesheets referenced via <link rel="stylesheet" href="...">
- Scripts referenced via <script src="..."></script>
- Icons referenced via <link rel="icon" href="...">
- Images referenced via <img src="...">

Usage:
  python3 tools/bundle_single_html.py --input index.html --output single.html
"""
from __future__ import annotations

import argparse
import base64
import mimetypes
import os
import re
from pathlib import Path


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except Exception as e:
        raise SystemExit(f"Failed to read {path}: {e}")


def read_bytes(path: Path) -> bytes:
    try:
        return path.read_bytes()
    except Exception as e:
        raise SystemExit(f"Failed to read {path}: {e}")


def to_data_uri(path: Path) -> str:
    data = read_bytes(path)
    mime, _ = mimetypes.guess_type(path.name)
    if not mime:
        # default fallback for unknown
        mime = "application/octet-stream"
    b64 = base64.b64encode(data).decode("ascii")
    return f"data:{mime};base64,{b64}"


def inline_styles(html: str, base_dir: Path) -> str:
    # Match <link ... rel="stylesheet" ... href="..." ...>
    link_re = re.compile(r"<link[^>]*rel=[\"']stylesheet[\"'][^>]*href=[\"']([^\"']+)[\"'][^>]*/?>", re.IGNORECASE)

    def repl(m: re.Match) -> str:
        href = m.group(1)
        css_path = (base_dir / href).resolve()
        if not css_path.exists():
            # Try without leading ./
            css_path = (base_dir / href.lstrip("./")).resolve()
        css = read_text(css_path)
        return f"<style>\n{css}\n</style>"

    return link_re.sub(repl, html)


def inline_scripts(html: str, base_dir: Path) -> str:
    # Match <script ... src="..."></script>
    script_re = re.compile(r"<script([^>]*)\ssrc=[\"']([^\"']+)[\"']([^>]*)>\s*</script>", re.IGNORECASE)

    def repl(m: re.Match) -> str:
        pre_attrs, src, post_attrs = m.group(1), m.group(2), m.group(3)
        js_path = (base_dir / src).resolve()
        if not js_path.exists():
            js_path = (base_dir / src.lstrip("./")).resolve()
        js = read_text(js_path)
        # Remove type/module attrs from original; keep a plain script
        return f"<script>\n{js}\n</script>"

    return script_re.sub(repl, html)


def inline_icons(html: str, base_dir: Path) -> str:
    # Match <link ... rel="icon" ... href="..." ...>
    icon_re = re.compile(r"(<link[^>]*rel=[\"']icon[\"'][^>]*href=)[\"']([^\"']+)[\"']([^>]*>)", re.IGNORECASE)

    def repl(m: re.Match) -> str:
        prefix, href, suffix = m.group(1), m.group(2), m.group(3)
        icon_path = (base_dir / href).resolve()
        if not icon_path.exists():
            icon_path = (base_dir / href.lstrip("./")).resolve()
        data_uri = to_data_uri(icon_path)
        return f"{prefix}\"{data_uri}\"{suffix}"

    return icon_re.sub(repl, html)


def inline_images(html: str, base_dir: Path) -> str:
    # Match <img ... src="..." ...>
    img_re = re.compile(r"(<img[^>]*\ssrc=)[\"']([^\"']+)[\"']([^>]*>)", re.IGNORECASE)

    def repl(m: re.Match) -> str:
        prefix, src, suffix = m.group(1), m.group(2), m.group(3)
        # Skip data URIs already inlined
        if src.startswith("data:"):
            return m.group(0)
        img_path = (base_dir / src).resolve()
        if not img_path.exists():
            img_path = (base_dir / src.lstrip("./")).resolve()
        data_uri = to_data_uri(img_path)
        return f"{prefix}\"{data_uri}\"{suffix}"

    return img_re.sub(repl, html)


def bundle(input_html: Path, output_html: Path) -> None:
    base_dir = input_html.parent
    html = read_text(input_html)

    # Order matters: inline CSS/JS before images so replaced tags don't get reprocessed oddly.
    html = inline_styles(html, base_dir)
    html = inline_scripts(html, base_dir)
    html = inline_icons(html, base_dir)
    html = inline_images(html, base_dir)

    output_html.write_text(html, encoding="utf-8")
    print(f"Bundled to {output_html}")


def main():
    ap = argparse.ArgumentParser(description="Bundle the app into a single HTML file")
    ap.add_argument("--input", default="index.html", help="Path to root index.html")
    ap.add_argument("--output", default="single.html", help="Output bundled HTML path")
    args = ap.parse_args()

    input_path = Path(args.input).resolve()
    output_path = Path(args.output).resolve()

    if not input_path.exists():
        raise SystemExit(f"Input HTML not found: {input_path}")

    bundle(input_path, output_path)


if __name__ == "__main__":
    main()


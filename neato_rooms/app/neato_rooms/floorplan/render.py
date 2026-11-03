"""Listing-style floorplan page, rendered to SVG or PDF from the same layout.

The PDF writer is self-contained (standard Helvetica fonts, no embedding), so the add-on
needs no extra dependencies.
"""

from __future__ import annotations

import math
import zlib
from dataclasses import dataclass, field
from xml.sax.saxutils import escape

from .extract import FloorPlan

PAPER_PT = {"letter": (612.0, 792.0), "a4": (595.28, 841.89)}
PT_PER_M = 72 / 0.0254
SCALES = [20, 25, 50, 75, 100, 125, 150, 200, 250, 300, 400, 500, 750, 1000]

WALL = (0.16, 0.16, 0.16)
FIXTURE = (0.45, 0.45, 0.45)
DOOR = (0.25, 0.25, 0.25)
TEXT = (0.10, 0.10, 0.10)
MUTED = (0.42, 0.42, 0.42)


# -- Units ----------------------------------------------------------------------------------


def fmt_len(m: float, units: str) -> str:
    if units == "metric":
        return f"{m:.2f} m"
    inches = int(round(m / 0.0254))
    ft, inch = divmod(inches, 12)
    return f"{ft}'{inch}\""


def fmt_area(m2: float, units: str) -> str:
    if units == "metric":
        return f"{m2:,.1f} m²"
    return f"{round(m2 * 10.7639):,} sq ft"


# -- Page model -------------------------------------------------------------------------------


@dataclass
class Page:
    width: float
    height: float
    items: list[tuple] = field(default_factory=list)  # page coords: points, origin top-left, y down

    def poly(self, pts, fill=None, stroke=None, width=0.0, close=True):
        self.items.append(("poly", [tuple(p) for p in pts], fill, stroke, width, close))

    def line(self, a, b, stroke, width, dash=None):
        self.items.append(("line", tuple(a), tuple(b), stroke, width, dash))

    def text(self, x, y, s, size, bold=False, anchor="middle", color=TEXT):
        self.items.append(("text", x, y, s, size, bold, anchor, color))


def _choose_scale(span_w: float, span_h: float, avail_w: float, avail_h: float) -> tuple[float, int | None]:
    for denom in SCALES:
        k = PT_PER_M / denom
        if span_w * k <= avail_w and span_h * k <= avail_h:
            return k, denom
    return min(avail_w / max(span_w, 1e-6), avail_h / max(span_h, 1e-6)), None


def layout(plan: FloorPlan, units: str = "imperial", title: str = "", paper: str = "auto") -> Page:
    if paper == "auto":
        paper = "letter" if units == "imperial" else "a4"
    pw, ph = PAPER_PT.get(paper, PAPER_PT["letter"])
    bx0, by0, bx1, by1 = plan.bounds
    if bx1 <= bx0 or by1 <= by0:
        bx0, by0, bx1, by1 = -1, -1, 1, 1
    pad = 0.4
    bx0, by0, bx1, by1 = bx0 - pad, by0 - pad, bx1 + pad, by1 + pad
    span_w, span_h = bx1 - bx0, by1 - by0
    if span_w > span_h:
        pw, ph = ph, pw  # landscape
    margin, header, footer = 36.0, 56.0, 46.0
    avail_w, avail_h = pw - 2 * margin, ph - 2 * margin - header - footer
    k, denom = _choose_scale(span_w, span_h, avail_w, avail_h)
    ox = margin + (avail_w - span_w * k) / 2
    oy = margin + header + (avail_h - span_h * k) / 2

    def P(x, y):
        return (ox + (x - bx0) * k, oy + (by1 - y) * k)

    page = Page(pw, ph)

    # Walls (filled), drawn with a hairline in the same colour so touching bodies merge.
    for f in plan.faces:
        if not f.visible or f.kind not in ("wall", "exterior"):
            continue
        page.poly([P(*p) for p in f.body()], fill=WALL, stroke=WALL, width=0.4)
    for f in plan.faces:
        if f.visible and f.kind == "fixture":
            page.line(P(*f.a), P(*f.b), FIXTURE, 0.9)

    # Doors: leaf + swing arc.
    for d in plan.doors:
        dx, dy = math.cos(d.angle), math.sin(d.angle)
        nx, ny = -dy * d.side, dx * d.side
        hx, hy = d.x - dx * d.width / 2, d.y - dy * d.width / 2
        page.line(P(hx, hy), P(hx + nx * d.width, hy + ny * d.width), DOOR, 0.9)
        a0 = math.atan2(ny, nx)
        a1 = math.atan2(dy, dx)
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        arc = [P(hx + d.width * math.cos(a0 + da * t / 24), hy + d.width * math.sin(a0 + da * t / 24)) for t in range(25)]
        page.poly(arc, stroke=DOOR, width=0.5, close=False)

    # Room labels.
    name_pt, dim_pt = 8.5, 7.5
    for r in plan.rooms:
        x, y = P(r.x, r.y)
        page.text(x, y - 2, r.name.upper(), name_pt, bold=True)
        if r.width_m and r.depth_m:
            page.text(x, y + dim_pt + 1, f"{fmt_len(r.width_m, units)} x {fmt_len(r.depth_m, units)}", dim_pt, color=MUTED)

    # Header.
    page.text(margin, margin + 16, title or "Floor Plan", 16, bold=True, anchor="start")
    sub = f"Approx. interior area {fmt_area(plan.interior_area_m2, units)}"
    page.text(margin, margin + 32, sub, 9, anchor="start", color=MUTED)

    # Footer: scale bar + scale + disclaimer.
    fy = ph - margin - 18
    if units == "metric":
        bar_m, label = 2.0, "2 m"
        if 2.0 * k > 160:
            bar_m, label = 1.0, "1 m"
    else:
        bar_m, label = 10 * 0.3048, "10 ft"
        if bar_m * k > 160:
            bar_m, label = 5 * 0.3048, "5 ft"
    bar = bar_m * k
    x0 = margin
    for i in range(4):
        seg = (x0 + bar * i / 4, fy, x0 + bar * (i + 1) / 4)
        page.poly([(seg[0], fy - 3), (seg[2], fy - 3), (seg[2], fy + 1), (seg[0], fy + 1)],
                  fill=TEXT if i % 2 == 0 else (1, 1, 1), stroke=TEXT, width=0.5)
    page.text(x0 + bar + 6, fy + 1, label, 7.5, anchor="start", color=MUTED)
    scale_txt = f"Scale 1:{denom} on {paper.upper() if paper == 'a4' else paper.title()}" if denom else "Not to scale"
    page.text(pw - margin, fy + 1, scale_txt, 7.5, anchor="end", color=MUTED)
    page.text(pw - margin, fy + 13,
              "Generated from robot LIDAR scans. All dimensions are approximate and not for construction.",
              6.5, anchor="end", color=MUTED)
    return page


# -- SVG ----------------------------------------------------------------------------------------


def _rgb(c) -> str:
    return "#%02x%02x%02x" % tuple(int(round(v * 255)) for v in c)


def to_svg(page: Page) -> str:
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{page.width:.0f}pt" height="{page.height:.0f}pt" '
        f'viewBox="0 0 {page.width:.2f} {page.height:.2f}">',
        f'<rect width="{page.width:.2f}" height="{page.height:.2f}" fill="#ffffff"/>',
    ]
    for it in page.items:
        kind = it[0]
        if kind == "poly":
            _, pts, fill, stroke, width, close = it
            d = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
            tag = "polygon" if close else "polyline"
            out.append(
                f'<{tag} points="{d}" fill="{_rgb(fill) if fill else "none"}" '
                f'stroke="{_rgb(stroke) if stroke else "none"}" stroke-width="{width:.2f}" stroke-linejoin="miter"/>'
            )
        elif kind == "line":
            _, a, b, stroke, width, dash = it
            da = f' stroke-dasharray="{dash}"' if dash else ""
            out.append(f'<line x1="{a[0]:.2f}" y1="{a[1]:.2f}" x2="{b[0]:.2f}" y2="{b[1]:.2f}" '
                       f'stroke="{_rgb(stroke)}" stroke-width="{width:.2f}"{da}/>')
        elif kind == "text":
            _, x, y, s, size, bold, anchor, color = it
            weight = 'font-weight="bold" ' if bold else ""
            out.append(
                f'<text x="{x:.2f}" y="{y:.2f}" font-family="Helvetica, Arial, sans-serif" font-size="{size}" '
                f'{weight}text-anchor="{anchor}" fill="{_rgb(color)}">{escape(s)}</text>'
            )
    out.append("</svg>")
    return "\n".join(out)


# -- PDF ------------------------------------------------------------------------------------------

# Helvetica advance widths (1/1000 em) for ASCII 32..126, from the standard AFM.
_HELV = [
    278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278,
    556, 556, 556, 556, 556, 556, 556, 556, 556, 556, 278, 278, 584, 584, 584, 556,
    1015, 667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833, 722, 778,
    667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 278, 278, 278, 469, 556,
    333, 556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556,
    556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, 334, 260, 334, 584,
]


def _text_width(s: str, size: float, bold: bool) -> float:
    w = sum(_HELV[ord(c) - 32] if 32 <= ord(c) <= 126 else 556 for c in s)
    return w * size / 1000 * (1.06 if bold else 1.0)


def _pdf_str(s: str) -> bytes:
    raw = s.encode("cp1252", errors="replace")
    return b"(" + raw.replace(b"\\", b"\\\\").replace(b"(", b"\\(").replace(b")", b"\\)") + b")"


def to_pdf(page: Page, title: str = "Floor Plan") -> bytes:
    H = page.height
    ops: list[bytes] = []

    def c(col, stroke):
        return f"{col[0]:.3f} {col[1]:.3f} {col[2]:.3f} {'RG' if stroke else 'rg'}".encode()

    for it in page.items:
        kind = it[0]
        if kind == "poly":
            _, pts, fill, stroke, width, close = it
            path = [f"{pts[0][0]:.2f} {H - pts[0][1]:.2f} m".encode()]
            path += [f"{x:.2f} {H - y:.2f} l".encode() for x, y in pts[1:]]
            if close:
                path.append(b"h")
            ops.append(b"q")
            if fill:
                ops.append(c(fill, False))
            if stroke:
                ops += [c(stroke, True), f"{width:.2f} w".encode(), b"0 j"]
            ops += path
            ops.append(b"B" if (fill and stroke) else b"f" if fill else b"S")
            ops.append(b"Q")
        elif kind == "line":
            _, a, b, stroke, width, dash = it
            ops += [b"q", c(stroke, True), f"{width:.2f} w".encode()]
            if dash:
                ops.append(f"[{dash}] 0 d".encode())
            ops += [f"{a[0]:.2f} {H - a[1]:.2f} m {b[0]:.2f} {H - b[1]:.2f} l S".encode(), b"Q"]
        elif kind == "text":
            _, x, y, s, size, bold, anchor, color = it
            w = _text_width(s, size, bold)
            if anchor == "middle":
                x -= w / 2
            elif anchor == "end":
                x -= w
            font = b"/F2" if bold else b"/F1"
            ops += [b"BT", c(color, False), font + f" {size} Tf".encode(),
                    f"1 0 0 1 {x:.2f} {H - y:.2f} Tm".encode(), _pdf_str(s) + b" Tj", b"ET"]
    content = zlib.compress(b"\n".join(ops))

    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page.width:.2f} {page.height:.2f}] "
        f"/Resources << /Font << /F1 5 0 R /F2 6 0 R >> >> /Contents 4 0 R >>".encode(),
        f"<< /Length {len(content)} /Filter /FlateDecode >>\nstream\n".encode() + content + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>",
        b"<< /Title " + _pdf_str(title) + b" /Producer (Neato Rooms) >>",
    ]
    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = []
    for i, body in enumerate(objs, start=1):
        offsets.append(len(out))
        out += f"{i} 0 obj\n".encode() + body + b"\nendobj\n"
    xref = len(out)
    out += f"xref\n0 {len(objs) + 1}\n0000000000 65535 f \n".encode()
    for off in offsets:
        out += f"{off:010d} 00000 n \n".encode()
    out += f"trailer\n<< /Size {len(objs) + 1} /Root 1 0 R /Info {len(objs)} 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode()
    return bytes(out)

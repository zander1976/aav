#!/usr/bin/env python3
"""
Make the Gazebo road world's stop sign: an octagon mesh, and the picture painted on it.

Writes, in src/backends/aav_gazebo/models/stop_sign/:
  meshes/stop_sign.obj                the plate: a flat octagon, its face looking along +x
  materials/textures/stop_sign.png    the face: red, a white rim, STOP in white

The picture is our own drawing, like the bench camera's sign (aav_bench pictures.py),
so it has no licence to track. The stock YOLO model finds drawn signs like it: the bench's
at 0.90. The letters are DejaVu Sans Bold (matplotlib's copy; its licence lets anyone
use it), squeezed to the narrow shape of a real sign's letters.

The texture's corners, outside the octagon, are the grey of the plate's back: the back
face takes its colour from there, so one picture paints both sides.

usage: pixi run python3 tools/make_stop_sign.py   (needs Pillow and matplotlib, not ROS)
"""

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src' / 'backends' / 'aav_gazebo' / 'models' / 'stop_sign'

ACROSS = 0.75         # m between opposite flats: the common Ontario size (Ra-1, 750 mm)
THICKNESS = 0.004     # m: the plate, front face to back
PIXELS = 512          # the texture, square
RED = (193, 39, 45)
WHITE = (250, 250, 250)
BACK = (150, 150, 150)  # galvanised steel


def corners(across=ACROSS):
    """Return the octagon's eight corners, as (y, z) in metres, anticlockwise from the front."""
    radius = across / 2.0 / math.cos(math.pi / 8)
    return [(radius * math.cos(math.pi / 8 + k * math.pi / 4),
             radius * math.sin(math.pi / 8 + k * math.pi / 4)) for k in range(8)]


def uv(y, z, across=ACROSS):
    """Return where (y, z) on the plate lands on the texture: u right, v up, 0 to 1."""
    return 0.5 + y / across, 0.5 + z / across


def number(value):
    """Format a coordinate for the OBJ file, with no -0."""
    return f'{round(value, 5) + 0.0:.5f}'


def obj(across=ACROSS, thickness=THICKNESS):
    """Return the plate as Wavefront OBJ text: a front and a back, each a fan of 8."""
    ring = corners(across)
    lines = ["# The stop sign's plate. Made by tools/make_stop_sign.py: don't edit.",
             f'# A flat octagon {across} m across the flats, its front looking along +x.']
    for x in (thickness / 2.0, -thickness / 2.0):
        lines.append(f'v {number(x)} 0 0')
        lines += [f'v {number(x)} {number(y)} {number(z)}' for y, z in ring]
    lines.append('vt 0.5 0.5')
    lines += ['vt ' + ' '.join(number(t) for t in uv(y, z, across)) for y, z in ring]
    lines.append('vt 0.02 0.02')                 # a corner: the back's grey
    lines += ['vn 1 0 0', 'vn -1 0 0']
    for k in range(8):                           # the front: anticlockwise seen from +x
        a, b = 2 + k, 2 + (k + 1) % 8
        lines.append(f'f 1/1/1 {a}/{a}/1 {b}/{b}/1')
    for k in range(8):                           # the back: the other way round
        a, b = 11 + k, 11 + (k + 1) % 8
        lines.append(f'f 10/10/2 {b}/10/2 {a}/10/2')
    return '\n'.join(lines) + '\n'


def picture(pixels=PIXELS):
    """Return the texture: the sign's face inside the octagon, grey outside it."""
    from matplotlib import font_manager
    from PIL import Image, ImageDraw, ImageFont

    big = pixels * 4           # draw large, then shrink: smooth edges

    def ring(scale):
        return [(big / 2.0 + big * y * scale / ACROSS, big / 2.0 - big * z * scale / ACROSS)
                for y, z in corners()]

    im = Image.new('RGB', (big, big), BACK)
    draw = ImageDraw.Draw(im)
    draw.polygon(ring(1.02), fill=RED)       # a little larger than the mesh: no grey edge
    draw.polygon(ring(0.96), fill=WHITE)     # the white rim
    draw.polygon(ring(0.90), fill=RED)
    font = ImageFont.truetype(font_manager.findfont(
        font_manager.FontProperties(family='DejaVu Sans', weight='bold')), big // 3)
    left, top, right, bottom = font.getbbox('STOP')
    word = Image.new('L', (right - left, bottom - top), 0)
    ImageDraw.Draw(word).text((-left, -top), 'STOP', font=font, fill=255)
    # A real sign's letters are narrow: the word fills 3/4 of the width, 1/3 of the height.
    word = word.resize((int(0.74 * big), int(0.30 * big)), Image.LANCZOS)
    im.paste(Image.new('RGB', word.size, WHITE),
             ((big - word.size[0]) // 2, (big - word.size[1]) // 2), word)
    return im.resize((pixels, pixels), Image.LANCZOS)


def main():
    (MODEL / 'meshes').mkdir(parents=True, exist_ok=True)
    (MODEL / 'materials' / 'textures').mkdir(parents=True, exist_ok=True)
    (MODEL / 'meshes' / 'stop_sign.obj').write_text(obj())
    picture().save(MODEL / 'materials' / 'textures' / 'stop_sign.png', optimize=True)
    print(f'wrote {MODEL.relative_to(ROOT)}: meshes/stop_sign.obj, '
          'materials/textures/stop_sign.png')
    return 0


if __name__ == '__main__':
    sys.exit(main())

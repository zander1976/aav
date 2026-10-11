#!/usr/bin/env python3
"""
Render docs/interfaces.yaml as docs/interfaces.md. No ROS needed.

usage: tools/gen_interfaces_md.py [docs/interfaces.yaml] > docs/interfaces.md
       tools/gen_interfaces_md.py --check    (exit 1 if interfaces.md is out of date)
"""

from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]


def cell(items):
    return ', '.join(f'`{i}`' for i in items) if items else '-'


def render(spec):
    out = ['# Interfaces', '',
           'Generated from `docs/interfaces.yaml` by `tools/gen_interfaces_md.py`; edit the YAML.',
           'Check a running system against it with '
           '`pixi run check <turtle|sim|road|car|replay|bench>`.',
           '`road` is the sim in its road world (`world:=road`): what is live in `sim` must be '
           'live there too.',
           '', '## Topics', '',
           '| Topic | Type | Publishers | Subscribers | Must be live in | Notes |',
           '|---|---|---|---|---|---|']
    for name, t in spec['topics'].items():
        out.append(f"| `{name}` | `{t['type']}` | {cell(t.get('pub'))} | {cell(t.get('sub'))} "
                   f"| {', '.join(t.get('live', [])) or '-'} | {t.get('note', '')} |")
    for section in ('services', 'actions'):
        out += ['', f'## {section.capitalize()}', '', '| Name | Type | Server | Notes |',
                '|---|---|---|---|']
        for name, s in spec[section].items():
            out.append(f"| `{name}` | `{s['type']}` | `{s['server']}` | {s.get('note', '')} |")
    out += ['', '## Frames', '', '| Frame | Parent | Published by | Notes |', '|---|---|---|---|']
    for name, f in spec['frames'].items():
        out.append(f"| `{name}` | {f.get('parent') or '-'} | {f.get('by', '-')} "
                   f"| {f.get('note', '')} |")
    return '\n'.join(out) + '\n'


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    src = Path(args[0]) if args else ROOT / 'docs' / 'interfaces.yaml'
    text = render(yaml.safe_load(src.read_text()))
    if '--check' in sys.argv:
        current = (ROOT / 'docs' / 'interfaces.md').read_text()
        if current != text:
            sys.exit('docs/interfaces.md is out of date: run tools/gen_interfaces_md.py '
                     '> docs/interfaces.md')
        return
    sys.stdout.write(text)


if __name__ == '__main__':
    main()

"""
The MCP server's parts that need neither ROS nor MCP.

So they're tested on any computer (tools/tests/test_mcp_parts.py). The server itself is
tools/mcp_server.py.

- which files the server may read, and finding one by a short name;
- searching them;
- the contract (docs/interfaces.yaml): everything, or what one name touches;
- the QoS to read a topic with, from its publishers' QoS;
- the running graph as text, and long answers cut to size.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

ROOT = Path(__file__).resolve().parents[1]

# The longest answer the server sends, in characters (about 5,000 words).
MAX_ANSWER = 30_000

# The three contracts, readable alongside the Markdown.
CONTRACTS = ('docs/interfaces.yaml', 'firmware/can/aav.dbc',
             'src/aav_description/config/vehicle.yaml')

# Calls that would write to the ROS graph or the car. tools/mcp_server.py must contain
# none of them; the tests check.
WRITES = ('create_publisher', 'create_client', 'create_service', 'ActionClient',
          'ActionServer', 'set_parameters', 'call_async', 'send_goal', '.publish(')

# Every node has these services; listing them hides the ones that say something.
PARAMETER_SERVICES = ('/describe_parameters', '/get_parameter_types', '/get_parameters',
                      '/list_parameters', '/set_parameters', '/set_parameters_atomically',
                      '/get_type_description')

PROFILES = ('turtle', 'sim', 'road', 'car', 'replay', 'bench')


def readable_files(root: Path = ROOT) -> List[str]:
    """
    List every file the server may read, as paths from the repo root.

    Only files in git: the Markdown (not the built HTML) and the three contracts. So
    untracked files, such as the firmware's secrets.h or vendor/'s source, are never
    offered.
    """
    out = subprocess.run(['git', 'ls-files', '--', '*.md', *CONTRACTS], cwd=root,
                         capture_output=True, text=True)
    if out.returncode != 0:
        return []
    return sorted(f for f in out.stdout.splitlines() if not f.startswith('docs/html/'))


def find_page(name: str, files: Sequence[str]) -> Optional[str]:
    """
    Find the file a short name means: None if there isn't one.

    'docs/perception.md' (exact), 'perception' or 'perception.md' (a docs page),
    'README' (the root folder first), then any one file with that name.
    """
    name = name.strip().strip('/')
    if name in files:
        return name
    stem = name[:-3] if name.lower().endswith('.md') else name
    for candidate in (f'docs/{stem}.md', f'{stem}.md', f'docs/history/{stem}.md'):
        for f in files:
            if f.lower() == candidate.lower():
                return f
    same = [f for f in files if Path(f).stem.lower() == stem.lower()
            or Path(f).name.lower() == name.lower()]
    return same[0] if len(same) == 1 else None


def search(text: str, files: Sequence[str], root: Path = ROOT, limit: int = 40) -> List[str]:
    """Find the lines that contain `text` (any case), as 'path:line: text', at most `limit`."""
    want = text.strip().lower()
    hits: List[str] = []
    if not want:
        return hits
    for f in files:
        try:
            lines = (root / f).read_text(encoding='utf-8', errors='replace').splitlines()
        except OSError:
            continue
        for number, line in enumerate(lines, 1):
            if want in line.lower():
                hits.append(f'{f}:{number}: {line.strip()[:200]}')
                if len(hits) >= limit:
                    return hits
    return hits


def load_contract(root: Path = ROOT) -> dict:
    return yaml.safe_load((root / 'docs' / 'interfaces.yaml').read_text())


def contract_summary(spec: dict) -> str:
    """Give one line for each topic, service and action: its name and type."""
    lines = []
    for section in ('topics', 'services', 'actions'):
        entries = spec.get(section) or {}
        lines.append(f'{section} ({len(entries)}):')
        lines += [f'  {name}  {entry.get("type", "?")}' for name, entry in entries.items()]
    frames = spec.get('frames') or {}
    lines.append(f'frames ({len(frames)}): ' + ', '.join(frames))
    return '\n'.join(lines)


def contract_for(spec: dict, name: str) -> Optional[str]:
    """
    Say what the contract says about one name, as YAML: None if it says nothing.

    A topic, service or action ('/cmd_vel/joy', or 'cmd_vel/joy'), or a frame, gives
    its entry. A node ('twist_mux') gives every entry it publishes, subscribes or serves.
    """
    name = name.strip()
    found: Dict[str, dict] = {}
    slashed = name if name.startswith('/') else '/' + name
    for section in ('topics', 'services', 'actions', 'frames'):
        entries = spec.get(section) or {}
        for key in (name, slashed):
            if key in entries:
                found.setdefault(section, {})[key] = entries[key]
    if not found:
        for section in ('topics', 'services', 'actions'):
            for key, entry in (spec.get(section) or {}).items():
                nodes = list(entry.get('pub') or []) + list(entry.get('sub') or [])
                if entry.get('server'):
                    nodes.append(entry['server'])
                if name in nodes:
                    found.setdefault(section, {})[key] = entry
    if not found:
        return None
    return yaml.safe_dump(found, sort_keys=False, allow_unicode=True, width=100)


def choose_qos(publishers: Sequence[Tuple[bool, bool]]) -> Tuple[bool, bool]:
    """
    How to subscribe so every publisher's messages arrive: (reliable, transient_local).

    `publishers` holds each publisher's (reliable, transient_local). The same rule as
    `ros2 topic echo`: ask for reliable or transient_local only if every publisher
    offers it, since a stricter subscriber than its publisher receives nothing.
    """
    if not publishers:
        return False, False
    return all(r for r, _ in publishers), all(t for _, t in publishers)


def graph_text(nodes: Sequence[str],
               topics: Dict[str, Tuple[Sequence[str], Sequence[str], Sequence[str]]],
               services: Dict[str, Sequence[str]],
               where: str) -> str:
    """
    Write the running graph as plain text.

    topics maps a name to (types, publishers, subscribers); services maps a name to its
    types. `where` says which ROS network this is (domain, discovery range).
    """
    lines = [where, '', f'nodes ({len(nodes)}):']
    lines += [f'  {n}' for n in nodes] or ['  (none)']
    lines += ['', f'topics ({len(topics)}):']
    for name in sorted(topics):
        types, pubs, subs = topics[name]
        lines.append(f'  {name}  {", ".join(types)}')
        lines.append(f'      pub: {", ".join(pubs) or "-"}   sub: {", ".join(subs) or "-"}')
    shown = sorted(s for s in services if not s.endswith(PARAMETER_SERVICES)
                   and '/_action/' not in s)
    lines += ['', f"services ({len(shown)}, not counting each node's parameter services):"]
    lines += [f'  {s}  {", ".join(services[s])}' for s in shown] or ['  (none)']
    return '\n'.join(lines)


def shorten(text: str, limit: int = MAX_ANSWER) -> str:
    """Cut a long answer to `limit` characters, and say how much was cut."""
    if len(text) <= limit:
        return text
    return text[:limit] + f'\n… (cut: {len(text) - limit} more characters)'

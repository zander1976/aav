#!/usr/bin/env python3
"""
The AAV MCP server: AI apps read this project and the live ROS graph through it.

Claude, or another AI app, starts it and talks to it over stdio. It only reads. How to
connect an app: docs/mcp.md.

    pixi run -e mcp mcp      (an AI app starts it, through .mcp.json, and talks over stdio)

Tools:
  list_pages, read_page, search_pages   the docs, README, CLAUDE.md, TODO.md, the contracts
  contract                              docs/interfaces.yaml: a summary, or one name
  repo_status                           the branch, the last commits, uncommitted files
  ros_graph                             the running nodes, topics and services
  read_topic                            one message from a topic
  check_contract                        the running graph against the contract
  run_tests                             tools/test.sh: the tests that don't need ROS

It never sends anything to a ROS node: no publishing, no service or action calls, no
parameters. So it can't move the car. Its ROS node has no publishers of its own; ROS
gives every node one for /parameter_events, which sends only when a parameter changes,
and this node's never do. It sees whichever ROS network it's started on: the car's, when
this computer is on it.

The parts that need neither ROS nor MCP are in tools/mcp_parts.py, with their tests.
"""

import atexit
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from mcp.server.mcpserver import MCPServer
from mcp.types import ToolAnnotations

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mcp_parts as parts  # noqa: E402

ROOT = parts.ROOT
# Seconds to let ROS discovery find the other nodes, after this server's node starts.
DISCOVERY_WAIT = float(os.environ.get('AAV_MCP_DISCOVERY_WAIT', '3'))

INSTRUCTIONS = """\
The Carleton AAV 2026-27 autonomy stack: ROS 2 Jazzy, Python nodes, Rust firmware.
Start with read_page('README'), then read_page('how-it-fits') and read_page('status').
CLAUDE.md has the project's rules; docs/interfaces.yaml is the topic contract.
Every tool only reads. The ROS tools see the network this server was started on: the
car's, if this computer is on it."""

server = MCPServer('aav', title='AAV project (read-only)', instructions=INSTRUCTIONS)
FILES = ToolAnnotations(read_only_hint=True, open_world_hint=False)
ROS = ToolAnnotations(read_only_hint=True, open_world_hint=True)


def network() -> str:
    """Say which ROS network this server sees."""
    scope = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') or 'SUBNET (the default)'
    where = f'ROS domain {os.environ.get("ROS_DOMAIN_ID") or "0"}, discovery range {scope}'
    if os.environ.get('ROS_STATIC_PEERS'):
        where += f', static peers {os.environ["ROS_STATIC_PEERS"]}'
    return where


# --- the project's files ---

@server.tool(annotations=FILES, structured_output=False)
def list_pages() -> str:
    """List every file read_page can open: the Markdown in git, and the three contracts."""
    files = parts.readable_files()
    return '\n'.join(files) if files else 'No files: is this a git checkout?'


@server.tool(annotations=FILES, structured_output=False)
def read_page(name: str) -> str:
    """
    Read one file.

    `name` is a path from list_pages, or a short name: 'perception' (a docs page),
    'README', 'CLAUDE', 'TODO', 'interfaces.yaml', 'vehicle.yaml', 'aav.dbc'.
    """
    files = parts.readable_files()
    found = parts.find_page(name, files)
    if found is None:
        near = [f for f in files if name.strip().lower() in f.lower()][:10]
        return f'No page called {name!r}.' + (f' Did you mean: {", ".join(near)}?' if near
                                              else ' list_pages lists them all.')
    return parts.shorten(f'# file: {found}\n\n' + (ROOT / found).read_text(encoding='utf-8'))


@server.tool(annotations=FILES, structured_output=False)
def search_pages(text: str) -> str:
    """Find the lines, in every readable file, that contain `text` (any case)."""
    hits = parts.search(text, parts.readable_files())
    return '\n'.join(hits) if hits else f'Nothing contains {text!r}.'


@server.tool(annotations=FILES, structured_output=False)
def contract(name: str = '') -> str:
    """
    Look something up in the node-graph contract, docs/interfaces.yaml.

    No name: every topic, service and action with its type. A topic, service, action or
    frame ('/cmd_vel/joy'): its entry, with who publishes and subscribes, and the
    profiles where it must be live. A node ('twist_mux'): every entry it touches.
    """
    spec = parts.load_contract()
    if not name.strip():
        return parts.contract_summary(spec)
    return parts.contract_for(spec, name) or f'The contract says nothing about {name!r}.'


@server.tool(annotations=FILES, structured_output=False)
def repo_status() -> str:
    """Show the branch, the last 10 commits, the uncommitted files, and GitHub's state."""
    def git(*args):
        out = subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True)
        return out.stdout.strip() if out.returncode == 0 else ''

    lines = [f'branch: {git("branch", "--show-current") or "?"}', '', 'last commits:',
             git('log', '-10', '--format=  %h %ad %s', '--date=format:%d %b %H:%M'),
             '', 'uncommitted:', git('status', '--short') or '  (none)']
    counts = git('rev-list', '--left-right', '--count', 'origin/main...HEAD').split()
    if len(counts) == 2:
        lines += ['', f'against GitHub (as last fetched): {counts[1]} ahead, {counts[0]} behind']
    return '\n'.join(lines)


# --- the live ROS graph ---

class RosReader:
    """This server's one ROS node. It reads the graph, and subscribes for a moment."""

    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy = rclpy
        self.node = rclpy.create_node('aav_mcp', enable_rosout=False,
                                      start_parameter_services=False)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, name='ros', daemon=True).start()
        self.ready_at = time.monotonic() + DISCOVERY_WAIT
        atexit.register(self.close)

    def settle(self):
        """Wait until discovery has had time to find the other nodes."""
        time.sleep(max(0.0, self.ready_at - time.monotonic()))

    def graph(self):
        self.settle()
        me = self.node.get_name()
        nodes = sorted(f'{ns.rstrip("/")}/{n}'
                       for n, ns in self.node.get_node_names_and_namespaces() if n != me)
        topics = {}
        for name, types in self.node.get_topic_names_and_types():
            pubs = sorted({i.node_name for i in self.node.get_publishers_info_by_topic(name)}
                          - {me})
            subs = sorted({i.node_name for i in self.node.get_subscriptions_info_by_topic(name)}
                          - {me})
            topics[name] = (types, pubs, subs)
        return nodes, topics, dict(self.node.get_service_names_and_types())

    def one_message(self, topic: str, timeout: float) -> str:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py import message_to_yaml
        from rosidl_runtime_py.utilities import get_message

        self.settle()
        types = dict(self.node.get_topic_names_and_types()).get(topic)
        if not types:
            return f'{topic} is not on the graph right now ({network()}).'
        publishers = self.node.get_publishers_info_by_topic(topic)
        if not publishers:
            return f'{topic} ({types[0]}) has no publisher right now.'
        reliable, latched = parts.choose_qos([
            (p.qos_profile.reliability == ReliabilityPolicy.RELIABLE,
             p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL)
            for p in publishers])
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                        else DurabilityPolicy.VOLATILE))
        try:
            kind = get_message(types[0])
        except (AttributeError, ModuleNotFoundError, ValueError) as e:
            return f"Can't load the type {types[0]} ({e}). Is the workspace built?"
        got = threading.Event()
        box = []

        def keep(msg):
            if not box:
                box.append(msg)
                got.set()

        subscription = self.node.create_subscription(kind, topic, keep, qos)
        try:
            got.wait(timeout)
        finally:
            self.node.destroy_subscription(subscription)
        if not box:
            return (f'No message on {topic} in {timeout:g} s '
                    f'({len(publishers)} publisher(s), {types[0]}).')
        body = message_to_yaml(box[0], truncate_length=16)
        return f'# {topic} ({types[0]}), one message; long arrays cut to 16 items\n{body}'

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.rclpy.try_shutdown()
        except Exception:  # noqa: B902 - the process is ending anyway
            pass


_ros = None
_ros_lock = threading.Lock()


def ros() -> RosReader:
    """Start the ROS node the first time a ROS tool needs it, and return it."""
    global _ros
    with _ros_lock:
        if _ros is None:
            _ros = RosReader()
        return _ros


@server.tool(annotations=ROS, structured_output=False)
def ros_graph() -> str:
    """
    List the running ROS graph: its nodes, topics and services.

    Each topic comes with its type, publishers and subscribers. The first line says which
    ROS network this is.
    """
    nodes, topics, services = ros().graph()
    return parts.shorten(parts.graph_text(nodes, topics, services, network()))


@server.tool(annotations=ROS, structured_output=False)
def read_topic(topic: str, timeout_s: float = 5.0) -> str:
    """
    Read one message from a topic, as YAML.

    Long arrays (images, point clouds) are cut to 16 items. It waits up to `timeout_s`
    seconds (at most 30) for a message.
    """
    topic = '/' + topic.strip().lstrip('/')
    return parts.shorten(ros().one_message(topic, min(max(timeout_s, 0.5), 30.0)))


@server.tool(annotations=ROS, structured_output=False)
def check_contract(profile: str = 'car') -> str:
    """
    Compare the running graph with the contract, the way `pixi run check` does.

    It reports missing publishers, wrong types, and topics the contract doesn't list.
    `profile` is what is meant to be running: turtle, sim, road (the sim's road world), car,
    replay or bench.
    """
    if profile not in parts.PROFILES:
        return f'profile must be one of: {", ".join(parts.PROFILES)}'
    out = subprocess.run([sys.executable, str(ROOT / 'tools' / 'check_graph.py'),
                          '--profile', profile], cwd=ROOT, capture_output=True, text=True,
                         timeout=120)
    return parts.shorten(f'{network()}\n\n{out.stdout}{out.stderr[-2000:]}\n'
                         f'(check_graph.py exit status {out.returncode})')


@server.tool(annotations=ToolAnnotations(read_only_hint=False, destructive_hint=False,
                                         idempotent_hint=True, open_world_hint=False),
             structured_output=False)
def run_tests() -> str:
    """
    Run tools/test.sh: the tests that don't need ROS (a few minutes).

    The unit tests, the Rust tests and the contract checks. ROS discovery stays on this
    computer, and it writes only build output, which git ignores. It returns the last 150
    lines, and the exit status.
    """
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    env.pop('ROS_STATIC_PEERS', None)
    try:
        out = subprocess.run(['bash', 'tools/test.sh'], cwd=ROOT, env=env,
                             capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return 'tools/test.sh took more than 15 minutes, and was stopped.'
    tail = '\n'.join((out.stdout + out.stderr).splitlines()[-150:])
    verdict = 'passed' if out.returncode == 0 else 'FAILED'
    return f'{tail}\n\n(tools/test.sh exit status {out.returncode}: {verdict})'


if __name__ == '__main__':
    server.run()

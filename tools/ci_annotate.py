#!/usr/bin/env python3
"""
Turn a failed `pixi run test` into GitHub annotations, which anyone can read.

GitHub hides a job's log from anyone not signed in, but serves its annotations to all:
  https://api.github.com/repos/zander1976/aav/commits/<sha>/check-runs
  https://api.github.com/repos/zander1976/aav/check-runs/<id>/annotations
So the workflow runs this after a failed test step. It prints one `::error` line per
failing test case (from colcon's JUnit files in build/), and the end of the step's log
when no test case failed (tools/test.sh, lint or the build stopped first).

  python3 tools/ci_annotate.py [log] [build]      (defaults: test.log, build)
"""

import glob
import os
import sys
import xml.etree.ElementTree as ET

LIMIT = 9           # GitHub keeps 10 error annotations per step: the last is the log's end
MESSAGE_LINES = 25  # an annotation's message is cut at about 64 KB, but short reads better


def escape(text):
    """Escape text for a workflow command's message (GitHub's own rules)."""
    return text.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')


def escape_property(text):
    return escape(text).replace(':', '%3A').replace(',', '%2C')


def failures(build):
    """Yield (title, message) for each failing or erroring test case under build/."""
    for path in sorted(glob.glob(os.path.join(build, '*', 'test_results', '**', '*.xml'),
                                 recursive=True)):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            yield os.path.relpath(path, build), f'unreadable JUnit file: {e}'
            continue
        package = os.path.relpath(path, build).split(os.sep)[0]
        for case in root.iter('testcase'):
            for bad in list(case.findall('failure')) + list(case.findall('error')):
                name = case.get('name', '?')
                where = case.get('classname', '')
                body = (bad.get('message') or '') + '\n' + (bad.text or '')
                lines = [line for line in body.strip().splitlines() if line.strip()]
                # The end of a traceback says what went wrong; keep it.
                if len(lines) > MESSAGE_LINES:
                    lines = ['...'] + lines[-MESSAGE_LINES:]
                yield f'{package}: {where} {name}'.strip(), '\n'.join(lines)


def tail(log, count=40):
    try:
        with open(log, errors='replace') as f:
            return ''.join(f.readlines()[-count:])
    except OSError:
        return ''


def annotations(log, build):
    """Return the workflow command lines to print."""
    found = list(failures(build))
    out = [f'::error title={escape_property(t)}::{escape(m)}' for t, m in found[:LIMIT]]
    if len(found) > LIMIT:
        names = '\n'.join(t for t, _ in found[LIMIT:])
        out.append(f'::error title={len(found) - LIMIT} more failing tests::{escape(names)}')
    elif tail(log) or not found:
        end = tail(log) or f'no {log}, and no failing test case in {build}/'
        out.append(f'::error title=the end of the test step::{escape(end)}')
    return out


def main(argv):
    log = argv[1] if len(argv) > 1 else 'test.log'
    build = argv[2] if len(argv) > 2 else 'build'
    for line in annotations(log, build):
        print(line)


if __name__ == '__main__':
    main(sys.argv)

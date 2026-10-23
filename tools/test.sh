#!/usr/bin/env bash
# Run every test that doesn't need ROS: our packages' Python unit tests, the Rust spine
# library, the HUD's and the map window's logic (hud-core, map-core), and the check that docs/interfaces.md matches
# docs/interfaces.yaml. Where the mcp environment is installed, also the MCP server over
# stdio (on a private, local domain); where Flutter is (pixi run get-flutter), also the
# HUD app's tests (tools/hud_test.sh).
# Needs Python 3 with pytest and PyYAML; Rust (cargo) for the spine. Works on any OS.
#
# usage: tools/test.sh
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1

# Each package's tests import the package, so pytest runs from inside it.
# (aav_gazebo's adapter test imports ROS messages, so only `colcon test` runs that one.)
for pkg in src/ego/aav_ego src/mux/aav_io_mux src/backends/aav_spine src/backends/aav_legacy \
           src/backends/aav_gazebo src/backends/aav_carla \
           src/drivers/aav_scripted src/drivers/aav_language src/drivers/aav_navigation \
           src/drivers/aav_actuator_tests src/drivers/aav_goto src/drivers/aav_routes \
           src/aav_description \
           src/perception/aav_yolo \
           src/perception/aav_lane_finder src/perception/aav_lidar \
           src/safety/aav_safety src/safety/aav_ghost \
           src/bench/aav_bench src/hud/aav_hud; do
  echo "== $pkg"
  (cd "$pkg" && python3 -m pytest -q -p no:cacheprovider)
done

echo "== tools: the MCP server's parts, the Pico 2's UF2 converter, CI's annotations"
python3 -m pytest -q -p no:cacheprovider tools/tests

echo "== firmware/spine"
if command -v cargo >/dev/null; then
  (cd firmware/spine && cargo test -q)
  echo "== src/sensors/aav_imu_yahboom: the IMU's serial frames"
  (cd src/sensors/aav_imu_yahboom && cargo test -q -p yahboom-protocol)
  echo "== src/perception/aav_radar_clusters: DBSCAN, the boxes, the C++ node's answers"
  (cd src/perception/aav_radar_clusters && cargo test -q --locked -p radar-clustering)
  echo "== src/backends/aav_spine_bridge: the Rust CAN bridge's logic, the shared CAN vectors"
  (cd src/backends/aav_spine_bridge && cargo test -q --locked -p spine-bridge)
  # Only the logic: the window (raylib) needs a screen's libraries, and colcon builds it.
  echo "== src/hud/aav_hud_raylib: the HUD's state, GPS, e-stop and formatting; the map's maths"
  (cd src/hud/aav_hud_raylib && cargo test -q --locked -p hud-core -p map-core)
  # Every firmware crate must build without the standard library, for the Pico 2's chip.
  if [[ -d "$(rustc --print target-libdir --target thumbv8m.main-none-eabihf 2>/dev/null)" ]]; then
    echo "== firmware/spine, no_std, for the Pico 2 (thumbv8m.main-none-eabihf)"
    (cd firmware/spine && cargo build -q --target thumbv8m.main-none-eabihf -p spine-core \
       -p aav-inputs -p wheel-encoder -p sentinel -p wheel-core -p legacy-link -p jrk-g2 \
       -p ad5311 -p pca9555 -p mcp2515 -p ibus -p hd44780 -p spine-board \
       && echo "builds")
    # The board programs: each a workspace of its own, built only for the Pico 2's chip.
    # Their UF2 files too, the ones to drag on: tools/elf2uf2.py checks the boot ROM's
    # IMAGE_DEF. pico2 is the spine; the other two replace the 2025-26 C++ Pico 2s.
    for board in pico2:spine-pico2 wheel-encoder-pico2:wheel-encoder-pico2 \
                 sentinel-pico2:sentinel-pico2; do
      dir=${board%%:*} bin=${board##*:}
      echo "== firmware/spine/boards/$dir: a board program, for the Pico 2"
      (cd "firmware/spine/boards/$dir" && cargo build -q --release --locked && echo "builds")
      python3 tools/elf2uf2.py \
        "firmware/spine/boards/$dir/target/thumbv8m.main-none-eabihf/release/$bin"
    done
  fi
  # Today's ESP32 control board: only with Espressif's Xtensa Rust (pixi run esp-tools).
  echo "== firmware/spine/boards/esp32-umod: the spine on today's ESP32 control board"
  tools/esp32.sh check
else
  echo "skipped: no cargo here (pixi run test has it; or install Rust with rustup)"
fi

echo "== the stand-ins stay on this computer"
# turtle, sim, bench and view must never meet a real car on the same network and domain.
python3 - <<'PY'
import re
import sys
tasks = dict(re.findall(r'^([\w-]+) = \{ cmd = "([^"]*)"', open('pixi.toml').read(), re.M))
bad = [t for t in ('turtle', 'sim', 'bench', 'bench-check', 'view', 'laptop-bench',
                       'sim-check', 'sim-fault', 'carla-car', 'controller-compare',
                       'timing-test')
       if not tasks.get(t, '').startswith('tools/ros.sh --local ')]
sys.exit(f'these pixi tasks need "tools/ros.sh --local": {bad}' if bad else None)
PY
if [[ -f install/setup.bash ]]; then
  got=$(env -u ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS=192.0.2.1 tools/ros.sh --local \
        bash -c 'echo "$ROS_AUTOMATIC_DISCOVERY_RANGE ${ROS_STATIC_PEERS:-no-peers}"')
  [[ "$got" == "LOCALHOST no-peers" ]] || { echo "tools/ros.sh --local gave: $got"; exit 1; }
  # As pixi runs it: its activation has already set SUBNET (15 Oct: --local wasn't local).
  got=$(ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET tools/ros.sh --local \
        bash -c 'echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"')
  [[ "$got" == "LOCALHOST" ]] || { echo "tools/ros.sh --local kept SUBNET: $got"; exit 1; }
  got=$(AAV_DISCOVERY=SUBNET tools/ros.sh --local bash -c 'echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"')
  [[ "$got" == "SUBNET" ]] || { echo "tools/ros.sh --local ignored AAV_DISCOVERY: $got"; exit 1; }
fi
# The keyboard with --sim or --local stays here too (a stand-in ros2 prints what it got).
stub=$(mktemp -d)
trap 'rm -rf "$stub"' EXIT
printf '#!/usr/bin/env bash\necho "${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset} ${ROS_STATIC_PEERS:-no-peers}"\n' \
  > "$stub/ros2"
chmod +x "$stub/ros2"
for flag in --sim --local; do
  got=$(ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS=192.0.2.1 PATH="$stub:$PATH" \
        tools/keyboard.sh $flag)
  [[ "$got" == "LOCALHOST no-peers" ]] || { echo "tools/keyboard.sh $flag gave: $got"; exit 1; }
done
echo "they do"

echo "== the car kit's scripts (docs/car-install.md)"
for s in tools/car_kit.sh tools/car_install.sh tools/car_listen.sh tools/car_screen.sh \
         tools/car_screens_install.sh; do bash -n "$s"; done
# Away from a kit, the installer must stop at its checks, and say it changed nothing.
if out=$(bash tools/car_install.sh --dry-run --no-verify 2>&1); then
  echo "car_install.sh --dry-run passed with no kit: $out"; exit 1
fi
[[ "$out" == *"nothing was changed"* ]] || { echo "car_install.sh: $out"; exit 1; }
AAV_PREFIX=/nonexistent bash tools/car_listen.sh > /dev/null
echo "they parse, the installer refuses cleanly, and car_listen runs"

echo "== vendor/ stays out of the build"
# Without COLCON_IGNORE, pixi run build would try to build all of ROS 2's source.
[[ -f vendor/COLCON_IGNORE ]] || { echo "vendor/COLCON_IGNORE is missing"; exit 1; }
python3 -c "import glob, yaml; [yaml.safe_load(open(f))['repositories'] for f in glob.glob('vendor/*.repos')]"
echo "it does"

echo "== the MCP server, over stdio, as an AI app starts it"
if [[ -d .pixi/envs/mcp ]] && command -v pixi >/dev/null; then
  pixi run --frozen -e mcp python3 tools/tests/mcp_smoke.py
else
  echo "skipped: the mcp environment isn't installed (pixi install -e mcp)"
fi

echo "== hud/flutter_pi: the HUD app (flutter pub get, analyze, test)"
if [[ -x vendor/flutter/bin/flutter ]]; then
  tools/hud_test.sh
else
  echo "skipped: no Flutter in vendor/flutter (pixi run get-flutter)"
fi

echo "== docs/interfaces.md"
python3 tools/gen_interfaces_md.py --check && echo "in sync"

#!/usr/bin/env bash
# bb_robotx_dashboard runtime bootstrap.
#
# Runs once per container shell (hooked from ~/.bashrc). Generates the protobuf
# bindings the dashboard backend needs, against the dashboard source in the
# mounted workspace (the source is NOT baked into the image). Every step is
# idempotent and best-effort (|| true) so a missing network never blocks the
# shell. The frontend build and colcon build are intentionally left to the
# user — see the reminder printed at the end.
#
# Modeled on ros2-docker/environments/multivehicle_sim/on_entry.sh, adapted to
# the rocker mount layout ($HOME mounted at /root/HOST).
set -uo pipefail

# Locate the dashboard package in the mounted workspace. Override with
# DASHBOARD_PKG_DIR if your workspace lives somewhere else.
PKG_DIR="${DASHBOARD_PKG_DIR:-/root/HOST/dave_ws/src/bb_robotx_dashboard}"
if [ ! -d "${PKG_DIR}" ]; then
    # Fall back to a shallow search under the mounted home.
    PKG_DIR="$(find /root/HOST -maxdepth 5 -type d -name bb_robotx_dashboard \
        -not -path '*/build/*' -not -path '*/install/*' 2>/dev/null | head -n1)"
fi

# No dashboard checked out — nothing to bootstrap, stay quiet.
[ -n "${PKG_DIR}" ] && [ -d "${PKG_DIR}" ] || exit 0

did_work=0

# 1. Clone the upstream robocommand repo (provides the .proto source).
if [ ! -d "${PKG_DIR}/third_party/robocommand/proto" ]; then
    echo "[setup_dashboard] cloning robocommand proto source..."
    git clone https://github.com/MonkeScripts/robocommand \
        "${PKG_DIR}/third_party/robocommand" || true
    did_work=1
fi

# 2. Wire the image's pinned protoc 33 into third_party/protoc/ so
#    compile_protos.sh picks the local (edition-2024-capable) binary instead of
#    the system protoc 3.x. We symlink the baked /opt/protoc rather than
#    re-downloading it over the network.
if [ ! -x "${PKG_DIR}/third_party/protoc/bin/protoc" ] \
        && [ -x /opt/protoc/bin/protoc ]; then
    mkdir -p "${PKG_DIR}/third_party/protoc/bin"
    ln -sf /opt/protoc/bin/protoc "${PKG_DIR}/third_party/protoc/bin/protoc"
fi

# 3. Generate the Python protobuf bindings if missing.
if [ -d "${PKG_DIR}/third_party/robocommand/proto" ] \
        && [ ! -f "${PKG_DIR}/bb_robotx_dashboard/proto/report_pb2.py" ]; then
    echo "[setup_dashboard] compiling proto bindings..."
    ( cd "${PKG_DIR}" && bash scripts/compile_protos.sh ) || true
    did_work=1
fi

# 4. Remind about the manual steps if the frontend hasn't been built yet.
if [ ! -d "${PKG_DIR}/frontend/dist" ]; then
    cat <<MSG
[setup_dashboard] dashboard tooling ready. Remaining manual steps:
  cd ${PKG_DIR}/frontend && npm ci && npm run build
  cd ~/dave_ws && colcon build --packages-up-to bb_robotx_dashboard && source install/setup.bash
MSG
elif [ "${did_work}" -eq 1 ]; then
    echo "[setup_dashboard] proto bindings generated. Rebuild if needed: colcon build --packages-up-to bb_robotx_dashboard"
fi

exit 0

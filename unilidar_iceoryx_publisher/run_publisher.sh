#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0 OR MIT
# Helper script to run the Unilidar iceoryx2 publisher with the correct
# runtime library path so that the iceoryx shared libraries are found.

set -euo pipefail

# Directory containing this script (project root for the publisher)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# iceoryx2 shared libraries location – adjust if your build dir changes
ICEORYX_HOOFS_LIB_DIR="$HOME/iceoryx2/target/iceoryx/build/hoofs"
ICEORYX_PLATFORM_LIB_DIR="$HOME/iceoryx2/target/iceoryx/build/platform"
ICEORYX_INSTALL_LIB_DIR="$HOME/iceoryx2/target/iceoryx/install/lib"

export LD_LIBRARY_PATH="${ICEORYX_HOOFS_LIB_DIR}:${ICEORYX_PLATFORM_LIB_DIR}:${ICEORYX_INSTALL_LIB_DIR}:${LD_LIBRARY_PATH:-}"

echo "LD_LIBRARY_PATH set to: $LD_LIBRARY_PATH"

exec "${SCRIPT_DIR}/build/unilidar_publisher" "$@" 
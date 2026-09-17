#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
telemetry_test="${TMPDIR:-/tmp}/temperature_gateway_telemetry_test"
control_test="${TMPDIR:-/tmp}/temperature_gateway_control_test"
remote_test="${TMPDIR:-/tmp}/temperature_gateway_remote_test"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$project_dir/main" \
   "$project_dir/main/telemetry_protocol.c" \
   "$project_dir/tests/test_telemetry_protocol.c" \
   -lm -o "$telemetry_test"
"$telemetry_test"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$project_dir/main" \
   "$project_dir/main/control_protocol.c" \
   "$project_dir/tests/test_control_protocol.c" \
   -o "$control_test"
"$control_test"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$project_dir/../Core/Inc" \
   "$project_dir/../Core/Src/remote_control.c" \
   "$project_dir/tests/test_remote_control.c" \
   -o "$remote_test"
"$remote_test"

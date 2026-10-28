#!/usr/bin/env bash
# The build wrapper (one lock per machine, half the cores, nice/ionice, scratch
# off tmpfs) lives in espOS; this keeps the old path working. Same arguments.
exec "$(dirname "$0")/../espos/scripts/build.sh" "$@"

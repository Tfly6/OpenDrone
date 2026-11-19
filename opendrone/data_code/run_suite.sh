#!/usr/bin/env bash
set -e
export OPENDRONE_DATA_ENTRY=run_suite.py
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_revision.sh" "$@"

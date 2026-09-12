#!/usr/bin/env bash
set -euo pipefail
task_root="$(rtk proxy dirname "$(rtk proxy dirname "$(rtk proxy realpath "$0")")")"
exec rtk proxy "$task_root/.venv/bin/python" -I "$@"

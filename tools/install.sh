#!/usr/bin/env bash
set -euo pipefail
task_root="$(rtk proxy dirname "$(rtk proxy dirname "$(rtk proxy realpath "$0")")")"
cd "$task_root"
if [ -L .venv ]; then
  rtk proxy printf '%s\n' 'Refusing a symlinked .venv; create an independent local environment.'
  exit 1
fi
if [ ! -x .venv/bin/python ]; then
  rtk proxy python3 -m venv --without-pip .venv
fi
rtk proxy .venv/bin/python -I -c 'import pathlib,sys; expected=pathlib.Path.cwd()/".venv"; assert pathlib.Path(sys.prefix).resolve()==expected.resolve(), "Refusing external environment"'
if ! rtk proxy .venv/bin/python -I -m pip --version >/dev/null 2>&1; then
  if [ -d /usr/lib/python3/dist-packages/pip ]; then
    rtk proxy .venv/bin/python -I /usr/lib/python3/dist-packages/pip install --isolated --ignore-installed pip==25.2
  else
    rtk proxy .venv/bin/python -I -m ensurepip
  fi
fi
rtk proxy .venv/bin/python -I -m pip install --isolated -r requirements.lock
if [ ! -f dependencies/compliant_control_core-0.5.1+7aec01379ff9-py3-none-any.whl ]; then
  rtk proxy .venv/bin/python -I tools/build_upstream_wheel.py
fi
rtk proxy .venv/bin/python -I -m pip install --isolated --no-deps dependencies/compliant_control_core-0.5.1+7aec01379ff9-py3-none-any.whl
rtk proxy .venv/bin/python -I -m pip install --isolated --no-deps --no-build-isolation -e .
rtk proxy .venv/bin/python -I -m pip check

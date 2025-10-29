#!/usr/bin/env bash
# SenseBeetle workspace environment loader (root only)
# 在仓库根目录仅 source 本目录下的 install/setup.bash；
# 进入仿真或 single_robot_explore 目录时，由各自目录的 .envrc/enter_env.sh 重新加载并覆盖环境。

# 防止重复加载
if [[ -n "$SERVER_SENSEBEETLE_ENV" ]]; then
  return 0
fi

# Resolve repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

info() { echo -e "[sensebeetle-env] $*"; }

if [[ -f "install/setup.bash" ]]; then
  info "sourcing SenseServer install/setup.bash"
  # shellcheck disable=SC1090
  source "install/setup.bash"
else
  info "skip SenseServer install/setup.bash (not found)"
fi

export SERVER_SENSEBEETLE_ENV=1
info "environment ready (root overlay only)"

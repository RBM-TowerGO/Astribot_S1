#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$ROOT/activate.sh"
mkdir -p "$ROOT/cache/build-tmp"
export TMPDIR="$ROOT/cache/build-tmp"
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=8.7 FORCE_CUDA=1 MAX_JOBS=4 BUILD_VERSION=0.20.1
export PYTORCH_VERSION=$(python -c 'from importlib.metadata import version; print(version("torch"))')
cd "$ROOT/assets/downloads/vision-0.20.1"
python -m pip wheel --no-deps --no-build-isolation . -w "$ROOT/assets/downloads"
python -m pip install --no-deps "$ROOT/assets/downloads/torchvision-0.20.1-cp310-cp310-linux_aarch64.whl"

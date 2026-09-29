# Native (non-Docker) environment for snitch_cluster.
# Source this from the repository root, do NOT add to ~/.zshrc:
#
#   source ./env-native.sh
#
# Undo with:
#
#   snitch_deactivate
#
# Mirrors the ENV settings from util/container/Dockerfile (hw stage)
# and the PATH setup from iis-setup.sh, adapted for a native Arch/CachyOS
# install with tools in ~/tools.
#
# Skipped per local setup: doxygen, bender, yosys installs.
#
# Extra Verilator flags (e.g. -fno-dfg, needed with Verilator 5.052, see
# make verilator debugging notes): export them so the makefile's `+=`
# appends instead of replacing. Do NOT pass SN_VLT_FLAGS on the make
# command line -- that discards the makefile's own flags (--timing, ...).
#   export SN_VLT_FLAGS="-fno-dfg"

# Deactivate function (venv-activate style). Restores PATH and unsets or
# restores every variable this script exports. Removes itself unless called
# as `snitch_deactivate nondestructive`.
snitch_deactivate() {
    if [ -n "${_SNITCH_ENV_ACTIVE:-}" ]; then
        PATH="$_SNITCH_OLD_PATH"
        export PATH
        if [ -n "${_SNITCH_OLD_SN_LLVM_BINROOT_SET:-}" ]; then
            SN_LLVM_BINROOT="$_SNITCH_OLD_SN_LLVM_BINROOT"
            export SN_LLVM_BINROOT
        else
            unset SN_LLVM_BINROOT
        fi
        if [ -n "${_SNITCH_OLD_SN_ROOT_SET:-}" ]; then
            SN_ROOT="$_SNITCH_OLD_SN_ROOT"
            export SN_ROOT
        else
            unset SN_ROOT
        fi
        if [ -n "${_SNITCH_OLD_UV_PROJECT_ENVIRONMENT_SET:-}" ]; then
            UV_PROJECT_ENVIRONMENT="$_SNITCH_OLD_UV_PROJECT_ENVIRONMENT"
            export UV_PROJECT_ENVIRONMENT
        else
            unset UV_PROJECT_ENVIRONMENT
        fi
        if [ -n "${_SNITCH_OLD_UV_CACHE_DIR_SET:-}" ]; then
            UV_CACHE_DIR="$_SNITCH_OLD_UV_CACHE_DIR"
            export UV_CACHE_DIR
        else
            unset UV_CACHE_DIR
        fi
        if [ -n "${_SNITCH_OLD_UV_PYTHON_INSTALL_DIR_SET:-}" ]; then
            UV_PYTHON_INSTALL_DIR="$_SNITCH_OLD_UV_PYTHON_INSTALL_DIR"
            export UV_PYTHON_INSTALL_DIR
        else
            unset UV_PYTHON_INSTALL_DIR
        fi
        unset _SNITCH_OLD_PATH
        unset _SNITCH_OLD_SN_LLVM_BINROOT _SNITCH_OLD_SN_LLVM_BINROOT_SET
        unset _SNITCH_OLD_SN_ROOT _SNITCH_OLD_SN_ROOT_SET
        unset _SNITCH_OLD_UV_PROJECT_ENVIRONMENT _SNITCH_OLD_UV_PROJECT_ENVIRONMENT_SET
        unset _SNITCH_OLD_UV_CACHE_DIR _SNITCH_OLD_UV_CACHE_DIR_SET
        unset _SNITCH_OLD_UV_PYTHON_INSTALL_DIR _SNITCH_OLD_UV_PYTHON_INSTALL_DIR_SET
        unset _SNITCH_ENV_ACTIVE
    fi
    if [ "${1:-}" != "nondestructive" ]; then
        unset -f snitch_deactivate 2>/dev/null || unset snitch_deactivate
    fi
}

# Re-sourcing must not clobber the saved originals with already-modified
# values: quietly roll back first, keeping this function around.
if [ -n "${_SNITCH_ENV_ACTIVE:-}" ]; then
    snitch_deactivate nondestructive
fi

# Save pristine state before overwriting it.
_SNITCH_OLD_PATH="$PATH"
if [ -n "${SN_LLVM_BINROOT+x}" ]; then
    _SNITCH_OLD_SN_LLVM_BINROOT="$SN_LLVM_BINROOT"
    _SNITCH_OLD_SN_LLVM_BINROOT_SET=1
fi
if [ -n "${SN_ROOT+x}" ]; then
    _SNITCH_OLD_SN_ROOT="$SN_ROOT"
    _SNITCH_OLD_SN_ROOT_SET=1
fi
if [ -n "${UV_PROJECT_ENVIRONMENT+x}" ]; then
    _SNITCH_OLD_UV_PROJECT_ENVIRONMENT="$UV_PROJECT_ENVIRONMENT"
    _SNITCH_OLD_UV_PROJECT_ENVIRONMENT_SET=1
fi
if [ -n "${UV_CACHE_DIR+x}" ]; then
    _SNITCH_OLD_UV_CACHE_DIR="$UV_CACHE_DIR"
    _SNITCH_OLD_UV_CACHE_DIR_SET=1
fi
if [ -n "${UV_PYTHON_INSTALL_DIR+x}" ]; then
    _SNITCH_OLD_UV_PYTHON_INSTALL_DIR="$UV_PYTHON_INSTALL_DIR"
    _SNITCH_OLD_UV_PYTHON_INSTALL_DIR_SET=1
fi
_SNITCH_ENV_ACTIVE=1

# Resolve the repo root from this file's location (works in bash and zsh).
if [ -n "${BASH_SOURCE:-}" ]; then
    _sn_file="${BASH_SOURCE[0]}"
else
    _sn_file="$0"
fi
export SN_ROOT="$(cd "$(dirname "$_sn_file")" && pwd)"
unset _sn_file

# PULP LLVM toolchain (22.1.7-pulp-0.1.0) installed natively in ~/tools.
export SN_LLVM_BINROOT="$HOME/tools/riscv-llvm/bin"

# Simulator / build outputs produced by `make` (cf. common.mk SN_BIN_DIR).
export PATH="$SN_ROOT/target/sim/build/bin:$PATH"

# Toolchain + user tool binaries first so they shadow system ones.
export PATH="$SN_LLVM_BINROOT:$HOME/tools/bin:$PATH"

# Python environment, mirroring the Dockerfile UV_* settings: venv and uv
# caches live inside the repo so they persist across shells.
export UV_PROJECT_ENVIRONMENT="$SN_ROOT/.venv"
export UV_CACHE_DIR="$SN_ROOT/.cache/uv"
export UV_PYTHON_INSTALL_DIR="$SN_ROOT/.local/share/uv/python"
export PATH="$SN_ROOT/.venv/bin:$PATH"

# Verilator DFG bug in 5.0.52
export SN_VLT_FLAGS="-fno-dfg"
echo "snitch native env ready: SN_ROOT=$SN_ROOT (undo with snitch_deactivate)"
command -v riscv32-unknown-elf-clang >/dev/null && echo "  LLVM: $(riscv32-unknown-elf-clang --version | head -n1)"
command -v verilator >/dev/null && echo "  verilator: $(verilator --version)"

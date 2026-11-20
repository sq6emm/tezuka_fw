#!/usr/bin/env bash
# Regression guard against dead Kconfig symbols (from upstream tezuka_fw,
# adapted to this tree's boards.json flavours).
#
# A line like "BR2_PACKAGE_FOO=y" in a defconfig only warns (never errors)
# if BR2_PACKAGE_FOO isn't a real Kconfig symbol: Buildroot's olddefconfig
# silently drops it from the generated .config.
#
# For each image this runs "make <defconfig>" (plus the flavour fragment,
# as build.sh does) into a scratch output directory, so the real
# output/<board>/ trees are not touched, and fails if any "BR2_*=y" line
# of the defconfig or the fragment is missing from the resulting .config.
#
# Usage:
#   tools/check_defconfig_symbols.sh <image> [image2 ...]   (names from boards.json)
#   tools/check_defconfig_symbols.sh all

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILDROOT_DIR="${SCRIPT_DIR}/buildroot"

command -v jq >/dev/null 2>&1 || { echo "ERROR: jq required to read boards.json" >&2; exit 1; }
[ -d "${BUILDROOT_DIR}" ] || { echo "ERROR: buildroot/ not found. Run ./getbuildroot.sh first." >&2; exit 1; }

if [ -z "${BR2_EXTERNAL:-}" ]; then
    # shellcheck source=../sourceme.first
    source "${SCRIPT_DIR}/sourceme.first"
fi

declare -A DEFCONFIG=() FLAVOUR=()
while IFS=$'\t' read -r _board _defconfig _flavour; do
    DEFCONFIG[$_board]="$_defconfig"
    FLAVOUR[$_board]="$_flavour"
done < <(jq -r '.[] | [.board, .defconfig, (.flavour // "")] | @tsv' "${SCRIPT_DIR}/boards.json")

[ $# -eq 0 ] && { echo "Usage: $0 <image|all> [image2 ...]" >&2; exit 1; }
if [ "$1" = "all" ]; then TARGETS=("${!DEFCONFIG[@]}"); else TARGETS=("$@"); fi

SCRATCH="$(mktemp -d)"
trap 'rm -rf "${SCRATCH}"' EXIT

FAIL=0
for board in "${TARGETS[@]}"; do
    defconfig="${DEFCONFIG[$board]:-}"
    if [ -z "$defconfig" ]; then
        echo "ERROR: unknown image '${board}'" >&2
        FAIL=1
        continue
    fi
    flavour="${FLAVOUR[$board]}"
    out="${SCRATCH}/${board}"
    sources=("${SCRIPT_DIR}/configs/${defconfig}")
    echo "=== ${board} (${defconfig}${flavour:+ + flavour/${flavour}.config}) ==="
    make -C "${BUILDROOT_DIR}" O="${out}" "${defconfig}" >/dev/null
    if [ -n "${flavour}" ]; then
        sources+=("${SCRIPT_DIR}/configs/flavour/${flavour}.config")
        cat "${SCRIPT_DIR}/configs/flavour/${flavour}.config" >> "${out}/.config"
        make -C "${BUILDROOT_DIR}" O="${out}" olddefconfig >/dev/null
    fi

    dead=0
    for src in "${sources[@]}"; do
        while read -r sym; do
            if ! grep -q "^${sym}=y$" "${out}/.config"; then
                echo "  DEAD SYMBOL: ${sym}=y (in ${src#"${SCRIPT_DIR}"/}, absent from the generated .config)"
                dead=1
            fi
        done < <(grep -E '^BR2_[A-Z0-9_]+=y$' "${src}" | cut -d= -f1)
    done

    if [ "$dead" -eq 1 ]; then FAIL=1; else echo "  OK: no dead symbols"; fi
done

exit "$FAIL"

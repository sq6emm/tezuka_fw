#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILDROOT_DIR="${SCRIPT_DIR}/buildroot"

# Image name (<board>-<flavour>, docs/FLAVOURS.md) -> defconfig, flavour and
# output directory, loaded from boards.json (single source of truth shared
# with .github/workflows/main.yml). A flavour is configs/flavour/<f>.config,
# appended to the board's defconfig. The plus images keep the old output
# directory (output/<board>); a bare board name means its first image there
# (libre = libre-plus, pluto = pluto-basic).
if ! command -v jq >/dev/null 2>&1; then
    echo "ERROR: jq required to read boards.json" >&2
    exit 1
fi
declare -A BOARDS=()
declare -A ARTIFACT=()   # board -> release-artifact group name (defaults to board)
declare -A FLAVOUR=()    # image -> flavour (configs/flavour/<f>.config)
declare -A OUTNAME=()    # image -> output/<dir>
declare -A ALIAS=()      # bare board name -> its default image
declare -A HW=()         # image -> board/tezuka/<hw> (its bitstreams, DTS)
while IFS=$'\t' read -r _board _defconfig _artifact _flavour _out _hw; do
    BOARDS[$_board]=$_defconfig
    HW[$_board]=$_hw
    ARTIFACT[$_board]=$_artifact
    FLAVOUR[$_board]=$_flavour
    OUTNAME[$_board]=$_out
    [ -n "${ALIAS[$_hw]+x}" ] || ALIAS[$_hw]=$_board
done < <(jq -r '.[] | [.board, .defconfig, (.artifact // .board), (.flavour // ""), (.output // .board), (.hw // .board)] | @tsv' "${SCRIPT_DIR}/boards.json")

# Artifact groups with more than one member board (e.g. fishball_mini_7010's
# flash-only build merges into fishball's own zip) get merged into one zip;
# see merge_group() below. A group's name is often a real board itself (its
# "primary" member), not just a synthetic label.
declare -A GROUP_MEMBERS=()  # artifact name -> space-separated member boards
for board in "${!BOARDS[@]}"; do
    GROUP_MEMBERS[${ARTIFACT[$board]}]+="${board} "
done
declare -A MERGE_GROUPS=()  # artifact names that have >1 member (i.e. real groups)
for artifact in "${!GROUP_MEMBERS[@]}"; do
    # shellcheck disable=SC2206
    members=(${GROUP_MEMBERS[$artifact]})
    [ "${#members[@]}" -gt 1 ] && MERGE_GROUPS[$artifact]=1
done

usage() {
    echo "Usage: $0 [OPTIONS] <board|group|all> [board2 ...]"
    echo ""
    echo "Options:"
    echo "  -j N    Parallel make jobs (default: auto)"
    echo "  -c      Clean the board's output directory before building"
    echo ""
    echo "Boards:"
    for board in $(printf '%s\n' "${!BOARDS[@]}" | sort); do
        printf "  %-20s %s + configs/flavour/%s.config\n" "${board}" "${BOARDS[$board]}" "${FLAVOUR[$board]}"
    done
    echo "  (a bare board name: libre = libre-plus, plutoskyr2 = plutoskyr2-plus, pluto = pluto-basic)"
    if [ "${#MERGE_GROUPS[@]}" -gt 0 ]; then
        echo ""
        echo "Groups (build all members, merge flash images into one zip):"
        for artifact in $(printf '%s\n' "${!MERGE_GROUPS[@]}" | sort); do
            printf "  %-20s %s\n" "${artifact}" "${GROUP_MEMBERS[$artifact]}"
        done
    fi
    echo "  all                  Build all boards"
    exit 1
}

JOBS=""
CLEAN=false

while getopts "j:ch" opt; do
    case $opt in
        j) JOBS="-j${OPTARG}" ;;
        c) CLEAN=true ;;
        h) usage ;;
        *) usage ;;
    esac
done
shift $((OPTIND - 1))

[ $# -eq 0 ] && usage

# Source BR2_EXTERNAL if not already set
if [ -z "${BR2_EXTERNAL:-}" ]; then
    # shellcheck source=sourceme.first
    source "${SCRIPT_DIR}/sourceme.first"
fi

# Verify buildroot exists
if [ ! -d "${BUILDROOT_DIR}" ]; then
    echo "ERROR: buildroot/ not found. Run ./getbuildroot.sh first."
    exit 1
fi

# Expand "all" and group names to their member boards. A group name is
# checked before a literal board match since some groups are named after
# their primary member (e.g. "fishball" itself, grouped with
# fishball_mini_7010) -- requesting that name should pull in the whole
# group, not just the one board. REQUESTED_GROUPS tracks which groups were
# named explicitly, so their members get merged into one zip after building
# (see merge_group() below).
TARGETS=()
REQUESTED_GROUPS=()
for arg in "$@"; do
    if [ "$arg" = "all" ]; then
        TARGETS=("${!BOARDS[@]}")
    elif [ -n "${MERGE_GROUPS[$arg]+x}" ]; then
        # shellcheck disable=SC2206
        TARGETS+=(${GROUP_MEMBERS[$arg]})
        REQUESTED_GROUPS+=("$arg")
    elif [ -n "${BOARDS[$arg]+x}" ]; then
        TARGETS+=("$arg")
    elif [ -n "${ALIAS[$arg]+x}" ]; then
        TARGETS+=("${ALIAS[$arg]}")
    else
        echo "ERROR: Unknown board '${arg}'"
        echo ""
        usage
    fi
done

build_board() {
    local board="$1"
    local defconfig="${BOARDS[$board]}"
    local flavour="${FLAVOUR[$board]}"
    local output_dir="${SCRIPT_DIR}/output/${OUTNAME[$board]}"

    echo "=== Building ${board} (${defconfig}) ==="
    echo "    Output: ${output_dir}"

    if [ "${CLEAN}" = true ] && [ -d "${output_dir}" ]; then
        echo "    Cleaning ${output_dir}..."
        if command -v trash >/dev/null 2>&1; then
            trash "${output_dir}"
        else
            rm -rf "${output_dir}"
        fi
    fi

    make -C "${BUILDROOT_DIR}" O="${output_dir}" "${defconfig}"
    if [ -n "${flavour}" ]; then
        cat "${SCRIPT_DIR}/configs/flavour/${flavour}.config" >> "${output_dir}/.config"
        make -C "${BUILDROOT_DIR}" O="${output_dir}" olddefconfig
    fi
    # trxd is built from the tree (package/trxd/trxd.mk, local site):
    # Buildroot rebuilds such a package only when told, and once shipped
    # the daemon built hours before a change to src/trxd. Its stamps go
    # when the sources are newer than its last build.
    local trxd_build="${output_dir}/build/trxd-0.1.0"
    if [ -f "${trxd_build}/.stamp_built" ] && [ -n "$(find "${SCRIPT_DIR}/src/trxd/src" "${SCRIPT_DIR}/src/trxd/web" "${SCRIPT_DIR}/src/trxd/Cargo.toml" "${SCRIPT_DIR}/src/trxd/Cargo.lock" -newer "${trxd_build}/.stamp_built" -print -quit 2>/dev/null)" ]; then
        echo "    src/trxd changed since its last build: rebuilding trxd"
        rm -f "${trxd_build}"/.stamp_built "${trxd_build}"/.stamp_installed "${trxd_build}"/.stamp_target_installed "${trxd_build}"/.stamp_staging_installed
    fi
    # The same for the bitstreams (package/board-fpga takes them from the
    # tree): any file in bitstream/ newer than its last build (.xsa, .rate,
    # boot-mode), or one its copy has that the tree no longer does (a mode
    # dropped: the rsync left it there and it went into the rootfs).
    local fpga_build fpga_src f stale
    fpga_src="${SCRIPT_DIR}/board/tezuka/${HW[$board]:-$board}/bitstream"
    for fpga_build in "${output_dir}"/build/board-fpga*; do
        [ -f "${fpga_build}/.stamp_built" ] || continue
        stale=
        for f in "${fpga_build}"/*.xsa "${fpga_build}"/*.rate; do
            [ -e "$f" ] && [ ! -e "${fpga_src}/${f##*/}" ] && stale="${f##*/}"
        done
        if [ -n "${stale}" ] || [ -n "$(find "${fpga_src}" -type f -newer "${fpga_build}/.stamp_built" -print -quit 2>/dev/null)" ]; then
            echo "    a bitstream changed or went since its last build${stale:+ (${stale})}: rebuilding board-fpga"
            rm -f "${fpga_build}"/.stamp_built "${fpga_build}"/.stamp_installed "${fpga_build}"/.stamp_target_installed "${fpga_build}"/.stamp_staging_installed "${fpga_build}"/.stamp_extracted "${fpga_build}"/.stamp_rsynced
        fi
    done
    # And for the kernel's device trees (custom DTS files from the tree):
    # a newer .dts/.dtsi than the kernel's last build rebuilds it (only the
    # device trees change; make does the rest incrementally).
    local linux_build="${output_dir}/build/linux-custom"
    if [ -f "${linux_build}/.stamp_built" ] && [ -n "$(find "${SCRIPT_DIR}/board/tezuka/${HW[$board]:-$board}/dts" -name '*.dts*' -newer "${linux_build}/.stamp_built" -print -quit 2>/dev/null)" ]; then
        echo "    a device tree changed since the kernel's last build: rebuilding linux"
        rm -f "${linux_build}"/.stamp_built "${linux_build}"/.stamp_installed "${linux_build}"/.stamp_target_installed "${linux_build}"/.stamp_images_installed
    fi
    # shellcheck disable=SC2086
    make -C "${BUILDROOT_DIR}" O="${output_dir}" ${JOBS}

    local zip="${output_dir}/images/tezuka.zip"
    if [ -f "${zip}" ]; then
        mkdir -p "${SCRIPT_DIR}/build"
        cp "${zip}" "${SCRIPT_DIR}/build/${board}.zip"
        # the plus images also under the old name (build/libre.zip)
        [ "${OUTNAME[$board]}" = "${board}" ] || cp "${zip}" "${SCRIPT_DIR}/build/${OUTNAME[$board]}.zip"
        echo "=== ${board} complete: build/${board}.zip ==="
    else
        echo "WARNING: ${zip} not found"
    fi
}

# Merges a group's member boards' images/flash/ and images/sdimg/ outputs
# into one build/<artifact>.zip. Each member populates only one of the two
# (e.g. fishball's own build only fills sdimg/, fishball_mini_7010's only
# fills flash/ -- prepost-image.sh always mkdir's both, but only the
# postimage-sd.sh/postimage-qspi.sh scripts actually in that defconfig's
# script chain populate them), so a plain union has nothing to collide.
merge_group() {
    local artifact="$1"
    # shellcheck disable=SC2206
    local members=(${GROUP_MEMBERS[$artifact]})
    local merge_dir="${SCRIPT_DIR}/build/.merge-${artifact}"

    rm -rf "${merge_dir}"
    mkdir -p "${merge_dir}/flash" "${merge_dir}/sdimg"
    for board in "${members[@]}"; do
        local images="${SCRIPT_DIR}/output/${board}/images"
        if [ ! -d "${images}" ]; then
            echo "WARNING: ${images} not found for ${board}, skipping in merged ${artifact}.zip"
            continue
        fi
        [ -d "${images}/flash" ] && cp -r "${images}/flash/." "${merge_dir}/flash/"
        [ -d "${images}/sdimg" ] && cp -r "${images}/sdimg/." "${merge_dir}/sdimg/"
    done

    (cd "${merge_dir}" && zip -rq "${SCRIPT_DIR}/build/${artifact}.zip" flash sdimg)
    rm -rf "${merge_dir}"
    echo "=== ${artifact} complete: build/${artifact}.zip (merged: ${members[*]}) ==="
}

for board in "${TARGETS[@]}"; do
    build_board "${board}"
done

for artifact in "${REQUESTED_GROUPS[@]}"; do
    merge_group "${artifact}"
done

echo ""
echo "=== All builds complete ==="
ls -lh "${SCRIPT_DIR}/build/"*.zip 2>/dev/null || true

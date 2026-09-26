# Sourced by the post-image scripts: sets BOOTGEN to a bootgen that works.
#
# Buildroot's host-bootgen can build but fail to parse any command line
# ("Command line parsing failed"), e.g. against a newer host toolchain.
# Candidates, first that answers -help: host-bootgen, $BOOTGEN from the
# environment, the distribution's /usr/bin/bootgen (bootgen-xlnx), then any
# Vivado/Vitis install under ~/Xilinx or /opt/Xilinx (newest first).
find_bootgen() {
    for b in "$HOST_DIR/bin/bootgen" "${BOOTGEN_OVERRIDE:-}" /usr/bin/bootgen \
             $(ls -d "$HOME"/Xilinx/*/*/bin/bootgen /opt/Xilinx/*/*/bin/bootgen 2>/dev/null | sort -r); do
        [ -n "$b" ] && [ -x "$b" ] || continue
        if "$b" -help >/dev/null 2>&1; then
            [ "$b" != "$HOST_DIR/bin/bootgen" ] && echo "NOTE: using bootgen $b (host-bootgen unusable)"
            BOOTGEN="$b"
            return 0
        fi
    done
    echo "ERROR: no working bootgen. Install bootgen-xlnx (apt) or Vivado, or set BOOTGEN_OVERRIDE." >&2
    exit 1
}
find_bootgen

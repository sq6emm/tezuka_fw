#!/bin/sh
# Print the A/B boot scripts from uboot-env.txt as `fw_setenv -s` input
# ("name value", one per line, continuation lines joined). fw-update installs
# them into the board's flash environment, so a board whose U-Boot and saved
# environment predate A/B slots still boots them. The console settings go
# with them: a saved environment from another firmware may name a UART the
# board does not have (LibreSDR stock: UART1), leaving U-Boot silent. And
# preboot_main, whose stock version imported stray RAM as environment after a
# warm reboot (and ab_count then saved the result: a board that no longer boots). Backslash escapes are
# resolved as mkenvimage does (`\"` -> `"`), since fw_setenv stores values verbatim.
#   uboot-ab-env.sh [uboot-env.txt]
ENV="${1:-$(dirname "$0")/uboot-env.txt}"
awk -v want=" ab_count slot_select sdboot_ram qspi_slot_select qspi_try qspiboot wdt_start stdin stdout stderr dfu_sf preboot_main " '
	function flush() { if (name != "" && index(want, " " name " ")) print name " " val; name = "" }
	cont { l = $0; sub(/^[ \t]+/, "", l); c = sub(/\\$/, "", l); val = val " " l; cont = c; if (!cont) flush(); next }
	/^[A-Za-z_][A-Za-z0-9_]*=/ {
		flush(); name = substr($0, 1, index($0, "=") - 1); val = substr($0, index($0, "=") + 1)
		cont = sub(/\\$/, "", val); if (!cont) flush(); next
	}
	END { flush() }
' "$ENV" | sed 's/\\\(.\)/\1/g'

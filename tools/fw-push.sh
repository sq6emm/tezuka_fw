#!/bin/sh
#
# Push a firmware build to a board over SSH and install it with fw-update
# (A/B slots, automatic rollback; see board/tezuka/common/overlay_base/usr/sbin/fw-update).
#
#   tools/fw-push.sh [-p password] [--bootloader] [--migrate] <board-host> <build/plutoskyr2.zip>
#
# --migrate: the board still runs a pre-A/B firmware (no fw-update on it).
#   Copies fw-update over first; the old files in / stay as the fallback.
# -p: password for sshpass (default: key authentication).

set -e
PASS=""
OPTS=""
MIGRATE=0
while [ $# -gt 0 ]; do
	case "$1" in
	-p) PASS="$2"; shift ;;
	--bootloader) OPTS="$OPTS --bootloader" ;;
	--migrate) MIGRATE=1 ;;
	*) break ;;
	esac
	shift
done
HOST="$1"
ZIP="$2"
[ -n "$HOST" ] && [ -f "$ZIP" ] || { sed -n '3,12p' "$0"; exit 2; }

SSH="ssh -o StrictHostKeyChecking=accept-new root@$HOST"
SCP="scp -O -o StrictHostKeyChecking=accept-new"
if [ -n "$PASS" ]; then
	SSH="sshpass -p $PASS $SSH"
	SCP="sshpass -p $PASS $SCP"
fi

if [ "$MIGRATE" = 1 ]; then
	HERE="$(cd "$(dirname "$0")/.." && pwd)"
	$SCP "$HERE/board/tezuka/common/overlay_base/usr/sbin/fw-update" "root@$HOST:/tmp/fw-update"
	FWU="sh /tmp/fw-update"
else
	FWU="/usr/sbin/fw-update"
fi

echo "== $HOST: current state"
$SSH "$FWU --status" || true
echo "== copying $(basename "$ZIP") ($(du -h "$ZIP" | cut -f1))"
$SCP "$ZIP" "root@$HOST:/tmp/fw.zip"
echo "== installing"
# In the foreground: dropbear kills a backgrounded job when the session
# closes, which used to abort the install silently.
$SSH "$FWU $OPTS --no-reboot /tmp/fw.zip; rc=\$?; rm -f /tmp/fw.zip; exit \$rc"
$SSH reboot || true
echo "== rebooting; waiting for the board to come back"
sleep 20
for i in $(seq 1 60); do
	if $SSH -o ConnectTimeout=3 true 2>/dev/null; then
		$SSH "/usr/sbin/fw-update --status"
		echo "== the new slot confirms itself once network and trxd are up (S99fwconfirm, <= 4 min)"
		exit 0
	fi
	sleep 5
done
echo "== $HOST did not come back within 5 min; after 3 unconfirmed boots U-Boot rolls back by itself" >&2
exit 1

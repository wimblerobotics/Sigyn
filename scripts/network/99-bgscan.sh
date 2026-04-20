#!/bin/sh
set -eu

PATH=/usr/sbin:/usr/bin:/sbin:/bin
IFACE="${1:-}"
ACTION="${2:-}"

case "$ACTION" in
  up|dhcp4-change|reapply|connectivity-change)
    ;;
  *)
    exit 0
    ;;
esac

[ -n "$IFACE" ] || exit 0

(
  sleep 5
  network_id=$(wpa_cli -i "$IFACE" status 2>/dev/null | awk -F= '/^id=/{print $2; exit}')
  [ -n "$network_id" ] || exit 0

  wpa_cli -i "$IFACE" set_network "$network_id" bgscan '"simple:30:-65:8"' >/dev/null 2>&1 || exit 0
  wpa_cli -i "$IFACE" reassociate >/dev/null 2>&1 || true
  logger -t nm-bgscan "Applied bgscan simple:30:-65:8 to $IFACE network $network_id after $ACTION"
) &

exit 0

#!/usr/bin/env bash
# laptop_hotspot.sh — isolated robot hotspot on a virtual AP interface beside the campus link.
#
# Why not NetworkManager: NM/wpa_supplicant re-type the virtual interface to station, and the
# AX210 allows only one station (#{managed} <= 1) -> EBUSY. Measured session 10, 30 Sep 2026.
#
# Isolation: no NAT, no default gateway and no DNS are advertised over DHCP, and ip_forward is
# forced to 0 on `up`. Hotspot clients can reach the laptop (and each other — the phone app needs
# that to reach the Go2), nothing else. Hard rule 5b: never IP-forward the robot segments.
#
# The AX210 allows AP + station on ONE channel only (#channels <= 1): `up` refuses to start if the
# campus link and $CONF disagree on channel.
#
# Usage: sudo scripts/laptop_hotspot.sh init|up|down|status
set -euo pipefail

PHY_IF=wlp48s0
AP_IF=wlhs0
AP_MAC=2a:d0:ea:3f:21:1a            # locally administered
AP_ADDR=192.168.50.1/24
DHCP_RANGE=192.168.50.10,192.168.50.50,255.255.255.0,12h
SSID=dogspider
CHANNEL=11
CONF=/etc/hostapd/${SSID}.conf       # root-only, holds the passphrase — never in the repo
RUN=/run/${SSID}

[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }

init() {
    # Passphrase is read from the stdin prompt, never from argv or the environment.
    read -rs -p "Hotspot passphrase (8-63 chars): " psk; echo
    [[ ${#psk} -ge 8 && ${#psk} -le 63 ]] || { echo "passphrase length invalid" >&2; exit 1; }
    mkdir -p "$(dirname "$CONF")"
    install -m 600 /dev/null "$CONF"
    cat > "$CONF" <<EOF
interface=$AP_IF
driver=nl80211
ctrl_interface=/run/hostapd
ssid=$SSID
country_code=US
hw_mode=g
channel=$CHANNEL
ieee80211n=1
wmm_enabled=1
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=$psk
EOF
    unset psk
    echo "wrote $CONF (mode $(stat -c %a "$CONF"))"
}

up() {
    [[ -f $CONF ]] || { echo "missing $CONF — run init first" >&2; exit 1; }
    local ch
    ch=$(iw dev "$PHY_IF" info | awk '/channel/{print $2; exit}')
    if [[ "$ch" != "$CHANNEL" ]]; then
        echo "ABORT: $PHY_IF is on channel ${ch:-none}, hotspot wants $CHANNEL (#channels <= 1)" >&2
        exit 1
    fi
    ip link show "$AP_IF" &>/dev/null || iw dev "$PHY_IF" interface add "$AP_IF" type __ap
    nmcli device set "$AP_IF" managed no
    ip link set "$AP_IF" down
    ip link set "$AP_IF" address "$AP_MAC"
    iw dev "$AP_IF" set type __ap
    ip addr flush dev "$AP_IF"
    ip addr add "$AP_ADDR" dev "$AP_IF"
    mkdir -p "$RUN"
    hostapd -B -P "$RUN/hostapd.pid" -f "$RUN/hostapd.log" "$CONF"
    # --port=0: no DNS. Bare --dhcp-option=3/6: advertise no router and no DNS server.
    dnsmasq --conf-file=/dev/null --interface="$AP_IF" --bind-interfaces --except-interface=lo \
        --port=0 --dhcp-range="$DHCP_RANGE" --dhcp-option=3 --dhcp-option=6 \
        --dhcp-authoritative --log-dhcp --log-facility="$RUN/dnsmasq.log" \
        --pid-file="$RUN/dnsmasq.pid" --dhcp-leasefile="$RUN/leases"
    sysctl -w net.ipv4.ip_forward=0
    sleep 2
    status
}

down() {
    for d in dnsmasq hostapd; do
        [[ -f $RUN/$d.pid ]] && kill "$(cat "$RUN/$d.pid")" 2>/dev/null || true
        rm -f "$RUN/$d.pid"
    done
    sleep 1
    ip link show "$AP_IF" &>/dev/null && iw dev "$AP_IF" del || true
    echo "hotspot down"
}

status() {
    date -u
    echo "## campus link"; iw dev "$PHY_IF" link | grep -E 'Connected|SSID|freq|signal' || true
    echo "## hotspot iface"; iw dev "$AP_IF" info 2>&1 | grep -E 'Interface|addr|type|channel|ssid' || true
    ip -br addr show "$AP_IF" 2>&1 || true
    echo "## stations"; iw dev "$AP_IF" station dump 2>/dev/null | grep -E '^Station|signal:' || echo "(none)"
    echo "## leases"; cat "$RUN/leases" 2>/dev/null || echo "(none)"
    echo "## forwarding"; sysctl net.ipv4.ip_forward net.ipv4.conf.all.forwarding
    echo "## hostapd log tail"; tail -5 "$RUN/hostapd.log" 2>/dev/null || true
}

case "${1:-}" in
    init|up|down|status) "$1" ;;
    *) echo "usage: sudo $0 init|up|down|status" >&2; exit 2 ;;
esac

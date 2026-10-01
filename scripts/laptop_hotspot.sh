#!/usr/bin/env bash
# laptop_hotspot.sh — isolated robot hotspot on a virtual AP interface beside the campus link.
#
# Why not NetworkManager: NM/wpa_supplicant re-type the virtual interface to station, and the
# AX210 allows only one station (#{managed} <= 1) -> EBUSY. Measured session 10, 30 Sep 2026.
#
# Isolation (default): no NAT, no default gateway and no DNS are advertised over DHCP, and
# ip_forward is forced to 0 on `up`. Hotspot clients can reach the laptop (and each other — the
# phone app needs that to reach the Go2), nothing else. Hard rule 5b.
#
# share-on / share-off: TEMPORARY internet for hotspot clients (e.g. a firmware download), NAT out
# the campus link only. The robot's wired segment stays explicitly blocked in both directions.
# Clients must re-join (renew DHCP) to pick up or drop the gateway. Always finish with share-off.
#
# The AX210 allows AP + station on ONE channel only (#channels <= 1): `up` refuses to start if the
# station link and the hotspot disagree on channel. `up 9` overrides the default 11, e.g. when the
# station is on the hexapod AP (2.4 GHz ch 9) instead of campus; it rewrites $CONF's channel line.
#
# Usage: sudo scripts/laptop_hotspot.sh init|up [channel]|down|status|share-on|share-off
set -euo pipefail

PHY_IF=wlp48s0                       # campus uplink
AP_IF=wlhs0
WIRED_IF=enp46s0                     # Go2 cable — never forwarded to/from the hotspot
AP_MAC=2a:d0:ea:3f:21:1a             # locally administered
AP_IP=192.168.50.1
AP_NET=192.168.50.0/24
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

# $1 = isolated | shared
start_dhcp() {
    stop_pid dnsmasq
    local dns_opts
    if [[ $1 == shared ]]; then
        # Forward client DNS to the campus resolvers, listening on the hotspot address only.
        local up
        up=$(resolvectl dns "$PHY_IF" | awk -F': ' '{print $2}')
        [[ -n $up ]] || { echo "no upstream DNS on $PHY_IF" >&2; exit 1; }
        dns_opts=(--listen-address="$AP_IP" --no-resolv)
        for s in $up; do dns_opts+=(--server="$s"); done
        dns_opts+=(--dhcp-option=3,"$AP_IP" --dhcp-option=6,"$AP_IP")
    else
        # --port=0: no DNS. Bare --dhcp-option=3/6: advertise no router and no DNS server.
        dns_opts=(--port=0 --dhcp-option=3 --dhcp-option=6)
    fi
    dnsmasq --conf-file=/dev/null --interface="$AP_IF" --bind-interfaces --except-interface=lo \
        --dhcp-range="$DHCP_RANGE" --dhcp-authoritative "${dns_opts[@]}" \
        --log-dhcp --log-facility="$RUN/dnsmasq.log" \
        --pid-file="$RUN/dnsmasq.pid" --dhcp-leasefile="$RUN/leases"
}

stop_pid() {
    [[ -f $RUN/$1.pid ]] && kill "$(cat "$RUN/$1.pid")" 2>/dev/null || true
    rm -f "$RUN/$1.pid"
    sleep 0.5
}

up() {
    [[ -f $CONF ]] || { echo "missing $CONF — run init first" >&2; exit 1; }
    if [[ -n ${1:-} ]]; then
        [[ $1 =~ ^([1-9]|1[0-3])$ ]] || { echo "channel must be 2.4 GHz 1-13" >&2; exit 1; }
        CHANNEL=$1
    fi
    sed -i -E "s/^channel=.*/channel=$CHANNEL/" "$CONF"
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
    ip addr add "$AP_IP/24" dev "$AP_IF"
    mkdir -p "$RUN"
    hostapd -B -P "$RUN/hostapd.pid" -f "$RUN/hostapd.log" "$CONF"
    start_dhcp isolated
    sysctl -w net.ipv4.ip_forward=0
    sleep 2
    status
}

down() {
    share_rules -D 2>/dev/null || true
    stop_pid dnsmasq
    stop_pid hostapd
    ip link show "$AP_IF" &>/dev/null && iw dev "$AP_IF" del || true
    sysctl -w net.ipv4.ip_forward=0
    echo "hotspot down"
}

# $1 = -I (insert) | -D (delete). Docker's FORWARD policy is DROP and user rules belong in
# DOCKER-USER, which Docker jumps to first; fall back to FORWARD when Docker is absent.
share_rules() {
    local op=$1 ch=FORWARD
    iptables -nL DOCKER-USER &>/dev/null && ch=DOCKER-USER
    local c=(-m comment --comment dogspider-share)
    # Inserted in reverse so the wired-segment DROPs end up first in the chain.
    iptables "$op" "$ch" $([[ $op == -I ]] && echo 1) -o "$AP_IF" -i "$PHY_IF" \
        -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT "${c[@]}"
    iptables "$op" "$ch" $([[ $op == -I ]] && echo 1) -i "$AP_IF" -o "$PHY_IF" -s "$AP_NET" -j ACCEPT "${c[@]}"
    iptables "$op" "$ch" $([[ $op == -I ]] && echo 1) -i "$WIRED_IF" -o "$AP_IF" -j DROP "${c[@]}"
    iptables "$op" "$ch" $([[ $op == -I ]] && echo 1) -i "$AP_IF" -o "$WIRED_IF" -j DROP "${c[@]}"
    if [[ $op == -I ]]; then op=-A; fi
    iptables -t nat "$op" POSTROUTING -s "$AP_NET" -o "$PHY_IF" -j MASQUERADE "${c[@]}"
}

share_on() {
    [[ -f $RUN/hostapd.pid ]] || { echo "hotspot is not up" >&2; exit 1; }
    share_rules -D 2>/dev/null || true      # idempotent
    share_rules -I
    start_dhcp shared
    sysctl -w net.ipv4.ip_forward=1
    echo "⚠️  SHARING ON — hotspot clients have internet via $PHY_IF. Re-join clients to get a gateway."
    echo "⚠️  Run share-off as soon as the download is done."
    status
}

share_off() {
    share_rules -D 2>/dev/null || true
    start_dhcp isolated
    sysctl -w net.ipv4.ip_forward=0
    echo "sharing off — re-join clients to drop the gateway from their lease"
    status
}

status() {
    date -u
    echo "## campus link"; iw dev "$PHY_IF" link | grep -E 'Connected|SSID|freq|signal' || true
    echo "## hotspot iface"; iw dev "$AP_IF" info 2>&1 | grep -E 'Interface|addr|type|channel|ssid' || true
    ip -br addr show "$AP_IF" 2>&1 || true
    echo "## stations"; iw dev "$AP_IF" station dump 2>/dev/null | grep -E '^Station|signal:' || echo "(none)"
    echo "## leases"; cat "$RUN/leases" 2>/dev/null || echo "(none)"
    echo "## forwarding"; sysctl net.ipv4.ip_forward net.ipv4.conf.all.forwarding
    echo "## sharing rules"
    { iptables -S; iptables -t nat -S; } 2>/dev/null | grep dogspider-share || echo "(none — isolated)"
    echo "## hostapd log tail"; tail -5 "$RUN/hostapd.log" 2>/dev/null || true
}

case "${1:-}" in
    init|down|status) "$1" ;;
    up) up "${2:-}" ;;
    share-on) share_on ;;
    share-off) share_off ;;
    *) echo "usage: sudo $0 init|up [channel]|down|status|share-on|share-off" >&2; exit 2 ;;
esac

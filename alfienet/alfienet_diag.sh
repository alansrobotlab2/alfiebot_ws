#!/usr/bin/env bash
# Run on the robot while alfienet is up, bringup is running and the headset is
# connected (ideally right after a failed page load). Writes ~/alfienet_diag_<time>.log
# so it can be read after switching back to the home network.
IF=wlP1p1s0
GW=192.168.5.1
LOG=~/alfienet_diag_$(date +%Y%m%d_%H%M%S).log
exec > >(tee "$LOG") 2>&1

echo "== interface";   ip -br addr show $IF; iw dev $IF info
echo "== routes";      ip route
echo "== listeners (8443 web, 8442 ws, 8082 nginx/foxglove, 8765 bridge)"
ss -ltn | grep -E ':(8443|8442|8082|8765) ' || echo "NONE - is bringup running?"
echo "== iptables INPUT"; sudo iptables -S INPUT
echo "== local fetch of the VR page via $GW"
curl -sk -o /dev/null -w 'http=%{http_code} connect=%{time_connect}s tls=%{time_appconnect}s total=%{time_total}s\n' \
     --max-time 5 https://$GW:8443/ || echo "local fetch FAILED"
echo "== associated stations"; iw dev $IF station dump | grep -E 'Station|signal:|tx bitrate|rx bitrate|tx retries|tx failed'
echo "== clients"
for ip in $(ip neigh show dev $IF | awk '/REACHABLE|STALE|DELAY/{print $1}'); do
    echo "-- $ip small ping";       ping -c 5 -W 1 "$ip" | tail -2
    echo "-- $ip full-MTU ping (DF)"; ping -c 5 -W 1 -M do -s 1472 "$ip" | tail -2
done
echo "== log: $LOG"

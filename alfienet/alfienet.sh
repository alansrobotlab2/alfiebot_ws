sudo systemctl stop NetworkManager

sudo nmcli device disconnect wlP1p1s0

sudo nmcli device set wlP1p1s0 managed no

sudo ./lnxrouter \
     -n \
     --ap wlP1p1s0 alfienet \
     -p alfienet \
     --freq-band 2.4 \
     -c 1 \
     --country US \
     --ieee80211n \
     --ht-capab '[HT40+][SHORT-GI-20][SHORT-GI-40]' \
     -g 192.168.5.1

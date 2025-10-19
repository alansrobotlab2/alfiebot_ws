sudo nmcli device set wlan0 managed no

sudo ./lnxrouter \
     -n -g 5 \
     --ap wlan0 alfienet \
     -p alfienet \
     -c 6 \
     --country US

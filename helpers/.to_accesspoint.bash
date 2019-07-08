#!/bin/bash

if [ $# -lt 2 ]
then
  echo "Please provide ssid and psk in the argument list of the command"
  exit 1
fi

cat << EOF | sudo tee -a /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.11.1/24

EOF

cat << EOF | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
    ssid="$1"
    psk="$2"
    mode=2
    proto=RSN
    key_mgmt=WPA-PSK
    pairwise=CCMP
    group=CCMP
    auth_alg=OPEN
}

EOF

sudo systemctl enable dnsmasq
sudo systemctl start dnsmasq

sudo systemctl restart dhcpcd

sudo reboot


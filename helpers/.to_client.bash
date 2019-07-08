#!/bin/bash
if [ $# -lt 2 ]
then
	echo "Please provide the ssid then psk in the argument list of the command"
	exit 1
fi

sudo systemctl stop dnsmasq
sudo systemctl disable dnsmasq
sudo sed -i 's/interface wlan0//' /etc/dhcpcd.conf
sudo sed -i 's/static ip_address=192.168.11.1\/24//' /etc/dhcpcd.conf
cat << EOF | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
    ssid="$1"
    psk="$2"
}

EOF
sudo systemctl restart dhcpcd

sudo reboot

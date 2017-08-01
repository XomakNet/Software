install wireless-tools wpasupplicant
sudo wpa_passphrase DuckieTown >> /etc/wpa_supplicant.conf
sudo ifdown wlan0
sudo ifup -v wlan0
sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant.conf
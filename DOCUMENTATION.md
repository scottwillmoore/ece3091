# Documentation

## Operating System

Ubuntu 20.04 LTS has been installed on the Raspberry Pi.

## Account

You may log in to the Raspberry Pi with:

```
User: group32
Password: raspberry32
```

## Network

The current network configuration has been setup to make it easier to connect to the Raspberry Pi. The on-board WiFi adaptor has been setup as an access point. The USB WiFi adaptor has been setup to automatically connect to a list of predefined networks.

You may connect to the access point with:

```
Name: Raspberry 32
Password: raspberry32
```

Once connected you may SSH into the Raspberry Pi with:

```sh
ssh group32@10.0.0.0
```

The access point will provide internet to your device provided that the second WiFi adaptor is connected to a WiFi network with internet.

Once SSHed into the Raspberry Pi you may add your WiFi network with:

```sh
sudo nano /etc/netplan/50-cloud-init.yaml
```

You must add your WiFi network name and password to the file and save it. You must make sure this format is followed:

```yaml
network:
    version: 2
    renderer: NetworkManager
    ethernets:
        eth0:
            optional: true
            dhcp4: true
    wifis:
        wlan0:
            optional: true
            dhcp4: true
            access-points:
                # Add your network SSID and password
                "SSID A":
                    password: "Password A"
                "SSID B":
                    password: "Password B"
                "SSID C":
                    password: "Password C"
        wlan1:
            optional: true
            dhcp4: true
            addresses:
                - 10.0.0.0/24
            access-points:
                "Raspberry 32":
                    password: raspberry32
```

Once you have saved and exited the file you may reload the network configuration with:

```sh
sudo netplan apply
```

### Notes

The device names `wlan0` and `wlan1` are assigned on boot and should remain stable, provided no additional wireless adaptors are plugged in. Otherwise, we can investigate setting devices names manually with their MAC addresses.

https://wiki.debian.org/NetworkInterfaceNames
https://www.freedesktop.org/software/systemd/man/systemd.link.html

To get this to work, `systemd-networkd` has been replaced with `NetworkManager` as it provides the ability to configure and setup access points using `netplan`.

https://www.configserverfirewall.com/ubuntu-linux/ubuntu-network-manager/
https://netplan.io/reference

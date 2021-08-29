# ECE3091

## Overview

This repository contains our code and documentation for [engineering design](https://handbook.monash.edu/2021/units/ECE3091). The aim is to design and build an autonomous robot that can identify and collect ball bearings of a known size and appearance. Please see the [project description](documents/project_description.pdf) for more information.

## Setup

The following tools are _highly_ recommended for team members on the project.

1. Install a text editor.

[Visual Studio Code](https://code.visualstudio.com/) is a good editor with some great extensions such as the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension to edit files over SSH, the [Live Share](https://marketplace.visualstudio.com/items?itemName=MS-vsliveshare.vsliveshare-pack) extension to edit code collaboratively in real time and the [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) and [Jupyter](https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter) extension to view and edit Python notebooks from within the editor.

2. Install the Conda package management system.

[Miniforge](https://github.com/conda-forge/miniforge) is a minimal distribution of [Python](https://www.python.org/) with [Conda](https://docs.conda.io/en/latest/) which uses [Conda Forge](https://conda-forge.org/) packages. This is used to provision a Python environment with the correct versions of dependencies that is isolated and reproducible across machines. Basically, it will make sure we are all testing and deploying the code with the exact same packages.

3. Clone this repository.

```sh
# Clone with HTTPS (easier, but less secure).
git clone https://github.com/scottwillmoore/ece3091
# Or... Clone with SSH (harder, but more secure).
git clone git@github.com:scottwillmoore/ece3091

# Then... Enter the repository.
cd ece3091

# Finally... You can open Visual Studio Code.
code .
```

4. Create a Conda environment.

```sh
conda env create --force -f environment.yml
```

5. Activate the Conda environment.

```sh
conda activate ece3091
```

TODO: Explain each step in a bit more detail.

## Robot

### Operating System

The robot has the 64-bit [Ubuntu Server 21.04](https://ubuntu.com/download/raspberry-pi) installed. You can log in with the username `robot` and password `group32`. The hostname of the device is `pi`. I have installed the [Starship prompt](https://starship.rs/) which makes it a little bit easier to understand what is going on.

### Wi-Fi

It has been configured to automatically connect to a predefined list of Wi-Fi access points. At some point in the future, I may set it up as a Wi-Fi hotspot again so that you can connect to the Pi directly.

TODO: Create a tutorial of how to add your Wi-Fi credentials to the Raspberry Pi.

### IP Address

In addition, I have installed `avahi-daemon` an implementation of [mDNS](https://en.wikipedia.org/wiki/Multicast_DNS). As mDNS is now supported on MacOS and the [latest versions of Windows](https://stackoverflow.com/questions/23624525/standard-mdns-service-on-windows/41019456#41019456) you should be able to use the `pi.local` domain name on your local network in place of the IP address.

```sh
# SSH with an IP address.
# It can be difficult to get the IP address of the Raspberry Pi.
# The IP address below is just for demonstration.
ssh robot@192.168.0.50

# Or... SSH with the local domain name.
# No IP address required.
ssh robot@pi.local
```

TODO: Create a tutorial on how to find the IP address of the Raspberry Pi.

### GPIO

This [great article](https://waldorf.waveform.org.uk/2021/the-pins-they-are-a-changin.html) describes how GPIO access has evolved in the recent years. It clearly explains the drawbacks of the old approach and introduces [`lgpio`](http://abyz.me.uk/lg/index.html) which is the best way to manage GPIO resources. We still can use [`gpiozero`](https://gpiozero.readthedocs.io/en/stable/) which provides a nice higher-level abstraction over devices connected to the GPIO pins, whereby the `lgpio` library is used [as a backend](https://gpiozero.readthedocs.io/en/stable/api_pins.html#changing-the-pin-factory). It is sometimes useful to understand how these libraries work under the hood so I have provided links to the GitHub repository of [`gpiozero`](https://github.com/gpiozero/gpiozero) and [`lgpio`](https://github.com/joan2937/lg).

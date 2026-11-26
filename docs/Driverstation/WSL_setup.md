# WSL2 ROS2 Setup

### Install Ubuntu 24.04
* Search Ubuntu 24.04 in Microsoft Store and install

### Install ROS2 Jazzy
* https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### Configure WSL2 Networking for ROS2
* Open WSL Settings 
* Set networking mode to "Mirrored"
* Add firewall rule via Admin Powershell:
```powershell
New-NetFirewallHyperVRule `
   -Name "ROS2-DDS-Inbound" `
   -DisplayName "ROS 2 DDS from Raspberry Pi" `
   -Direction Inbound `
   -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
   -Protocol UDP `
   -LocalPorts 7400-7500 `
   -RemoteAddresses 192.168.0.42 `
   -Action Allow
```

### Flight sticks
* Passthrough to WSL via usbipd
    * https://learn.microsoft.com/en-us/windows/wsl/connect-usb
* Make sure device exists: `lsusb` and `ls /dev/input`
* Add yourself to the input group: `sudo usermod -aG input $USER`
* Restart WSL
* See if ros2 recognizes joysticks: `ros2 run joy joy_enumerate_devices`
* Test
    * In one terminal: `ros2 run joy joy_node`
    * In another terminal: `ros2 topic echo /joy`

### Xbox controllers
* Update WSL
    * Powershell: `wsl --update`
* Add yourself to the input group
    * Bash: `sudo usermod -aG input $USER`
* Passthrough Xbox controller with
    * https://github.com/Yi-Cheng-Wang/com2tty
    1. Install python on Windows
    2. Powershell: `pip install com2tty`
    3. Bash: `sudo modprobe uinput`
    4. Bash: `sudo chmod 0666 /dev/uinput`
    5. Bash: `sudo usermod -aG input "$USER"`
    6. Powershell: `com2tty --gamepad --uinput`
    * To make uinput module and permissions persist across WSL restarts:
        1. Check that `systemd=true` exists in `/etc/wsl.conf` under `[boot]`
        2. Create a systemd service:
            * Create the service file: `sudo nano /etc/systemd/system/uinput-setup.service`
                * Paste:
                    ```
                    [Unit]
                    Description=Initialize uinput for virtual gamepads
                    After=systemd-modules-load.service
                    Before=multi-user.target

                    [Service]
                    Type=oneshot
                    ExecStart=/bin/bash -c 'modprobe uinput && chmod 0666 /dev/uinput'
                    RemainAfterExit=yes

                    [Install]
                    WantedBy=multi-user.target
                    ```
            * Enable the service: 
                * `sudo systemctl daemon-reload`
                * `sudo systemctl enable uinput-setup.service`
                * `sudo systemctl start uinput-setup.service`
        3. Check its status: `sudo systemctl status uinput-setup.service`
* Test
    * In one terminal: `ros2 run joy joy_node`
    * In another terminal: `ros2 topic echo /joy`

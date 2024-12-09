# Linuxcnc lathe controller based on OrangePI Plus 2E

#Disclaimer:
This is my personal hobby project. You can use it or any of its parts at your own risk. I'm not responsible for any damage to your equipment or your injury.

#Important!
Use only with a good active coolant system


Download and flash the Armbian_21.02.3 Buster 5.10.21 (not later) image and run the Orange PI.
https://drive.google.com/file/d/1FwBf0fV3swwGzbODMnL5-IoIjtYwaIQ2/view?usp=sharing

# Installation

```
cd ~/
git clone git@github.com:jmenz/orange-pi-lathe.git ~/linuxcnc/configs/orange-pi-lathe #if not done yet
git clone https://github.com/allwincnc/installer ~/installer
```

### simulate encoder index patch
```
cp ~/linuxcnc/configs/orange-pi-lathe/additional_files/simulate_index_v1.patch ~/installer/linuxcnc/drv/h3/simulate_index_v1.patch
cd ~/installer/linuxcnc/drv/h3/
patch < simulate_index_v1.patch
```
keep in mind that it's quite slow and misses position on speeds over 10kHz

### install LCNC
```
cd ~/installer
./install.sh
```

### reboot the system


### Install python-gtksourceview2 to be able to run touchy interface
```
cd ~/linuxcnc/configs/orange-pi-lathe/additional_files
sudo dpkg -i python-gtksourceview2_2.10.1-3_armhf.deb
```

### Install custom dark theme
```
cd ~/linuxcnc/configs/orange-pi-lathe
sudo sh ./install-theme.sh
```

### Add custom display resolution (optional)
```
cp ~/linuxcnc/configs/orange-pi-lathe/additional_files/.xsessionrc ~/.xsessionrc
```
NOTE: do this only if you don't have ~/.xsessionrc file yet, Otherwise, copy the commands.

It's a prepared file for 1024x600 display. If You need some other resolution, follow this [instruction](https://askubuntu.com/questions/377937/how-do-i-set-a-custom-resolution)


### Autoload (optional)

Add the next command to autoload (Application->Settings->Session and Startup->Application Autostart):
```
/usr/bin/linuxcnc '~/linuxcnc/configs/orange-pi-lathe/config.ini'
```





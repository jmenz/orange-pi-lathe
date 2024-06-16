# Linuxcnc lathe controller based on OrangePI Plus 2E

Use only with good active collant system

# Installation
```
git clone git@github.com:jmenz/orange-pi-lathe.git #if not done yet
git clone https://github.com/allwincnc/installer
```

### simulate encoder index patch
```
cp ~/orange-pi-lathe/additional_files/simulate_index_v1.patch ~/installer/linuxcnc/drv/h3/simulate_index_v1.patch
cd ~/installer/linuxcnc/drv/h3/
patch < simulate_index_v1.patch
```

### install LCNC
```
cd ~/installer
./install.sh
```

### reboot the system


### Install python-gtksourceview2 to be able to run touchy interface
```
cd ~/orange-pi-lathe/additional_files
sudo dpkg -i python-gtksourceview2_2.10.1-3_armhf.deb
```

### Add custom display resolution
```
cp ~/orange-pi-lathe/additional_files/.xsessionrc ~/.xsessionrc
```
NOTE: do this only if you don't have ~/.xsessionrc file yet, Otherwise, copy the commands.

It's a prepared file for 1024x600 display. If You need some other resolution, follow this [instruction](https://askubuntu.com/questions/377937/how-do-i-set-a-custom-resolution)










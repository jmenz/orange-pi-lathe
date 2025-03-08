# Name of the HAL component
COMPONENT = t3d_servo

# Compiler and Flags
CC = gcc
CFLAGS = -fPIC -I/usr/include/modbus -I/usr/include/linuxcnc -DULAPI
LDFLAGS = -shared -L/usr/lib/arm-linux-gnueabihf -lmodbus -lm

# Build Targets
all: $(COMPONENT).so

$(COMPONENT).so: $(COMPONENT).c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Install the shared object to LinuxCNC modules directory
install: $(COMPONENT).so
	sudo cp $(COMPONENT).so /usr/lib/linuxcnc/modules/

# Clean up compiled files
clean:
	rm -f $(COMPONENT).so

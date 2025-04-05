# Name of the HAL component
COMPONENT = t3d_servo

#before compile install libmodbus-dev
# sudo apt install libmodbus-dev

# Compiler and Flags
CC = gcc
CFLAGS = -fPIC -I/usr/include/modbus -I/usr/include/linuxcnc -DULAPI -Os
LDFLAGS = -L/usr/lib -llinuxcnchal -L/usr/lib/arm-linux-gnueabihf -lmodbus -lpthread -lm

# Build Targets
all: $(COMPONENT)

$(COMPONENT): $(COMPONENT).o t3d_servo_modbus.o t3d_servo_hal.o
	$(CC) -o $@ $^ $(LDFLAGS) 

# Compile source files
$(COMPONENT).o: $(COMPONENT).c $(COMPONENT).h
	$(CC) $(CFLAGS) -c $(COMPONENT).c -o $(COMPONENT).o

t3d_servo_modbus.o: t3d_servo_modbus.c t3d_servo.h
	$(CC) $(CFLAGS) -c t3d_servo_modbus.c -o t3d_servo_modbus.o

t3d_servo_hal.o: t3d_servo_hal.c t3d_servo.h
	$(CC) $(CFLAGS) -c t3d_servo_hal.c -o t3d_servo_hal.o

# Install the binary to the LinuxCNC userspace path
install: $(COMPONENT)
	sudo cp $(COMPONENT) /usr/bin/
	sudo chmod +x /usr/bin/$(COMPONENT)

# Clean up compiled files
clean:
	rm -f $(COMPONENT) $(COMPONENT).o t3d_servo_modbus.o t3d_servo_hal.o

# Name of the HAL component
COMPONENT = t3d_servo

# Compiler and Flags
CC = gcc
CFLAGS = -fPIC -I/usr/include/modbus -I/usr/include/linuxcnc -DULAPI
LDFLAGS = -shared -L/usr/lib/arm-linux-gnueabihf -lmodbus -lm

# Source files
SRCS = t3d_servo.c t3d_servo_modbus.c t3d_servo_hal.c
OBJS = $(SRCS:.c=.o)

# Build Target
all: $(COMPONENT).so

# Compile each .c file into an object file
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files into the final shared library
$(COMPONENT).so: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Install the shared object to LinuxCNC modules directory
install: $(COMPONENT).so
	sudo cp $(COMPONENT).so /usr/lib/linuxcnc/modules/

# Clean up compiled files
clean:
	rm -f $(COMPONENT).so $(OBJS)

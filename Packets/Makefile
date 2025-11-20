# Define variables
CC = gcc
CFLAGS = -Wall -Wextra -g
LDFLAGS =

# Define source files and executable name
SOURCES = test.c packets.c
OBJECTS = $(SOURCES:.c=.o)
TARGET = test

# Default target
all: $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $(TARGET)

# Rule to compile .c files into .o files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	rm -f $(OBJECTS) $(TARGET)
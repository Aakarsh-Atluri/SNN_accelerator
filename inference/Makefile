CC ?= gcc
CFLAGS ?= -Wall -Wextra -O2 -pedantic -std=c99 -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE
LDFLAGS ?= 

TARGET = snn_inference
GEN_TARGET = generate_dummy_data

SRCS = main.c uart.c snn_sim.c file_io.c
OBJS = $(SRCS:.c=.o)
HEADERS = snn_protocol.h uart.h snn_sim.h file_io.h

.PHONY: all clean test sample_data dummy_data help

all: $(TARGET) $(GEN_TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

$(GEN_TARGET): generate_dummy_data.c snn_protocol.h
	$(CC) $(CFLAGS) generate_dummy_data.c -o $(GEN_TARGET) $(LDFLAGS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

dummy_data: $(GEN_TARGET)
	./$(GEN_TARGET)

sample_data: $(TARGET)
	./$(TARGET) --generate

test: $(TARGET)
	./$(TARGET) --test

clean:
	rm -f $(OBJS) $(TARGET) $(GEN_TARGET) *.bin *.txt *.pbm

help:
	@echo "Available targets:"
	@echo "  make            - Build snn_inference and generate_dummy_data"
	@echo "  make dummy_data - Generate dummy weights and synthesized images"
	@echo "  make test       - Run software simulation self-tests"
	@echo "  make clean      - Remove build artifacts and temporary files"

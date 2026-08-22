CC ?= gcc
CFLAGS ?= -Wall -Wextra -O3 -std=gnu11
INCLUDES = -I.

SRCS = main.c eth_comm.c snn_sim.c file_io.c
OBJS = $(SRCS:.c=.o)
TARGET = snn_inference_eth

all: $(TARGET) generate_dummy_data

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(OBJS) -lm

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

generate_dummy_data: generate_dummy_data.c
	$(CC) $(CFLAGS) -o $@ $< -lm

clean:
	rm -f $(OBJS) $(TARGET) generate_dummy_data

.PHONY: all clean

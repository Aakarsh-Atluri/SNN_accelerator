#include "uart.h"
#include "snn_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <poll.h>
#include <glob.h>

static speed_t get_baud_constant(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:     return B115200;
    }
}

int uart_open(const char *port_name, int baudrate) {
    if (!port_name) return -1;

    int fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    speed_t speed = get_baud_constant(baudrate);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* 8-bit chars, no parity bit, 1 stop bit (8N1), enable receiver */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;         /* No parity */
    tty.c_cflag &= ~CSTOPB;         /* 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;        /* No hardware flow control */
    tty.c_cflag |= (CLOCAL | CREAD);/* Ignore modem controls, enable reading */

    /* Raw input mode: disable canonical mode, echo, signals */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);

    /* Raw output mode: disable output post-processing */
    tty.c_oflag &= ~OPOST;

    /* Raw input line settings: disable software flow control, CR/LF translation */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    /* Non-blocking read timeout controls */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5; /* 0.5 seconds inter-character timer */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    usleep(100000); /* 100ms settling time for FTDI/UART */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void uart_close(int fd) {
    if (fd >= 0) {
        tcdrain(fd);
        close(fd);
    }
}

int uart_flush(int fd) {
    if (fd < 0) return -1;
    return tcflush(fd, TCIOFLUSH);
}

ssize_t uart_write_all(int fd, const uint8_t *data, size_t len) {
    if (fd < 0 || !data) return -1;

    size_t total_written = 0;
    while (total_written < len) {
        ssize_t written = write(fd, data + total_written, len - total_written);
        if (written < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return -1;
        }
        total_written += (size_t)written;
    }
    return (ssize_t)total_written;
}

ssize_t uart_read_timeout(int fd, uint8_t *buffer, size_t len, int timeout_ms) {
    if (fd < 0 || !buffer || len == 0) return -1;

    size_t total_read = 0;
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    int remaining_ms = timeout_ms;
    struct timeval start_time, cur_time;
    gettimeofday(&start_time, NULL);

    while (total_read < len) {
        int ret = poll(&pfd, 1, remaining_ms);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (ret == 0) {
            /* Timeout reached */
            break;
        }

        if (pfd.revents & (POLLIN | POLLPRI)) {
            ssize_t r = read(fd, buffer + total_read, len - total_read);
            if (r > 0) {
                total_read += (size_t)r;
            } else if (r < 0 && errno != EAGAIN && errno != EINTR) {
                return -1;
            }
        } else if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return -1;
        }

        /* Calculate remaining timeout */
        if (timeout_ms >= 0) {
            gettimeofday(&cur_time, NULL);
            long elapsed = (cur_time.tv_sec - start_time.tv_sec) * 1000 +
                           (cur_time.tv_usec - start_time.tv_usec) / 1000;
            remaining_ms = timeout_ms - (int)elapsed;
            if (remaining_ms <= 0) break;
        }
    }

    return (ssize_t)total_read;
}

int uart_read_byte(int fd, uint8_t *byte_out, int timeout_ms) {
    if (!byte_out) return -1;
    ssize_t r = uart_read_timeout(fd, byte_out, 1, timeout_ms);
    return (r == 1) ? 0 : -1;
}

int uart_detect_ports(char found_ports[][64], int max_ports) {
    if (!found_ports || max_ports <= 0) return 0;

    int count = 0;
    const char *patterns[] = {
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/serial/by-id/*",
        NULL
    };

    for (int p = 0; patterns[p] != NULL && count < max_ports; p++) {
        glob_t globbuf;
        if (glob(patterns[p], GLOB_NOSORT, NULL, &globbuf) == 0) {
            for (size_t i = 0; i < globbuf.gl_pathc && count < max_ports; i++) {
                snprintf(found_ports[count], 64, "%s", globbuf.gl_pathv[i]);
                count++;
            }
            globfree(&globbuf);
        }
    }

    return count;
}

int uart_send_weights(int fd, const int16_t *weights, size_t num_weights,
                      void (*progress_cb)(size_t sent, size_t total)) {
    if (fd < 0 || !weights || num_weights != SNN_NUM_NEURONS) {
        return -1;
    }

    uart_flush(fd);

    /* 1. Send Header 0xAA to initiate RCV_WEIGHTS */
    uint8_t header = SNN_CMD_WRITE_WEIGHTS;
    if (uart_write_all(fd, &header, 1) != 1) {
        return -2;
    }
    tcdrain(fd);
    usleep(1000); /* 1ms pause */

    /* 2. Format 4096 x 16-bit weights as Big-Endian bytes (8192 bytes total) */
    uint8_t payload[SNN_WEIGHT_BYTES];
    for (size_t i = 0; i < num_weights; i++) {
        payload[2 * i]     = (uint8_t)((weights[i] >> 8) & 0xFF); /* MSB */
        payload[2 * i + 1] = (uint8_t)(weights[i] & 0xFF);        /* LSB */
    }

    /* Send payload in convenient chunks of 512 bytes with progress notification */
    const size_t chunk_size = 512;
    size_t bytes_sent = 0;
    while (bytes_sent < SNN_WEIGHT_BYTES) {
        size_t to_send = chunk_size;
        if (bytes_sent + to_send > SNN_WEIGHT_BYTES) {
            to_send = SNN_WEIGHT_BYTES - bytes_sent;
        }

        if (uart_write_all(fd, payload + bytes_sent, to_send) != (ssize_t)to_send) {
            return -3;
        }
        bytes_sent += to_send;
        if (progress_cb) {
            progress_cb(bytes_sent, SNN_WEIGHT_BYTES);
        }
        tcdrain(fd);
        usleep(200); /* Small pacing between chunks */
    }

    /* 3. Await 0x01 ACK from FPGA (sent automatically once 8192 bytes written to BRAM) */
    uint8_t ack_byte = 0;
    int read_ret = uart_read_byte(fd, &ack_byte, 5000);
    if (read_ret != 0) {
        return -5; /* Timeout: FPGA did not respond with ACK */
    }

    if (ack_byte != SNN_RESP_ACK) {
        return -6; /* Invalid response byte */
    }

    return 0; /* Successfully written and verified */
}

int uart_send_spikes(int fd, const uint8_t *spike_stream, size_t total_bytes,
                     int *out_result, void (*progress_cb)(int step, int total_steps)) {
    if (fd < 0 || !spike_stream || total_bytes != SNN_TOTAL_SPIKE_BYTES) {
        return -1;
    }

    uart_flush(fd);

    /* 1. Send 0xBB to initiate spike streaming mode */
    uint8_t start_cmd = SNN_CMD_START_SPIKES;
    if (uart_write_all(fd, &start_cmd, 1) != 1) {
        return -2;
    }
    tcdrain(fd);

    /* 2. Wait for initial 0x01 ACK */
    uint8_t ack = 0;
    if (uart_read_byte(fd, &ack, 2000) != 0 || ack != SNN_RESP_ACK) {
        return -3; /* FPGA not ready or no ACK received */
    }

    /* 3. Stream 25 timesteps, each 512 bytes */
    for (int t = 0; t < SNN_TIMESTEPS; t++) {
        const uint8_t *step_data = spike_stream + (t * SNN_BYTES_PER_STEP);
        
        /* Send 512 bytes for this timestep */
        if (uart_write_all(fd, step_data, SNN_BYTES_PER_STEP) != SNN_BYTES_PER_STEP) {
            return -4;
        }
        tcdrain(fd);

        /* Wait for 0x01 ACK for this 512-byte block */
        uint8_t step_ack = 0;
        if (uart_read_byte(fd, &step_ack, 2500) != 0 || step_ack != SNN_RESP_ACK) {
            return -5; /* Step ACK failure at timestep t */
        }

        if (progress_cb) {
            progress_cb(t + 1, SNN_TIMESTEPS);
        }
    }

    /* 4. Check if an inference result byte was emitted on UART */
    uint8_t res_byte = 0;
    if (uart_read_byte(fd, &res_byte, 300) == 0) {
        if (out_result) {
            *out_result = (int)res_byte;
        }
    } else {
        if (out_result) {
            *out_result = -1; /* No UART result byte received; use LEDs or SW model */
        }
    }

    return 0; /* Stream completed successfully */
}

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <termios.h>

#define TIMEOUT 2000
#define MAX_CLIENTS 2
#define PORT 5050
#define THRESHOLD 30
#define MIN_DIST 0
#define MAX_DIST 100
#define IP_ADDR "127.0.0.1"
#define BAUD_RATE B115200
#define HARDWARE_PORT "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066DFF525650657287242939-if02"

typedef enum {
    OFFLINE,
    ONLINE
} mcu_link_t;

typedef enum {
    LOCKED,
    ARMED,
    RELEASING,
    RETRACTING,
    FAULT
} payload_state_t;

typedef enum {
    ERR_NONE = 0x0,
    ERR_UART_TX = 0x1,
    ERR_UART_RX = 0x2,
    ERR_TCP_BIND = 0x4,
    ERR_INVALID_CMD = 0x8,
    ERR_BAD_TRANSITION = 0x10,
    ERR_MCU_OFFLINE = 0x20,
    ERR_UPTIME = 0x40,
    ERR_OPEN_FD = 0x80,
    ERR_UART_INIT = 0x100,
    ERR_TCP_LISTEN = 0x200,
    ERR_TCP_ACCEPT = 0x400,
    ERR_TCP_CONNECT = 0x800,
    ERR_CLIENT_REQUEST = 0x1000,
} health_error_t;

struct termios term;
sig_atomic_t run = 1;
int health_err;
int server_fd, mcu_fd;
static unsigned int distance = MAX_DIST;
static mcu_link_t mcu_link = OFFLINE;
static payload_state_t payload_state = LOCKED;
const char* commands[] = {
    "UPTIME",
    "STATUS",
    "HELP",
    "EXIT",
    "SIM_LOCK",
    "SIM_ARM",
    "SIM_RELEASE",
    "SIM_RETRACT",
    "SIM_DISTANCE",
    "SIM_FAULT",
    "SIM_RESET_FAULT",
    "LOCK",
    "ARM",
    "RELEASE",
    "RETRACT",
    "DISTANCE"
};

void handle_signal(int sig) {
	run = 0;
}

const char* get_mcu_link () {
    switch (mcu_link) {
        case OFFLINE:
            return "OFFLINE";
        case ONLINE:
            return "ONLINE";
    }
}

const char* get_payload_state () {
    switch (payload_state) {
        case LOCKED:
            return "LOCKED";
        case ARMED:
            return "ARMED";
        case RELEASING:
            return "RELEASING";
        case RETRACTING:
            return "RETRACTING";
        case FAULT:
            return "FAULT";
     }
}

int init_uart() {
    mcu_fd = open(HARDWARE_PORT, O_RDWR);
    mcu_link = OFFLINE;
    if (mcu_fd == -1) {
        health_err |= (ERR_UART_INIT | ERR_OPEN_FD);
        return 1;
    }
    int err = ioctl(mcu_fd, TCGETS, &term);
    if (err == -1) {
        health_err |= ERR_UART_INIT;
        return 1;
    }
    term.c_cflag |= CS8;        // 8 data bits
    term.c_cflag |= CREAD;      // Enable receiver
    term.c_cflag |= CLOCAL;     // Ignore modem control lines
    cfsetispeed(&term, BAUD_RATE); // 115200
    cfsetospeed(&term, BAUD_RATE);
    err = ioctl(mcu_fd, TCSETSW, &term);
    if (err == -1) {
        health_err |= ERR_UART_INIT;
        return 1;
    }
    health_err &= ~ERR_UART_INIT;
    mcu_link = ONLINE;
    return 0;
}

int uart_tx(const char* buffer) {
    if (mcu_link == ONLINE) {
        ssize_t bytes_written = write(mcu_fd, buffer, strlen(buffer));
        if (bytes_written > 0) {
            health_err &= ~ERR_UART_TX;
            return 0;
        } else {
            health_err |= ERR_UART_TX;
        }
    }
    return 1;
}

int uart_rx(char* buffer) {
    if (mcu_link == ONLINE) {
        ssize_t bytes_read = read(mcu_fd, buffer, 127);
        if (bytes_read <= 0) {
            health_err |= ERR_UART_RX;
            strcpy(buffer, "Error reading from MCU");
            return 1;
        } else {
            health_err &= ~ERR_UART_RX;
            buffer[bytes_read] = 0;
        }
        return 0;
    } else {
        health_err |= ERR_MCU_OFFLINE;
        strcpy(buffer, "MCU offline, cannot receive");
    }
    return 1;
}

int handle_uptime(char* buffer) {
    int proc_fd = open("/proc/uptime", O_RDONLY);
	if (proc_fd == -1) {
        health_err |= ERR_OPEN_FD;
        strcpy(buffer,  "Error opening /proc/uptime\n");
		return 1;
	} else {
        health_err &= ~ERR_OPEN_FD;
        char proc_buffer[64];
        char output_buffer[96];
    	ssize_t proc_bytes_read = read(proc_fd, proc_buffer, 63);
        if (proc_bytes_read == -1) {
            health_err |= ERR_UPTIME;
            strcpy(buffer, "Error reading from /proc/uptime\n");
            close(proc_fd);
            return 1;
        } else {
            health_err &= ~ERR_UPTIME;
            proc_buffer[proc_bytes_read] = 0;
            snprintf(buffer, sizeof(output_buffer), "alive, uptime = %d\n", atoi(proc_buffer));
        	close(proc_fd);
            return 0;
        }
     }
}

int cmd_lock(char* buffer) {
    int err;
    if (payload_state == RELEASING) {
        snprintf(buffer, 128, "ERR cannot transition from RELEASING to LOCKED\n");
        err = 1;
        health_err |= ERR_BAD_TRANSITION;
    } else {
        payload_state = LOCKED;
        snprintf(buffer, 128, "OK payload state = LOCKED\n");
        err = 0;
        health_err &= ~ERR_BAD_TRANSITION;
    }
    return err;
}

int cmd_arm(char* buffer) {
    int err;
    if (payload_state == LOCKED) {
        payload_state = ARMED;
        snprintf(buffer, 128, "OK payload state = ARMED\n");
        err = 0;
        health_err &= ~ERR_BAD_TRANSITION;
    } else {
        snprintf(buffer, 128, "ERR cannot transition from %s to ARMED\n", get_payload_state());
        err = 1;
        health_err |= ERR_BAD_TRANSITION;
    }
    return err;
}

int cmd_release(char* buffer) {
    int err;
    if (distance > THRESHOLD) {
        snprintf(buffer, 128, "ERR too far above ground; distance = %d; max = %d\n", distance, THRESHOLD);
        err = 1;
        health_err |= ERR_BAD_TRANSITION;
    } else if (payload_state != ARMED) {
        snprintf(buffer, 128, "ERR cannot transition from %s to RELEASING\n", get_payload_state());
        err = 1;
        health_err |= ERR_BAD_TRANSITION;
    } else {
        payload_state = RELEASING;
        snprintf(buffer, 128, "OK payload state = RELEASING\n");
        err = 0;
        health_err &= ~ERR_BAD_TRANSITION;
    }
    return err;
}

int cmd_retract(char* buffer) {
    int err;
    if (payload_state == RELEASING) {
        payload_state = RETRACTING;
        snprintf(buffer, 128, "OK payload state = RETRACTING\n");
        err = 0;
        health_err &= ~ERR_BAD_TRANSITION;
    } else {
        health_err |= ERR_BAD_TRANSITION;
        snprintf(buffer, 128, "ERR cannot transition from %s to RETRACTING\n", get_payload_state());
        err = 1;
    }
    return err;
}

int send_mcu_command(char* command_buffer, const char* command) {
    char buffer[128] = {0};
    int err = 0;
    snprintf(buffer, 128, "%s\n", command);
    err = uart_tx(buffer);
    if (err != 0) {
        snprintf(command_buffer, 384, "%s: Error with UART TX", command);
    } else {
        err = uart_rx(command_buffer);
        if (err != 0) {
            snprintf(command_buffer, 384, "%s: Error with UART RX", command);
        }
    }
    return err;
}

int handle_client(char* socket_buffer, int buffer_size, int client_fd) {
    int err;
    char command_buffer[384];
    while (run) {
        ssize_t socket_bytes_read = read(client_fd, socket_buffer, buffer_size - 1);
        if (socket_bytes_read == -1) {
            perror("client_fd read");
            return 1;
        } else if (socket_bytes_read == 0) {
            return 0;
        }
        socket_buffer[socket_bytes_read] = 0;
        if (strncmp("UPTIME", socket_buffer, 6) == 0) {
            err = handle_uptime(command_buffer);
            if (err != 0) {
                strcpy(command_buffer, "Error retrieving uptime\n");
            }
        } else if (strncmp("STATUS", socket_buffer, 6) == 0) {
            snprintf(command_buffer, 128, "health:alive\nmcu_link=%s\npayload_state=%s\nerrors=%#X",
                    get_mcu_link(),
                    get_payload_state(),
                    health_err
            );
        } else if (strncmp("SIM_LOCK", socket_buffer, 8) == 0) {
            err = cmd_lock(command_buffer);
        } else if (strncmp("LOCK", socket_buffer, 4) == 0) {
            send_mcu_command(command_buffer, "LOCK");
        } else if (strncmp("SIM_ARM", socket_buffer, 7) == 0) {
            err = cmd_arm(command_buffer);
        } else if (strncmp("ARM", socket_buffer, 3) == 0) {
            send_mcu_command(command_buffer, "ARM");
        } else if (strncmp("SIM_RELEASE", socket_buffer, 11) == 0) {
            err = cmd_release(command_buffer);
        } else if (strncmp("RELEASE", socket_buffer, 7) == 0) {
            send_mcu_command(command_buffer, "RELEASE");
        } else if (strncmp("SIM_RETRACT", socket_buffer, 11) == 0) {
            err = cmd_retract(command_buffer);
        } else if (strncmp("RETRACT", socket_buffer, 7) == 0) {
            send_mcu_command(command_buffer, "RETRACT");
        } else if (strncmp("SIM_DISTANCE", socket_buffer, 12) == 0 && socket_buffer[14] != 0) {
            int _distance;
            if (sscanf(socket_buffer, "SIM_DISTANCE %d", &_distance) != 1) {
                snprintf(command_buffer, 128, "Invalid format. Use: SIM_DISTANCE _\n");
                err = 1;
            } else if (_distance >= MIN_DIST && _distance <= MAX_DIST) {
                distance = _distance;
                snprintf(command_buffer, 128, "OK distance %d\n", distance);
                err = 0;
            } else {
                snprintf(command_buffer, 128, "ERR distance invalid %d\n", distance);
                err = 1;
            }
        } else if (strncmp("DISTANCE", socket_buffer, 8) == 0) {
            send_mcu_command(command_buffer, "DISTANCE");
        } else if (strncmp("SIM_FAULT", socket_buffer, 9) == 0) {
            snprintf(command_buffer, 128, "OK payload state = FAULT\n");
            payload_state = FAULT;
        }  else if (strncmp("SIM_RESET_FAULT", socket_buffer, 15) == 0) {
            if (payload_state == FAULT) {
                snprintf(command_buffer, 128, "OK manual reset triggered\n");
                payload_state = LOCKED;
            } else {
                snprintf(command_buffer, 128, "ERR no fault active, current state = %s\n", get_payload_state());
            }
        } else if (strncmp("HELP", socket_buffer, 4) == 0) {
            char str1[384] = "";
            strcat(str1, "Commands:");
            for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
                strcat(str1, "\n\t");
                strcat(str1, commands[i]);
            }
            strcat(str1, "\n");
            strcpy(command_buffer, str1);
            err = 0;
        } else if (strncmp("EXIT", socket_buffer, 4) == 0) {
            strcpy(command_buffer, "Exiting session ...\n");
            write(client_fd, command_buffer, strlen(command_buffer));
            shutdown(client_fd, SHUT_RDWR);
            return 0;
        } else {
            strcpy(command_buffer, "Unknown command. Try: HELP\n");
        }
        write(client_fd, command_buffer, strlen(command_buffer));
    }
    return 0;
}

int main () {
    char socket_buffer[128];
    char err_buffer[128];
    int err;
    signal(SIGTERM, handle_signal);
	signal(SIGINT, handle_signal);
	// creating socket file descriptor
    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0) {
        health_err |= ERR_OPEN_FD;
        perror("Error with TCP socket FD creation");
        return 1;
    } else {
        health_err &= ~ERR_OPEN_FD;
    }
    err = init_uart();
    if (err != 0) {
        perror("UART initialization failed\n");
    }
    int opt = 1;
    socklen_t opt_len = sizeof(opt);
    err = setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, opt_len);
    if (err != 0) {
        perror("Error setting socket options\n");
        return 1;
    }
    // binding socket to PORT at localhost
    struct in_addr ip_addr =  {
        .s_addr = inet_addr(IP_ADDR),
    };
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr = ip_addr,
    };
    socklen_t address_len = sizeof(address);
    int bind_err = bind(server_fd, (struct sockaddr*)&address, address_len);
    if (bind_err != 0) {
        snprintf(err_buffer, 128, "Error binding TCP socket to localhost at port %d\n", PORT);
        perror(err_buffer);
        health_err |= ERR_TCP_BIND;
        return 1;
    } else {
        health_err &= ~ERR_TCP_BIND;
     }
    // listening at port 5050 on localhost
    int listen_err = listen(server_fd, MAX_CLIENTS);
    if (listen_err != 0) {
        snprintf(err_buffer, 128, "Error listening at port %d on localhost\n", PORT);
        perror(err_buffer);
        health_err |= ERR_TCP_LISTEN;
        return 1;
    } else {
        health_err &= ~ERR_TCP_LISTEN;
    }
    struct pollfd _pollfd = {
        .fd = server_fd,
        .events = POLLIN,
    };
    while (run) {
        // wait until somebody connects
        err = poll(&_pollfd, 1, TIMEOUT);
        if (err == -1) {
            health_err |= ERR_TCP_CONNECT;
            perror("Error establishing socket connection\n");
            return 1;
        } else if (_pollfd.revents & POLLIN) {
            // accepting client
            health_err &= ~ERR_TCP_CONNECT;
            socklen_t address_len = sizeof(address);
            int client_fd = accept(server_fd, (struct sockaddr*)&address, &address_len);
            if (client_fd == -1) {
                health_err |= ERR_TCP_ACCEPT;
                perror("Error accepting client\n");
                return 1;
            }
            health_err &= ~ERR_TCP_ACCEPT;
            err = handle_client(socket_buffer, 128, client_fd);
            close(client_fd);
            if (err != 0) {
                health_err |= ERR_CLIENT_REQUEST;
                perror("Error handing TCP client");
                return 1;
            }
        }
    }
    close(server_fd);
    return 0;
}

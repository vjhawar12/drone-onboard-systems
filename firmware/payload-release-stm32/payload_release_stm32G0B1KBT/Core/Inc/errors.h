#ifndef ERRORS_H
#define ERRORS_H

typedef enum app_err_t {
    NONE,
    ERR_TX,
    ERR_RX,
    ERR_ULTRASONIC
} app_err_t;

#endif
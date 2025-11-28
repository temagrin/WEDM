#include "bilog.h"
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdlib>
#include <cmath>
#include "tusb.h"


static uint16_t crc16_modbus(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }}}
    return crc;
}

void send_text_packet(const char* text) {
    uint8_t text_len = strlen(text);
    if (text_len > 240) return;
    uint8_t packet[2 + 1 + 240 + 2];
// Маркеры + размер + текст
    packet[0] = 0xBC;
    packet[1] = 0xCA;
    packet[2] = text_len;
    memcpy(packet + 3, text, text_len);

    uint16_t final_crc = crc16_modbus(packet, 3 + text_len);
    packet[3 + text_len] = final_crc & 0xFF; // CRC low
    packet[3 + text_len + 1] = final_crc >> 8; // CRC high
    tud_cdc_write(packet, 5 + text_len);
    tud_cdc_write_flush();
}


size_t print_num_to_buf(char* buf, int32_t num, char fmt) {
    char temp[12];
    char* p = temp + 11;
    *p = '\0';

    bool negative = (num < 0 && fmt == 'd');
    uint32_t uval = negative ? -num : num;

// ✅ Деление зависит от fmt, НЕ от uval!
    int base = (fmt == 'x') ? 16 : 10;

    do {
        *--p = "0123456789ABCDEF"[uval % base]; // ✅ base вместо 16/10
        uval /= base; // ✅ base вместо условия
    } while (uval);

    if (negative) *--p = '-';
    strcpy(buf, p);
    return strlen(buf);
}

size_t print_float_to_buf(char* buf, float fval, int decimals) {
// Масштабируем: 7.7 → 7700000 (6 знаков)
    float scale = powf(10.0f, decimals);
    int scaled = (int)(fval * scale + (fval >= 0 ? 0.5f : -0.5f));

    int whole = scaled / (int)scale;
    int frac = abs(scaled % (int)scale);

    char temp[32];
    char* p = temp + 31;
    *p = '\0';

// Дробная часть (6 цифр)
    for (int i = decimals; i > 0; i--) {
        *--p = '0' + (frac % 10);
        frac /= 10;
    }
    *--p = '.';

// Целая часть
    uint32_t uval = abs(whole);
    if (uval == 0) *--p = '0';
    else {
        while (uval) {
            *--p = '0' + (uval % 10);
            uval /= 10;
        }
    }

    if (whole < 0) *--p = '-';

    strcpy(buf, p);
    return strlen(buf);
}

void send_printf_packet(const char* fmt, ...) {
    char text_buf[240];
    size_t text_len = 0;

    va_list args;
    va_start(args, fmt);

    while (*fmt && text_len < 230) {
        if (*fmt == '%') {
            fmt++;
            char type = *fmt++;

            if (type == 'f') {
// Проверяем %.6f → 6 знаков
                int decimals = 6; // По умолчанию
                if (*fmt >= '0' && *fmt <= '9') {
                    decimals = *fmt++ - '0';
                    fmt++; // Пропускаем f
                }

                float fval = (float)va_arg(args, double);
                char num_buf[24];
                size_t num_len = print_float_to_buf(num_buf, fval, decimals);
                memcpy(text_buf + text_len, num_buf, num_len);
                text_len += num_len;

            } else {
                int32_t val = va_arg(args, int32_t);
                char num_buf[12];
                size_t num_len = print_num_to_buf(num_buf, val, type);
                memcpy(text_buf + text_len, num_buf, num_len);
                text_len += num_len;
            }
        } else {
            text_buf[text_len++] = *fmt++;
        }
    }
    va_end(args);
    text_buf[text_len] = '\0';

    send_text_packet(text_buf);
}

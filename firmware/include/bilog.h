#ifndef FIRMWARE_BILOG_H
#define FIRMWARE_BILOG_H
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdlib>

void send_text_packet(const char* text);
size_t print_num_to_buf(char* buf, int32_t num, char fmt);
size_t print_float_to_buf(char* buf, float fval, int decimals);
void send_printf_packet(const char* fmt, ...);


#endif // FIRMWARE_BILOG_H
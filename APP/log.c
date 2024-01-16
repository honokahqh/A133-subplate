#include "log.h"

const char *log_level_str[] = {"N", "E", "W", "I", "D"};
/**
 * log_printf
 * @brief 打印时间戳或tick值，用于调试程序
 * @author Honokahqh
 * @date 2023-12-16
 */
void log_printf(log_level_t level, const char *tag, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    // 获取当前的tick值或时间戳
    unsigned long tick = millis();

    // 打印 level tick tag fmt
    printf("%s (%lu) %s: ", log_level_str[level], tick,  tag);

    // 打印剩余的信息
    vprintf(fmt, args);
    va_end(args);
}

#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>
#include <stdio.h>

/*
 * 项目级调试日志总开关。
 *
 * 只需在 main.c 中设置 PROJECT_DEBUG 为 1 或 0；main.c 使用该宏初始化
 * g_project_debug_enabled，其他模块通过这个全局开关决定是否打印。
 */
extern const uint8_t g_project_debug_enabled;

#define LOG_PRINT(level, module, ...)                                      \
    do {                                                                   \
        if (g_project_debug_enabled != 0U) {                               \
            printf("[" level "][" module "] " __VA_ARGS__);             \
        }                                                                  \
    } while (0)

#define LOG_DEBUG(module, ...) LOG_PRINT("DEBUG", module, __VA_ARGS__)
#define LOG_INFO(module, ...)  LOG_PRINT("INFO", module, __VA_ARGS__)
#define LOG_WARN(module, ...)  LOG_PRINT("WARN", module, __VA_ARGS__)
#define LOG_ERROR(module, ...) LOG_PRINT("ERROR", module, __VA_ARGS__)

#endif

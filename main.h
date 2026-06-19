/**
 * @file    main.h
 * @brief   Top-level header, as CubeMX generates it -- pulls in HAL and
 *          this project's own config header so any .c file can just
 *          #include "main.h" and get the peripheral handle externs too.
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include "project_config.h"

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

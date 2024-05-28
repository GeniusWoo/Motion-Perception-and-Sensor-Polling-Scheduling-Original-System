/**
  ******************************************************************************
  * @file    har_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Thu Apr 11 00:35:03 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef HAR_DATA_PARAMS_H
#define HAR_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_HAR_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_har_data_weights_params[1]))
*/

#define AI_HAR_DATA_CONFIG               (NULL)


#define AI_HAR_DATA_ACTIVATIONS_SIZES \
  { 1708, }
#define AI_HAR_DATA_ACTIVATIONS_SIZE     (1708)
#define AI_HAR_DATA_ACTIVATIONS_COUNT    (1)
#define AI_HAR_DATA_ACTIVATION_1_SIZE    (1708)



#define AI_HAR_DATA_WEIGHTS_SIZES \
  { 48300, }
#define AI_HAR_DATA_WEIGHTS_SIZE         (48300)
#define AI_HAR_DATA_WEIGHTS_COUNT        (1)
#define AI_HAR_DATA_WEIGHT_1_SIZE        (48300)



#define AI_HAR_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_har_activations_table[1])

extern ai_handle g_har_activations_table[1 + 2];



#define AI_HAR_DATA_WEIGHTS_TABLE_GET() \
  (&g_har_weights_table[1])

extern ai_handle g_har_weights_table[1 + 2];


#endif    /* HAR_DATA_PARAMS_H */

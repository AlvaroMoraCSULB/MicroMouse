/*
 * wall_position.h
 *
 *  Created on: May 22, 2024
 *      Author: guineverebergdorf
 */

# pragma once

#include "main.h"
#include <stdbool.h>

void getElementsGreaterThan300Flags(uint16_t array[], int size, bool result[]) {
    for (int i = 0; i < size; i++) {
        result[i] = array[i] > 300;
    }
}

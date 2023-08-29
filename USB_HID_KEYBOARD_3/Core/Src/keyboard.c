/*
 * keyboard.c
 *
 *  Created on: Aug 29, 2023
 *      Author: DNK124
 */

#include "usbd_hid.h"
#include "usb_device.h"
#include "stm32l0xx_hal.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} subKeyBoard;

subKeyBoard keyBoardHIDsub = { 0, 0, 0, 0, 0, 0, 0, 0 };

void KeyBoardPrint(char *data, uint16_t length) {
    uint8_t reportBuffer[sizeof(subKeyBoard)] = {0}; // Create a buffer for the HID report

    for (uint16_t count = 0; count < length; count++) {
        if (data[count] >= 0x41 && data[count] <= 0x5A) {
            keyBoardHIDsub.MODIFIER = 0x02;
            keyBoardHIDsub.KEYCODE1 = data[count] - 0x3D;
        } else if (data[count] >= 0x61 && data[count] <= 0x7A) {
            keyBoardHIDsub.KEYCODE1 = data[count] - 0x5D;
        } else if (data[count] == 0x20) {
            keyBoardHIDsub.KEYCODE1 = 0x2C;
        } else if (data[count] == 0x0A) {
            keyBoardHIDsub.KEYCODE1 = 0x28;
        }

        // Populate the report buffer with the appropriate values
        memcpy(reportBuffer, &keyBoardHIDsub, sizeof(keyBoardHIDsub));

        // Send the report buffer using USBD_HID_SendReport
        USBD_HID_SendReport(&hUsbDeviceFS, reportBuffer, sizeof(reportBuffer));

        HAL_Delay(15);
        keyBoardHIDsub.MODIFIER = 0x00;
        keyBoardHIDsub.KEYCODE1 = 0x00;
        USBD_HID_SendReport(&hUsbDeviceFS, reportBuffer, sizeof(reportBuffer));

        HAL_Delay(25);
    }
}

/*
 * camera.c
 *
 *  Created on: Nov 3, 2018
 *      Author: Graham Thoms
 */

#include "camera.h"
#include "arducam.h"


#include "malloc.h"
#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"



static uint32_t captureStart;
uint8_t fifoBuffer[BURST_READ_LENGTH];

uint8_t *ptr_picture;
static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);


void camera_setup(){

	cameraReady = pdFALSE;
	/**
	 * Detect and initialize the Arduchip interface.
	 * Ensure that the OV5642 is powered on.
	 * Detect and initialize the OV5642 sensor chip.
	 */
	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			printf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			printf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		printf("camera: setup failed\n\r");
		cameraReady = pdTRUE;
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	printf("camera: initiate capture\n\r");

	if (!cameraReady) {
		printf("camera: set up camera before capture\n\r");
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		printf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			printf("camera: capture timeout\n\r");
			return;
		}
	}

	printf("camera: capture complete\n\r");

	camera_get_image();

	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		printf("camera: captured jpeg image -> %lu bytes\n\r", length);
		write_fifo_to_buffer(length);
	} else {
		printf("camera: get fifo length failed\n\r");
	}

	return;
}

BaseType_t
write_fifo_to_buffer(uint32_t length) {
	/* Write the FIFO contents to disk. */
	uint16_t chunk = 0;

	free(ptr_picture);
	// jpeg pic size
	unsigned int jpeg_size = length*sizeof(uint8_t);
	// allocate memory to store jpeg picture
	if((ptr_picture = malloc(jpeg_size)) == NULL){
		printf("camera: ran out of memory\n\r");
	}else{
		printf("camera: allocated %d bytes of memory for picture\n\r", malloc_usable_size(ptr_picture));
	}

	uint8_t* current_picture_ptr = ptr_picture;
	int current_x = 10;
	int current_y = 10;

	for (uint16_t i = 0; length > 0; ++i) {
		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);
		memcpy(current_picture_ptr, &fifoBuffer, chunk*sizeof(uint8_t));
		length -= chunk;
		if(length > 0) {
			current_picture_ptr += chunk*sizeof(uint8_t);
		}

		BSP_LCD_DrawPixel(10, 10, *current_picture_ptr);
		current_x++;
		current_y++;
	}

	printf("Printing Image\r\n");




	// test image

	BSP_LCD_DrawBitmap(20, 100, (uint8_t *)stlogo);

    osDelay(500);

	return pdTRUE;
}



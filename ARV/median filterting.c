#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "qdbmp.h"
#include "qdbmp.c"

void medianFilter(uint16_t width, uint16_t height, uint8_t* originaldataPtr, uint8_t* processed_dataPtr) {
    uint8_t window[9];

    for (uint16_t x=1; x<width-1; x++) {
        for (uint16_t y=1; y<height-1; y++) {
            uint8_t k = 0;
            for (int u=x-1; u<=x+1;u++) {
                for (int v=y-1; v<=y+1; v++) {
                    window[k++] = *(originaldataPtr + width*v+u);
                }
            }
            for (uint8_t i=0; i<5; i++) {
                for (uint8_t j=i+1; j<9; j++) {
                    if (window[j] < window[i]) {
                        uint8_t temp = window[i];
                        window[i] = window[j];
                        window[j] = temp;
                    }
                }
            }
            *(processed_dataPtr  + width*y+x) = window[4];
        }
    }
}



int main(void) {

    // START CODE HERE (you may test other 8-bit grayscale bmp)
    BMP* imageIn = BMP_ReadFile("noise.bmp");
    // END CODE HERE

    BMP_CHECK_ERROR(stdout, -1);

    int height = BMP_GetHeight(imageIn);
    int width  = BMP_GetWidth(imageIn);

    BMP* imageOut = BMP_Create(width, height, 8);
    for (int i=0; i<256; i++) {
    	BMP_SetPaletteColor(imageOut, i, i, i, i);
    }

    // START CODE HERE
    uint8_t rPtr = 0;
    uint8_t gPtr = 0;
    uint8_t bPtr = 0;

    uint8_t* r = &rPtr;
    uint8_t* g = &gPtr;
    uint8_t* b = &bPtr;

    uint8_t imgData[height*width];
    uint8_t* imgDataptr = imgData;

    uint8_t processed_img_data[height*width];
    uint8_t* processed_img_dataPtr = processed_img_data;

    for (int i=0; i <height; i++) {
        for (int j=0; j< width; j++) {
            imgData[i*width+j] = BMP_GetPixelGray(imageIn, j,i);
        }
    }

    //median filter
    medianFilter(width, height, imgDataptr, processed_img_dataPtr);

    for (int i=0; i <height; i++) {
        for (int j=0; j<width; j++) {
            BMP_SetPixelGray(imageOut,j,i, processed_img_data[i*width+j]);
        }
    }

    // END CODE HERE

    BMP_WriteFile(imageOut, "processed_noise.bmp");

    BMP_Free(imageIn);
    BMP_Free(imageOut);
    return 0;
}

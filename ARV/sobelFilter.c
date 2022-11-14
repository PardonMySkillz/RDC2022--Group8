#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "qdbmp.h"
#include "qdbmp.c"

int main(void) {

    // START CODE HERE (you may test other 8-bit grayscale bmp)
    BMP* imageIn = BMP_ReadFile("lenna.bmp");
    // END CODE HERE

    BMP_CHECK_ERROR(stdout, -1);

    int height = BMP_GetHeight(imageIn);
    int width  = BMP_GetWidth(imageIn);

    BMP* imageOut = BMP_Create(width, height, 8);
    for (int i=0; i<256; i++) {
    	BMP_SetPaletteColor(imageOut, i, i, i, i);
    }

    // START CODE HERE
    uint8_t window[9]; //set an array to store values of 3x3 matrix
        
    for (uint16_t x = 1; x < width - 1; x++) //outer edge of image will be ignored
    {
        for (uint16_t y = 1; y < height - 1; y++) //outer edge of image will be ignored
        {
            // Fill the 3x3 window
            uint8_t k = 0;
            for (int u = x - 1; u <= x + 1; u++) //horizontal
            {
                for (int v = y - 1; v <= y + 1; v++) //vertical
                {
                    window[k++] = BMP_GetPixelGray(imageIn, u, v);
                }
            }
            // Pass through Sobel Filter
            int16_t sumX = 0;
            int16_t sumY = 0;
            for (uint8_t i = 0; i < 9; i++) 
            {
                if (i == 0)
                {
                    sumX += window[i] * -1;
                    sumY += window[i] * -1;
                }
                else if (i == 1)
                {
                    sumX += window[i] * 0;
                    sumY += window[i] * -2;
                }
                else if (i == 2)
                {
                    sumX += window[i] * 1;
                    sumY += window[i] * -1;
                }
                else if (i == 3)
                {
                    sumX += window[i] * -2;
                    sumY += window[i] * 0;
                }
                else if (i == 4)
                {
                    sumX += window[i] * 0;
                    sumY += window[i] * 0;
                }
                else if (i == 5)
                {
                    sumX += window[i] * 2;
                    sumY += window[i] * 0;
                }
                else if (i == 6)
                {
                    sumX += window[i] * -1;
                    sumY += window[i] * 1;
                }
                else if (i == 7)
                {
                    sumX += window[i] * 0;
                    sumY += window[i] * 2;
                }
                else if (i == 8)
                {
                    sumX += window[i] * 1;
                    sumY += window[i] * 1;
                }         
            }
            //Calculate Magnitude
            uint16_t magnitude = sqrt((sumX * sumX) + (sumY * sumY));
            BMP_SetPixelGray(imageOut, x, y, (magnitude / 8));
        }
    }
    
    // END CODE HERE

    BMP_WriteFile(imageOut, "output.bmp");
    BMP_Free(imageIn);
    BMP_Free(imageOut);
    return 0;
}
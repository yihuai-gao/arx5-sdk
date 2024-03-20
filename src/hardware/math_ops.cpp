#include "hardware/math_ops.h"

float fmaxf3(float x, float y, float z)
{
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z)
{
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

void limit_norm(float *x, float *y, float limit)
{
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if (norm > limit)
    {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
float int_to_float(int val, float max_, float min_, int bits)
{
    return min_ + ((float)val / (float)0xFFFF) * (max_ - min_);
}

union F32
{
    float v_float;
    unsigned int v_int;
    unsigned char buf[4];
} f32;

void float32_to_float16(float *float32, unsigned short int *float16)
{
    unsigned short int temp = 0;
    f32.v_float = *float32;
    //	*float16 = ((f32.v_int & 0x7fffffff) >> 13) - (0x38000000 >> 13);
    //  *float16 |= ((f32.v_int & 0x80000000) >> 16);
    temp = (f32.buf[3] & 0x7F) << 1 | ((f32.buf[2] & 0x80) >> 7);
    temp -= 112;
    *float16 = temp << 10 | (f32.buf[2] & 0x7F) << 3 | f32.buf[1] >> 5;
    *float16 |= ((f32.v_int & 0x80000000) >> 16);
}

void float16_to_float32(unsigned short int *float16, float *float32)
{
    //	f32.v_int=*float16;
    //	f32.v_int = ((f32.v_int & 0x7fff) << 13) + 0x7f000000;
    //  f32.v_int |= ((*float16 & 0x8000) << 16);
    //	*float32=f32.v_float;
    unsigned short int temp2 = 0;
    f32.v_int = 0;
    temp2 = (((*float16 & 0x7C00) >> 10) + 112);
    f32.buf[3] = temp2 >> 1;
    f32.buf[2] = ((temp2 & 0x01) << 7) | (*float16 & 0x03FC) >> 3;
    f32.buf[1] = (*float16 & 0x03) << 6;
    f32.v_int |= ((*float16 & 0x8000) << 16);
    *float32 = f32.v_float;
}
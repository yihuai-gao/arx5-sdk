
#include "utils.h"
#include <cstdarg>
#include <cstdio>

MovingAverage6d::MovingAverage6d(int window_size)
{
    set_window_size(window_size);
    reset();
}

MovingAverage6d::~MovingAverage6d()
{
}

void MovingAverage6d::set_window_size(int window_size)
{
    _window_size = window_size;
    _window = new Vec6d[_window_size];
}

void MovingAverage6d::reset()
{
    _window_index = 0;
    _window_sum = Vec6d::Zero();
    for (int i = 0; i < _window_size; i++)
    {
        _window[i] = Vec6d::Zero();
    }
}

Vec6d MovingAverage6d::filter(Vec6d new_data)
{
    _window_sum -= _window[_window_index];
    _window_sum += new_data;
    _window[_window_index] = new_data;
    _window_index = (_window_index + 1) % _window_size;
    return _window_sum / _window_size;
}
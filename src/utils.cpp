
#include "utils.h"
#include <cstdarg>
#include <cstdio>

namespace arx
{

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

// std::string vec2str(const Eigen::VectorXd& vec, int precision) {
//   std::string str = "[";
//   for (int i = 0; i < vec.size(); i++) {
//     char buffer[50];
//     std::sprintf(buffer, "%.*f", precision, vec(i));
//     str += buffer;
//     if (i < vec.size() - 1) {
//       str += ", ";
//     }
//   }
//   str += "]";
//   return str;
// }
} // namespace arx

std::string vec2str(const Eigen::Matrix<double, 6, 1> &vec, int precision)
{
    std::string str = "[";
    for (int i = 0; i < vec.size(); i++)
    {
        char buffer[50];
        std::sprintf(buffer, "%.*f", precision, vec(i));
        str += buffer;
        if (i < vec.size() - 1)
        {
            str += ", ";
        }
    }
    str += "]";
    return str;
}
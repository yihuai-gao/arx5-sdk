
#include "utils.h"
#include <cstdarg>
#include <cstdio>

namespace arx
{

MovingAverageXd::MovingAverageXd(int dof, int window_size)
{
    _dof = dof;
    _window_size = window_size;
    reset();
}

MovingAverageXd::~MovingAverageXd()
{
}

void MovingAverageXd::reset()
{
    _window_index = 0;
    _window_sum = Eigen::VectorXd::Zero(_dof);
    _window = Eigen::MatrixXd::Zero(_window_size, _dof);
}

Eigen::VectorXd MovingAverageXd::filter(Eigen::VectorXd new_data)
{
    _window_sum -= _window.row(_window_index);
    _window_sum += new_data;
    _window.row(_window_index) = new_data;
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

std::string vec2str(const Eigen::VectorXd &vec, int precision)
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
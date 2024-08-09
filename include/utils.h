#ifndef UTILS_H
#define UTILS_H
#include "app/common.h"
#include <Eigen/Core>

namespace arx
{
class MovingAverageXd
{
  public:
    MovingAverageXd(int dof, int window_size);
    ~MovingAverageXd();

    void set_window_size(int window_size);
    void reset();
    Eigen::VectorXd filter(Eigen::VectorXd new_data);

  private:
    int _dof;
    int _window_size;
    int _window_index;
    Eigen::VectorXd _window_sum;
    Eigen::MatrixXd _window;
};

// std::string vec2str(const Eigen::VectorXd& vec, int precision = 3);

} // namespace arx
std::string vec2str(const Eigen::VectorXd &vec, int precision = 3);

#endif

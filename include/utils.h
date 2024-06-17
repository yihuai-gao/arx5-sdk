#ifndef UTILS_H
#define UTILS_H
#include "app/common.h"

class MovingAverage6d {
 public:
  MovingAverage6d(int window_size);
  ~MovingAverage6d();

  void set_window_size(int window_size);
  void reset();
  Vec6d filter(Vec6d new_data);

 private:
  int _window_size;
  int _window_index;
  Vec6d _window_sum;
  Vec6d* _window;
};

#endif

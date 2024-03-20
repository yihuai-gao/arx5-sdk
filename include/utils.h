#ifndef UTILS_H
#define UTILS_H
#include "app/common.h"
enum LogLevel
{
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3
};
#ifndef LOG_LEVEL
#define LOG_LEVEL 1
#endif

void debug_printf(int msg_level, int log_level, const char *fmt, ...);

class MovingAverage6d
{
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
    Vec6d *_window;
};

#endif

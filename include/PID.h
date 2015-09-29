#ifndef H_PID
#define	H_PID

typedef struct {
    float desired;
    float error;
    float integ;
    float deriv;
    float Kp, Ki, Kd;
    float output;
} PIDControl;

#endif

typedef struct {
    double q[4];
    double dt;
    double roll;
    double pitch;
    double yaw;
    double anorm[3];
    double v[3];
    double eInt[3];
} Quaternion;

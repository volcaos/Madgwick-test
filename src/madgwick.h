
float gyro_error_rs;
float gyro_error_rss;

void reset_quaternion();

void read_quaternion( float *q );
void read_eulerAngle( float *v );

void madgwick_filterAHRS( float deltat, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z );
void madgwick_filterIMU( float deltat, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z );
void madgwick_filterIMU2( float deltat, float gx, float gy, float gz, float ax, float ay, float az );

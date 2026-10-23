#ifndef PID_H
#define PID_H

namespace PID {

void reset();
void update();
void setFollowing(bool enabled);
bool isFollowing();
float kp();
float ki();
float kd();
int baseSpeed();
int pwmLimit();
void setKp(float value);
void setKi(float value);
void setKd(float value);
void setBaseSpeed(int value);
void setPwmLimit(int value);

}  // namespace PID

#endif
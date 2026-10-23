#ifndef BUTTON_H
#define BUTTON_H

namespace Button {

bool handle(char buttonCode, bool pressed, int baseSpeed, int pwmLimit);
void clear();

}  // namespace Button

#endif
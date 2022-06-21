#ifndef COMMON_OPERATOR_H
#define COMMON_OPERATOR_H

float mapfloat(long x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float soften_value(float axis, float soften, float max_v){
  return axis * pow(abs(axis/max_v), soften);
}
#endif

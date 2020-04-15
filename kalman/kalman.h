// Tiny Kalman Filter.
// Based on https://github.com/denyssene/SimpleKalmanFilter

#include "Arduino.h"
#include <math.h>

// 1D Kalman filter.
class Kalman1d {
private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
public:
  Kalman1d(float mea_e, float est_e, float q);
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();
};

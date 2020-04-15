// Tiny Kalman Filter.
// Based on https://github.com/denyssene/SimpleKalmanFilter

#include "Arduino.h"
#include <math.h>
#include "kalman.h"

Kalman1d::Kalman1d(float mea_e, float est_e, float q) {
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}
float Kalman1d::updateEstimate(float mea) {
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate
                    + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;
  return _current_estimate;
}
void Kalman1d::setMeasurementError(float mea_e) { _err_measure=mea_e; }
void Kalman1d::setEstimateError(float est_e) { _err_estimate=est_e; }
void Kalman1d::setProcessNoise(float q) { _q=q; }
float Kalman1d::getKalmanGain() { return _kalman_gain; }
float Kalman1d::getEstimateError() { return _err_estimate; }

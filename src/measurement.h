#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "Eigen/Dense"
#include <memory>
#include <sstream>
#include "tools.h"

using namespace std;



class Sensor {
public:
  virtual ~Sensor(){}
protected:
  Sensor(){}
};


void normalize_ctrv(Eigen::VectorXd & x);

struct Measurement {
public:
  long long timestamp_;

  // decoded state variables
  float p_x = NAN;
  float p_y = NAN;
  float v = NAN;
  float yaw = NAN;
  float yaw_dot = NAN;

  // measurement information
  int n_z = NAN;

  // measurement covariance
  Eigen::MatrixXd R;

  // measurement / observation
  Eigen::VectorXd z;

  virtual void get_ctrv_state(Eigen::VectorXd & x) {}

  virtual void state_to_measure(Eigen::VectorXd x, Eigen::VectorXd &z) {}
  virtual void normalize_measure(Eigen::VectorXd &z){}

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;


  Eigen::VectorXd raw_measurements_;
};

struct LaserMeasurement : public Measurement
{
  // Laser measurement noise standard deviation position1 in m
  const float std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  const float std_laspy_ = 0.15;

public:
  LaserMeasurement(std::istream & s);

  virtual void get_ctrv_state(Eigen::VectorXd & x);

  virtual void state_to_measure(Eigen::VectorXd x, Eigen::VectorXd &z);

  virtual ~LaserMeasurement(){}
};

struct RadarMeasurement : public Measurement {
public:
  float ro = NAN;  // radial distance to object
  float phi = NAN; // angle to object
  float ro_dot = NAN; // rate of change of radial distance

  ///* Radar measurement noise standard deviation radius in m
  const double std_radr = 0.3;

  ///* Radar measurement noise standard deviation angle in rad
  const double std_radrphi = 0.03;

  ///* Radar measurement noise standard deviation radius change in m/s
  const double std_radrd = 0.3;

  RadarMeasurement(std::istream & s);

  // sets measure vector z for the given state vector x
  virtual void state_to_measure(Eigen::VectorXd x, Eigen::VectorXd &z);

  virtual void normalize_measure(Eigen::VectorXd &z);

  virtual void get_ctrv_state(Eigen::VectorXd &x) {
    x << p_x, p_y, v, 0, 0;
  }

  virtual ~RadarMeasurement(){}
};

Measurement * read_measurement(std::istream & s);


#endif /* MEASUREMENT_PACKAGE_H_ */

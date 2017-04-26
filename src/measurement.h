#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "Eigen/Dense"
#include <memory>
#include <sstream>

using namespace std;

class Sensor {
public:
  virtual ~Sensor(){}
protected:
  Sensor(){}
};


struct Measurement {
public:
  long long timestamp_;

  // decoded state variables
  float p_x = NAN;
  float p_y = NAN;
  float v = NAN;
  float yaw = NAN;
  float yaw_dot = NAN;

  // observation model, aka Measurement marix
  Eigen::MatrixXd H;

  // measurement covariance
  Eigen::MatrixXd R;

  // measurement / observation
  Eigen::VectorXd z;



  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;


  Eigen::VectorXd raw_measurements_;
};

struct LaserMeasurement : public Measurement
{
public:
  LaserMeasurement(std::istream & s) 
  {
    sensor_type_ = Measurement::LASER;
    s >> p_x >> p_y >> timestamp_;
    raw_measurements_ = Eigen::VectorXd(2);
    raw_measurements_ << p_x, p_y;
  }
  virtual ~LaserMeasurement(){}
};

struct RadarMeasurement : public Measurement {
public:
  float ro = NAN;  // radial distance to object
  float phi = NAN; // angle to object
  float ro_dot = NAN; // rate of change of radial distance

  RadarMeasurement(std::istream & s) 
  {
    sensor_type_ = Measurement::RADAR;
    s >> ro >> phi >> ro_dot >> timestamp_;
    p_x = ro * sin(phi);
    p_y = ro * cos(phi);
    raw_measurements_ = Eigen::VectorXd(3);
    raw_measurements_ << ro, phi, ro_dot;
  }
  virtual ~RadarMeasurement(){}
};

Measurement * read_measurement(std::istream & s);


#endif /* MEASUREMENT_PACKAGE_H_ */

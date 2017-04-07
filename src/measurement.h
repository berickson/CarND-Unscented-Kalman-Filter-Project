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
  float x = NAN;
  float y = NAN;

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
    s >> x >> y >> timestamp_;
    raw_measurements_ = Eigen::VectorXd(2);
    raw_measurements_ << x, y;
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
    x = ro * sin(phi);
    y = ro * cos(phi);
    raw_measurements_ = Eigen::VectorXd(3);
    raw_measurements_ << ro, phi, ro_dot;
  }
  virtual ~RadarMeasurement(){}
};

Measurement * read_measurement(std::istream & s);


#endif /* MEASUREMENT_PACKAGE_H_ */

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

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


struct MeasurementPackage {
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

struct LaserSensor : public MeasurementPackage
{
public:
  LaserSensor(std::istream & s) 
  {
    sensor_type_ = MeasurementPackage::LASER;
    s >> x >> y >> timestamp_;
    raw_measurements_ = Eigen::VectorXd(2);
    raw_measurements_ << x, y;
  }
  virtual ~LaserSensor(){}
};

struct RadarSensor : public MeasurementPackage {
public:
  float ro = NAN;  // radial distance to object
  float phi = NAN; // angle to object
  float ro_dot = NAN; // rate of change of radial distance

  RadarSensor(std::istream & s) 
  {
    sensor_type_ = MeasurementPackage::RADAR;
    s >> ro >> phi >> ro_dot >> timestamp_;
    x = ro * sin(phi);
    y = ro * cos(phi);
    raw_measurements_ = Eigen::VectorXd(3);
    raw_measurements_ << ro, phi, ro_dot;
  }
  virtual ~RadarSensor(){}
};

MeasurementPackage * read_measurement(std::istream & s);


#endif /* MEASUREMENT_PACKAGE_H_ */

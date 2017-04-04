#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
#include <memory>
#include <sstream>

using namespace std;

class Sensor {
public:
  virtual ~Sensor(){}
  Sensor(){}
  //Sensor(const Sensor&) = delete;
  //Sensor& operator=(const Sensor&) = delete;  
};

class LaserSensor : public Sensor{
public:
  float x = NAN;
  float y = NAN;

  LaserSensor(std::istream & s) : Sensor() {
    s >> x >> y;
  }
  virtual ~LaserSensor();
};

class RadarSensor : public Sensor {
public:
  float ro = NAN;  // radial distance to object
  float phi = NAN; // angle to object
  float ro_dot = NAN; // rate of change of radial distance

  float x = NAN;
  float y = NAN;

  RadarSensor(std::istream & s) {
    s >> ro >> phi >> ro_dot;
    x = ro * sin(phi);
    y = ro * cos(phi);
  }
  virtual ~RadarSensor();
};

class MeasurementPackage {
public:
  MeasurementPackage(std::istream & s) 
  {
    string sensor_type_char;
    s >> sensor_type_char;
    if (sensor_type_char.compare("L") == 0) {
      //sensor_.reset(new LaserSensor(s));
      return;
    } 
    if (sensor_type_char.compare("R") == 0) {
      //sensor_.reset(new RadarSensor(s));
      return;
    } 

  }
  long long timestamp_;

  unique_ptr<Sensor> sensor_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */

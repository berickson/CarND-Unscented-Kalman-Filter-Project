#include "measurement_package.h"

MeasurementPackage * read_measurement(std::istream & s) 
  {
    MeasurementPackage * rv;
    string t;
    s >> t;
    if (t.compare("L") == 0) {
      rv = new LaserSensor(s);
    } else if (t.compare("R") == 0){  
      rv = new RadarSensor(s);
    } else {
      throw "Unknown sensor type";
    }
  }


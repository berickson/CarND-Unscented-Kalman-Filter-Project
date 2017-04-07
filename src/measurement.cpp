#include "measurement.h"

Measurement * read_measurement(std::istream & s) 
  {
    Measurement * rv;
    string t;
    s >> t;
    if (t.compare("L") == 0) {
      rv = new LaserMeasurement(s);
    } else if (t.compare("R") == 0){  
      rv = new RadarMeasurement(s);
    } else {
      throw "Unknown sensor type";
    }
  }


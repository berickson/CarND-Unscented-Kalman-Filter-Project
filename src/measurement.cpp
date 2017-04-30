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
    return rv;
  }


void normalize_ctrv(Eigen::VectorXd &x) {
  x(3) = normalize_angle(x(3));
}

LaserMeasurement::LaserMeasurement(istream &s)
{
  sensor_type_ = Measurement::LASER;
  n_z = 2;
  R = Eigen::MatrixXd(n_z,n_z);
  R <<    std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;

  s >> p_x >> p_y >> timestamp_;
  raw_measurements_ = Eigen::VectorXd(2);
  raw_measurements_ << p_x, p_y;
}

void LaserMeasurement::get_ctrv_state(Eigen::VectorXd &x){
  x << p_x, p_y, 0, 0, 0;
}

void LaserMeasurement::state_to_measure(Eigen::VectorXd x, Eigen::VectorXd &z) {
  z(0) = x(0);
  z(1) = x(1);
}

RadarMeasurement::RadarMeasurement(istream &s)
{
  sensor_type_ = Measurement::RADAR;
  n_z = 3;

  R = Eigen::MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
      0, std_radrphi*std_radrphi, 0,
      0, 0,std_radrd*std_radrd;

  s >> ro >> phi >> ro_dot >> timestamp_;
  p_x = ro * cos(phi);
  p_y = ro * sin(phi);
  auto v_x = ro_dot * cos(phi);
  auto v_y = ro_dot * sin(phi);
  v = sqrt(v_x * v_x + v_y + v_y);
  raw_measurements_ = Eigen::VectorXd(3);
  raw_measurements_ << ro, phi, ro_dot;
}

void RadarMeasurement::state_to_measure(Eigen::VectorXd x, Eigen::VectorXd &z) {

  double p_x = x(0);
  double p_y = x(1);
  double v   = x(2);
  double yaw = x(3);

  double v1 = cos(yaw)*v;
  double v2 = sin(yaw)*v;

  // measurement model
  z(0) = sqrt(p_x*p_x + p_y*p_y);                        //r
  z(1) = atan2(p_y,p_x);                                 //phi
  z(2) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
}

void RadarMeasurement::normalize_measure(Eigen::VectorXd &z) {
  z(1) = normalize_angle(z(1));
}

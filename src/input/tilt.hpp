// Tilt driving (Ally X accelerometer through the Windows Sensor API). Off by default; only while RT is held.
// The angle maths is plain and portable (unit-tested); the sensor itself exists on Windows only.
#pragma once

#include <memory>
#include <string>

namespace ally::input {

struct Gravity {
  double x{0}, y{0}, z{0};  // device axes in g (Windows: x right, y up along the screen, z out of the screen)
};

// Pitch: angle between gravity and the screen plane — tipping the top edge away from the user (+), 0 = upright facing
// the user, 90 = flat screen-up. Roll: gravity towards the right edge (+). Turning the handheld like a steering wheel
// (about the screen normal) changes only roll; tipping it forward/back (about its right axis) changes only pitch.
double tilt_pitch_deg(const Gravity& g);
double tilt_roll_deg(const Gravity& g);

struct TiltDrive {
  double forward{0}, turn{0};  // -1..1 (+turn = counter-clockwise, like the sticks)
};
// Angles relative to the pose captured when RT went down. dead_deg: no motion inside; full_deg: full stick.
// Tipping forward drives forward; right side down turns right (clockwise).
TiltDrive tilt_to_drive(double pitch_deg, double roll_deg, double ref_pitch_deg, double ref_roll_deg, double dead_deg,
                        double full_deg);

class TiltSensor {
 public:
  TiltSensor();
  ~TiltSensor();
  TiltSensor(const TiltSensor&) = delete;
  TiltSensor& operator=(const TiltSensor&) = delete;
  void start();                 // starts the sensor thread; never blocks
  int state() const;            // 0 = starting, 1 = running, -1 = no sensor
  bool open(std::string* err);  // start() + wait up to 1.5 s for the result (console tool)
  bool available() const;
  // Latest value. stamp = mono time of the sensor's last answer (new report or "unchanged"). false if none or stale.
  bool read(Gravity* g, double* stamp = nullptr);
  std::string last_error() const;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;  // shared with the sensor thread, which may outlive this object briefly
};

}  // namespace ally::input

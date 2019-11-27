#pragma once
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include "ur_modern_driver/ur/server.h"
#include "ur_modern_driver/ur/stream.h"

struct URCommanderOpts
{
  std::string reverse_ip;
  int reverse_port;
  bool version_3;
  double servoj_time{ 0.008 };
  double servoj_lookahead_time{ 0.03 };
  double servoj_gain{ 300. };
};

class URCommander
{
private:
  URStream &stream_;

  URServer server_;

  std::string position_program_, velocity_program_;
  bool servo_loop_running_;
  double servoj_time_;

  template <typename T>
  size_t append(uint8_t *buffer, T &val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

protected:
  void preparePositionProgram(const URCommanderOpts &options);
  void prepareVelocityProgram(const URCommanderOpts &options, const std::string &speedj_replace);

  bool write(const std::string &s);
  void formatArray(std::ostringstream &out, std::array<double, 6> &values);

public:
  URCommander(URStream &stream, const URCommanderOpts &options);

  virtual bool setDigitalOut(uint8_t pin, bool value) = 0;
  virtual bool setAnalogOut(uint8_t pin, double value) = 0;

  // shared
  bool uploadProg(const std::string &s);
  bool stopj(double a = 10.0);
  bool setToolVoltage(uint8_t voltage);
  bool setFlag(uint8_t pin, bool value);
  bool setPayload(double value);

  bool startSpeedLoop();
  void stopSpeedLoop();
  bool speedj(std::array<double, 6> &speeds, double acceleration, bool keep_alive = true);

  bool startServoLoop();
  void stopServoLoop();
  bool servoj(std::array<double, 6> &positions, bool keep_alive);
  double servojTime()
  {
    return servoj_time_;
  }
};

class URCommander_CB : public URCommander
{
public:
  URCommander_CB(URStream &stream, const URCommanderOpts &options);
};

class URCommander_V1_X : public URCommander_CB
{
public:
  URCommander_V1_X(URStream &stream, const URCommanderOpts &options) : URCommander_CB(stream, options)
  {
  }

  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

class URCommander_V3_X : public URCommander_CB
{
public:
  URCommander_V3_X(URStream &stream, const URCommanderOpts &options) : URCommander_CB(stream, options)
  {
  }

  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

class URCommander_e : public URCommander
{
public:
  URCommander_e(URStream &stream, const URCommanderOpts &options);

  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

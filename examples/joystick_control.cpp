#include "create/create.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <thread>
#include <chrono>

struct JoystickCommand
{
    float left_wheel_vel_mps;
    float right_wheel_vel_mps;
    float accel_mpss;
    float p_gain;
    float i_gain;
};

struct WheelStatus
{
    float current_vel_mps;
    float accum_error;
};

static const uint32_t CONTROL_LOOP_UPDATE_INTERVAL_MS = 16;
static const float CONTROL_LOOP_HZ = 1000.0f/CONTROL_LOOP_UPDATE_INTERVAL_MS;

static create::Create* robot;

static JoystickCommand s_current_command;
static WheelStatus s_left_status;
static WheelStatus s_right_status;


static void joystick_thread_execute(void *args)
{
    (void)args;

    int sock_fd;
    struct sockaddr_in in_addr;
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        exit(1);
    }
    memset((char*)&in_addr, 0, sizeof(in_addr));
    in_addr.sin_family = AF_INET;
    in_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    in_addr.sin_port = htons(9999);
    if (bind(sock_fd, (struct sockaddr *)&in_addr, sizeof(in_addr)) < 0)
    {
        std::cout << "Failed to bind to socket" << std::endl;
        exit(1);
    }

    while (1)
    {
        recvfrom(sock_fd, (uint8_t*)&s_current_command, sizeof(s_current_command), 0, NULL, 0);
    }

    close(sock_fd);
}

static void ramp_vel(float target_vel_mps, float accel_mpscy, float &current_vel_mps)
{
    float vel_delta = target_vel_mps - current_vel_mps;

    if (fabsf(vel_delta) < accel_mpscy)
    {
        current_vel_mps = target_vel_mps;
    }
    else if (vel_delta > 0)
    {
        current_vel_mps += accel_mpscy;
    }
    else
    {
        current_vel_mps -= accel_mpscy;
    }
}

static void pid_update(float target_vel_mps, float measured_vel_mps, float p_gain, float i_gain, float &accum_error, int16_t &output)
{
    static const int16_t MAX_OUTPUT = 255;

    if (target_vel_mps != 0)
    {
        float error = target_vel_mps - measured_vel_mps;

        if (abs(output) < MAX_OUTPUT)
        {
            accum_error += error;
        }
        output = p_gain * error;
        output += i_gain * accum_error;

        output = std::min(output, MAX_OUTPUT);
        output = std::max(output, static_cast<int16_t>(-1*MAX_OUTPUT));
    }
    else
    {
        accum_error = 0;
        output = 0;
    }
}


int main(int argc, char** argv)
{
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  create::RobotModel model = create::RobotModel::CREATE_2;

  if (argc > 1 && std::string(argv[1]) == "create1")
  {
    model = create::RobotModel::CREATE_1;
    baud = 57600;
    std::cout << "1st generation Create selected" << std::endl;
  }

  robot = new create::Create(model);

  // Attempt to connect to Create
  if (robot->connect(port, baud))
  {
    std::cout << "Successfully connected to Create" << std::endl;
  }
  else
  {
    std::cout << "Failed to connect to Create on port " << port.c_str() << std::endl;
    return 1;
  }

  robot->setMode(create::MODE_FULL);

  usleep(1000000);

  std::thread joystick_thread(joystick_thread_execute, nullptr);

  std::cout << "Entering control loop" << std::endl;

  // Quit when center "Clean" button pressed
  while (!robot->isCleanButtonPressed())
  {
      static std::chrono::high_resolution_clock::time_point last_time;

      ramp_vel(s_current_command.left_wheel_vel_mps,
               s_current_command.accel_mpss / CONTROL_LOOP_HZ,
               s_left_status.current_vel_mps);
      ramp_vel(s_current_command.right_wheel_vel_mps,
               s_current_command.accel_mpss / CONTROL_LOOP_HZ,
               s_right_status.current_vel_mps);

      static float last_left_wheel_dist = 0;
      static float last_right_wheel_dist = 0;

      float left_measured_vel_mps = (robot->getLeftWheelDistance() - last_left_wheel_dist) * CONTROL_LOOP_HZ;
      float right_measured_vel_mps = (robot->getRightWheelDistance() - last_right_wheel_dist) * CONTROL_LOOP_HZ;
      last_left_wheel_dist = robot->getLeftWheelDistance();
      last_right_wheel_dist = robot->getRightWheelDistance();

      if (s_current_command.p_gain == 0 && s_current_command.i_gain == 0)
      {
          robot->driveWheels(s_left_status.current_vel_mps, s_right_status.current_vel_mps);
      }
      else
      {
          static int16_t left_duty;
          static int16_t right_duty;

          pid_update(s_left_status.current_vel_mps,
                     left_measured_vel_mps,
                     s_current_command.p_gain,
                     s_current_command.i_gain,
                     s_left_status.accum_error,
                     left_duty);
          pid_update(s_right_status.current_vel_mps,
                     right_measured_vel_mps,
                     s_current_command.p_gain,
                     s_current_command.i_gain,
                     s_right_status.accum_error,
                     right_duty);

          robot->driveWheelsPWM(left_duty, right_duty);

          auto cur_time = std::chrono::high_resolution_clock::now();
          auto dur = cur_time - last_time;
          auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

          std::cout << left_duty << " " << left_measured_vel_mps << " " << ms << std::endl;
          last_time = cur_time;
      }

      usleep(1000 * CONTROL_LOOP_UPDATE_INTERVAL_MS); //66hz
  }

  std::cout << "Stopping Create." << std::endl;


  robot->disconnect();
  delete robot;

  return 0;
}

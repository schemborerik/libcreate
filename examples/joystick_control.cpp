#include "create/create.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

struct JoystickCommand
{
    float trans_vel;
    float ang_vel;
};

create::Create* robot;

int main(int argc, char** argv) {
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  create::RobotModel model = create::RobotModel::CREATE_2;

  if (argc > 1 && std::string(argv[1]) == "create1") {
    model = create::RobotModel::CREATE_1;
    baud = 57600;
    std::cout << "1st generation Create selected" << std::endl;
  }

  robot = new create::Create(model);

  // Attempt to connect to Create
  if (robot->connect(port, baud))
    std::cout << "Successfully connected to Create" << std::endl;
  else {
    std::cout << "Failed to connect to Create on port " << port.c_str() << std::endl;
    return 1;
  }

  robot->setMode(create::MODE_SAFE);

  usleep(1000000);

  // Drive in a circle
  robot->drive(0, 0);


  int sock_fd;
  struct sockaddr_in in_addr;
  if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
      return 1;
  }
  memset((char*)&in_addr, 0, sizeof(in_addr));
  in_addr.sin_family = AF_INET;
  in_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  in_addr.sin_port = htons(9999);
  if (bind(sock_fd, (struct sockaddr *)&in_addr, sizeof(in_addr)) < 0)
  {
      std::cout << "Failed to bind to socket" << std::endl;
      return 1;
  }


  // Quit when center "Clean" button pressed
  while (!robot->isCleanButtonPressed()) {

      JoystickCommand command;

      recvfrom(sock_fd, (uint8_t*)&command, sizeof(command), 0, NULL, 0);

      robot->drive(command.trans_vel, command.ang_vel);

      usleep(1000 * 100); //10hz
  }

  std::cout << "Stopping Create." << std::endl;

  close(sock_fd);

  robot->disconnect();
  delete robot;

  return 0;
}

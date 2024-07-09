#include "../include/uart.hpp"
#include <string.h>
using namespace std;

int main(int argc, char* argv[])
{
    printf("speed: +-0.8; angle 1500+-1000\n");
    char ttyUSB[100];
    ttyUSB[0] = 0;
    std::string cmd = "ls /dev | grep ttyUSB";
    while(strlen(ttyUSB) == 0){
        executeCMD(cmd.c_str(), ttyUSB);
    }
    cout << ttyUSB << std::endl;
    ttyUSB[strlen(ttyUSB) - 1] = 0;
    std::shared_ptr<Driver> driver = std::make_shared<Driver>(std::string("/dev/") + std::string(ttyUSB), BaudRate::BAUD_115200);
    driver->open();
    if (argc == 1)
    {
        int tar = 500;
        while(true){
            waitKey(50);
            driver->sendCMD_pid(0);
            waitKey(1);
            driver->sendCMD_servo(tar);
            tar = 3000 - tar;
        }
    }
    else 
    {
        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "speed")==0) driver->sendCMD_pid(atof(argv[++i]));
            else if (strcmp(argv[i], "angle")==0) driver->sendCMD_servo(atoi(argv[++i]));
            else if (strcmp(argv[i], "EnableMotor") == 0) driver->sendCMD_enableMotor();
            else {printf("invalid string = \"%s\"!\n", argv[i]); return 0; }
        }
    }
    return 0;
}
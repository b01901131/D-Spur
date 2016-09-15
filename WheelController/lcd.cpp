#include "LcdController.h"
#include <string>

using namespace std;

int main (void){
    //string i2c_filename = "/dev/i2c-0";
    I2CBusController* i2c_bus = new I2CBusController("/dev/i2c-0");
    LcdController lcd(i2c_bus, 0x27, 16, 2);
    lcd.init();
    string msg = "Hello, world!";
    lcd.print(msg.c_str(), 13);

    return 0;
}

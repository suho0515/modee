#include <ModEE_Controller.h>

ModEE_Controller modee_controller;

void setup()
{
  
}

void loop()
{
  modee_controller.rotary_control(2, 30, 30);
}

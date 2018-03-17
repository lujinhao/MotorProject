# PID_controller

PID_controller.c is developped by group C7 for motor position control  

PID_controller.c has been tested with commercial motor with the following performance:

      - 3% overshoot
      - 30ms raising time for 9 degree change
      - ~0 steady state error
 
PID_controller is designed for Arduino mega 2560

             ├── calculates PID values at 4Hkz (timer1)
             ├── updates setpoint at 25Hz (timer3).
             └── integrates the interface to quadrature decoders and motor drivers

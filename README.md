# PID-Controller-to-Control-Speed-of-DC-motor

**Objective of this project**: Design a PID (Proportional Integral Derivative) control mechanism for accurate speed control of a DC Motor.<br>
<br>
**Components Used**: DC Motor,Breadboard,STM32 Microcontroller,L293D Motor Driver,Power Supply Module,Rotatory Encoder ,Jumper wires.<br>
<br>
**Softwares used**:STM32CubeIDE,RealTerm.<br>

**Implementation**:<br>
1)Setting GPIO Pins for motor rotation as well as encoder rotation.<br>
2)Using timers to implement PWM to run DC motor.<br>
3)Using interrupt for encoder to tune the motor based on PID logic mimising error and giving the desired speed value.<br>
4)Via UART, results are displayed using RealTerm application.

<br>
PID mechanism helps minimising steady state and transient errors and make the motor rotate at the desired speed smoothly even under varying load conditions.


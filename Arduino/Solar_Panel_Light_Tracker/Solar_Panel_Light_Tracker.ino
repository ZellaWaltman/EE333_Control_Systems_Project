#include <Servo.h>  
#include <PID_v1.h>

//----------------
// PID VARIABLES
//----------------

double Setpoint; // Same for both, zero

// Vertical PID variables
double Input_Vert, Output_Vert;
double KpV = 0.05, KiV = 0.05, KdV = 0.0;

// Horizontal PID variables
double Input_Hori, Output_Hori;
double KpH = 0.15, KiH = 0.1, KdH = 0.0;

// PID controllers
PID myPID_Vert(&Input_Vert, &Output_Vert, &Setpoint, KpV, KiV, KdV, DIRECT);
PID myPID_Hori(&Input_Hori, &Output_Hori, &Setpoint, KpH, KiH, KdH, DIRECT);

Servo vertical; // Vertical Servo Motor
double servovert = 90.0;  // Double allows for smoother motion - decimal points of degree changes!
int servovertLimitHigh = 150;
int servovertLimitLow = 30;

Servo horizontal; // Horizontal Servo Motor
double servohori = 90.0;  
int servohoriLimitHigh = 180;
int servohoriLimitLow = 0;

// LDR pin connections
int ldrtl = A0; // Top Left LDR
int ldrtr = A1; // Top Right LDR
int ldrbl = A2; // Bottom Left LDR
int ldrbr = A3; // Bottom Right LDR

void setup() {

  Serial.begin(9600); // Set Baud rate for Serial data

// Delay for Setting Up Serial Monitoring (ONLY When Recording Data)
//Serial.println("Waiting 10 seconds before starting...");
//delay(10000);  // 10-second pause

  //----------------------
  // SERVO SETUP 
  //----------------------
  horizontal.attach(2);
  vertical.attach(13);

  // Initialize Servos at 90 Degrees
  horizontal.write(90);
  vertical.write(90);

  delay(2500); // Let system settle
  
  //----------------------
  // VERTICAL PID SETUP
  //----------------------
  Setpoint = 0; // Goal: equal light in all 4 Quadrants (Panel Recieving Maximum Light)
  myPID_Vert.SetMode(AUTOMATIC); // Turn PID on
  myPID_Vert.SetTunings(KpV, KiV, KdV); // Adjust PID values
  myPID_Vert.SetOutputLimits(-5, 5); // Limit how much the servo can change per loop

  //----------------------
  // Horizontal PID SETUP
  //----------------------
  myPID_Hori.SetMode(AUTOMATIC); // Turn PID on
  myPID_Hori.SetTunings(KpH, KiH, KdH); // Adjust PID values
  myPID_Hori.SetOutputLimits(-5, 5); // Limit how much the servo can change per loop

}

void loop() {
  
  // Read LDR analog values
  int ld = analogRead(ldrbl); // Bottom left
  int rd = analogRead(ldrbr); // Bottom right
  int lt = analogRead(ldrtl); // Top left
  int rt = analogRead(ldrtr); // Top right

  // Comput average value of sensors
  int avt = (lt + rt) / 2; // Average value of top sensors
  int avd = (ld + rd) / 2; // Average value of bottom sensors
  int avl = (lt + ld) / 2; // Average value of left sensors
  int avr = (rt + rd) / 2; // Average value of right sensors

  // ------------------------------------------------------------------------------
  // Error Values. Ideally, we want the difference between the top and bottom
  // (Vertical) and left and right (Horizontal) to be 0 (equal light = max light)
  // ------------------------------------------------------------------------------
  int dvert = avt - avd; // Difference between top and bottom: Vertical Error
  int dhori = avl - avr; // Difference between left and right: Horizontal Error
  
  
  // Scale and set PID input
  Input_Vert = (double)(dvert) / 10;  // Scale input down - typically around 600-700 due to ADC conversion (5/2^10)
  Input_Hori = (double)(dhori) / 10;  

  // Compute PID correction
  myPID_Vert.Compute();
  myPID_Hori.Compute();

  // Apply PID output to vertical servo position
  servovert += Output_Vert;
  servovert = constrain(servovert, servohoriLimitLow, servohoriLimitHigh); // Limit position to 30-150 degrees
  vertical.write(servovert);

  // Apply PID output to horizontal servo position
  servohori += Output_Hori;
  servohori = constrain(servohori, servohoriLimitLow, servohoriLimitHigh); // Limit position to 0-180 degrees
  horizontal.write(servohori);

  // -----------------------------
  // Serial CSV Logging (for Serial)
  // -----------------------------
  Serial.print(millis());         // Time (ms)
  Serial.print(",");
  Serial.print(Input_Vert);      // Vertical error (PID input)
  Serial.print(",");
  Serial.print(Output_Vert);     // Vertical PID output
  Serial.print(",");
  Serial.print(servovert);       // Vertical servo position
  Serial.print(",");
  Serial.print(Input_Hori);      // Horizontal error
  Serial.print(",");
  Serial.print(Output_Hori);     // Horizontal PID output
  Serial.print(",");
  Serial.println(servohori);     // Horizontal servo position

  delay(50); // Allow time for response, ~20 updates per second
}
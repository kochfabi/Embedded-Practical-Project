#include <Servo.h>

// Pins
const int sensorPIN = A0;
const int servoPIN  = 5;

// Servo
Servo myservo;

// PID variables
double Setpoint = 15.0;   // distance from sensor to center [in cm]
double Input;
double Output;

double errSum = 0.0;
double lastErr = 0.0;

// PID gains
double kp = 5.0;
double ki = 0; // 0.05;
double kd = 0; //0.2;

// Timing
const unsigned long Ts = 20;   // update interval in ms
unsigned long lastUpdate = 0;

void setup() {
  pinMode(sensorPIN, INPUT);
  myservo.attach(servoPIN);
  myservo.write(90);           // neutral position
  Serial.begin(9600);
}

void loop() {

  unsigned long now = millis();
  int angle = 20;
  if (now - lastUpdate >= Ts) {
    lastUpdate = now;

    // ---- SENSOR ----
    int raw = analogRead(sensorPIN);
    float voltage = raw * (5.0 / 1023.0);
    voltage = constrain(voltage, 0.45, 2.8);     // contrain voltage
    Input = 27.86 * pow(voltage, -1.15);

    // ---- PID ----
    double dt = Ts / 1000.0;   // seconds

    double error = Setpoint - Input;

    errSum += error * dt;
    errSum = constrain(errSum, -50, 50);  // anti-windup

    double dErr = (error - lastErr) / dt;
    lastErr = error;

    Output = kp * error + ki * errSum + kd * dErr;

    // ---- SERVO ----
    angle = constrain(90 + Output, 0, 180);


    // ---- DEBUG ----
    Serial.print("Dist: ");
    Serial.print(Input);
    Serial.print("  Out: ");
    Serial.print(Output);
    Serial.print("  Angle: ");
    Serial.println(angle);
  }
  
  myservo.write(angle);
}

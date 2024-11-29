#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1); // RX, TX

volatile int Front_Distance;
volatile boolean Flag = true;

const int Trig = A3;
const int Echo = A2;
const int PWM2A = 11; // M1 motor
const int PWM2B = 3;  // M2 motor
const int PWM0A = 6;  // M3 motor
const int PWM0B = 5;  // M4 motor
const int DIR_CLK = 4;
const int DIR_EN = 7;
const int DATA = 8;
const int DIR_LATCH = 12;

const int Move_Forward = 39;
const int Move_Backward = 216;
const int Right_Rotate = 149;
const int Left_Rotate = 106;
const int Stop = 0;

int Speed1 = 255;
int Speed2 = 255;
int Speed3 = 255;
int Speed4 = 255;

int number = 0;

bool runningUltrasonicAvoidance = false; // State variable
String direccion_actual = "arranca";  // Variable to store the current direction

unsigned long lastDistanceCheck = 0;    // To track time for ultrasonic checks
unsigned long distanceInterval = 100;  // Time between distance checks (ms)

unsigned long lastMotorAction = 0;     // To track time for motor actions
unsigned long motorDelay = 250;        // Delay between motor actions (ms)
bool motorDelayActive = false;         // Flag to manage motor delays

unsigned long lastDataSend = 0;        // To track time for data sending
unsigned long dataSendInterval = 500;  // Time between data sends (ms)

void Motor(int Dir, int Speed1, int Speed2, int Speed3, int Speed4) {
  analogWrite(PWM2A, Speed1);
  analogWrite(PWM2B, Speed2);
  analogWrite(PWM0A, Speed3);
  analogWrite(PWM0B, Speed4);
  digitalWrite(DIR_LATCH, LOW);
  shiftOut(DATA, DIR_CLK, MSBFIRST, Dir);
  digitalWrite(DIR_LATCH, HIGH);
  
  // Update direction state
  if (Dir == Move_Forward) { 
    direccion_actual = "adelante"; 
  } else if (Dir == Move_Backward) { 
    direccion_actual = "atrás"; 
  } else if (Dir == Left_Rotate) { 
    direccion_actual = "girando izquierda"; 
  } else if (Dir == Right_Rotate) { 
    direccion_actual = "girando derecha"; 
  } else if (Dir == Stop) { 
    direccion_actual = "detenido";
  }
}

float checkdistance() {
  static float lastDistance = 0; // Cache the last valid distance
  if (millis() - lastDistanceCheck >= distanceInterval) {
    lastDistanceCheck = millis();

    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);

    float distance = pulseIn(Echo, HIGH) / 58.00;

    // Ignore invalid readings
    if (distance > 0 && distance < 400) {
      lastDistance = distance;
    }
  }
  return lastDistance;
}

void mapEnvironment() {
  // Initialize variables for the map data
  int frontObstacle = 0, rightObstacle = 0, leftObstacle = 0;
  float frontDistance = 0, rightDistance = 0, leftDistance = 0;

  // Step 1: Check front
  frontDistance = checkdistance();
  frontObstacle = (frontDistance > 0 && frontDistance <= 35) ? 1 : 0;

  // Step 2: Turn right and check
  Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
  delay(500); // Allow enough time for a 90-degree turn
  rightDistance = checkdistance();
  rightObstacle = (rightDistance > 0 && rightDistance <= 35) ? 1 : 0;

  // Step 3: Turn left twice to check left side
  Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
  delay(1000); // Allow enough time for a 180-degree turn
  leftDistance = checkdistance();
  leftObstacle = (leftDistance > 0 && leftDistance <= 35) ? 1 : 0;

  // Step 4: Return to original orientation
  Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
  delay(500); // Return to the original position
  Motor(Stop, 0, 0, 0, 0);

  // Step 5: Send data to Python
  mySerial.print("{\"Front\": {\"Obstacle\": ");
  mySerial.print(frontObstacle);
  mySerial.print(", \"Distance\": ");
  mySerial.print(frontDistance);
  mySerial.print("}, \"Right\": {\"Obstacle\": ");
  mySerial.print(rightObstacle);
  mySerial.print(", \"Distance\": ");
  mySerial.print(rightDistance);
  mySerial.print("}, \"Left\": {\"Obstacle\": ");
  mySerial.print(leftObstacle);
  mySerial.print(", \"Distance\": ");
  mySerial.print(leftDistance);
  mySerial.println("}}");
  Serial.println(frontObstacle); // Debug print
  Serial.println(frontDistance); // Debug print
  Serial.println(rightObstacle); // Debug print
  Serial.println(rightDistance); // Debug print
  Serial.println(leftObstacle); // Debug print
  Serial.println(leftDistance); // Debug print



}

void executeCommand(String command) {

  //number = 1;
  if (command == "F") {
    Motor(Move_Forward, Speed1, Speed2, Speed3, Speed4);
    delay(1000); // Adjust based on desired distance
  } else if (command == "B") {
    Motor(Move_Backward, Speed1, Speed2, Speed3, Speed4);
    delay(250);
  } else if (command == "L") {
    Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
    delay(250); // 90-degree turn
  } else if (command == "R") {
    Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
    delay(1000);
  } else if (command == "Stop") {
    Motor(Stop, 0, 0, 0, 0);
  }
  
  Motor(Stop, 0, 0, 0, 0); // Ensure stop after movement
  mapEnvironment();       // Re-map after executing the command
}

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);

  pinMode(DIR_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(DIR_EN, OUTPUT);
  pinMode(DIR_LATCH, OUTPUT);
  pinMode(PWM0B, OUTPUT);
  pinMode(PWM0A, OUTPUT);
  pinMode(PWM2A, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop() {
  
  if(number == 1){
      Motor(Move_Forward, Speed1, Speed2, Speed3, Speed4);
      delay(250); // Adjust based on desired distance
  }
  if (mySerial.available() > 0) {
    String command = mySerial.readStringUntil('\n'); // Read the command from Python
    command.trim(); // Remove any extra whitespace or newlines
    Serial.print("Command: ");
    Serial.println(command); // Debug print
    executeCommand(command); // Execute the command
  }
}
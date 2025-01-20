#include <Servo.h>

#define IN1 9
#define IN2 8
#define IN3 10
#define IN4 11
#define ENA 5
#define ENB 6
#define trigPin 12
#define echoPin 7
#define servoPin 3

Servo myServo;

long duration;
int distance;
int distanceRight;
int distanceLeft;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);
  myServo.write(90); // Center position

}

void loop() {
  moveForward();
  distance = measureDistance();
  Serial.println("Distance:");
  Serial.println(distance);
  if (distance < 40) {
    stopCar();
    moveBackward(500); // Move back for 500ms (adjust time as needed)
    stopCar();

    // Check right distance
    myServo.write(0); // Turn servo to the right
    delay(500); // Wait for servo to reach position
    distanceRight = measureDistance();

    // Check left distance
    myServo.write(180); // Turn servo to the left
    delay(500); // Wait for servo to reach position
    distanceLeft = measureDistance();

    // Decide direction
    if (distanceRight > distanceLeft) {
      turnRight();
    } else {
      turnLeft();
    }

    delay(500); // Wait for the turn to complete
    stopCar();

    // Center the servo
    myServo.write(90);
    delay(500); // Wait for servo to reach center position
  }
  delay(100);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); // Full speed
  analogWrite(ENB, 255); // Full speed
}

void moveBackward(int duration) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); // Full speed
  analogWrite(ENB, 255); // Full speed
  delay(duration);
}
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); // Full speed
  analogWrite(ENB, 255); // Full speed
  delay(500); // Adjust time to achieve the desired turn
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); // Full speed
  analogWrite(ENB, 255); // Full speed
  delay(500); // Adjust time to achieve the desired turn
}

int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.017; // Calculate the distance in cm
  return distance;
}

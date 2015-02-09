int threshold = 300;
int frontThreshold = 200;

int prevFront = 0;
int prevRight = 0;
int prevLeft  = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  int front = analogRead(A0);
  int right = analogRead(A2);
  int left  = analogRead(A4);

  if (Serial.available() > 0) {
    char query = Serial.read();
    if (query == 'f') {              // Front sensor (averaged over two readings)
      Serial.write((front+prevFront)/2);
    } else if (query == 'r') {       // Right sensor (averaged over two readings)
      Serial.write((right+prevRight)/2);
    } else if (query == 'l') {       // Left  sensor (averaged over two readings)
      Serial.write((left+prevLeft)/2);
    } else {                         // Default to normal warning
      if (front > frontThreshold && prevFront > frontThreshold) {
          if (front > threshold && prevFront > threshold) {
            Serial.print('F');
          } else {
            Serial.print('f');
          }
      } else if (right > threshold && prevRight > threshold) {
          Serial.print('R');
      } else if (left > threshold && prevLeft > threshold) {
          Serial.print('L');
      } else {
        Serial.print('*');
      }
    }
  }

  prevFront = front;
  prevRight = right;
  prevLeft  = left;
}




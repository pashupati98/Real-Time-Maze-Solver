#define sensor_pins0 A0
#define sensor_pins1 A1
#define sensor_pins2 A2
#define sensor_pins3 A3
#define sensor_pins4 A4

#define left_dir 7
#define left_pwm 10

#define right_dir 4
#define right_pwm 9

const int pingPin = 8;
const int echoPin = 2;


int analog_sensor_data[5] = {0, 0, 0, 0, 0};
int sensor_treshhold[5] = {50, 100, 35, 100, 100};
int sensor_val[5] = {0, 0, 0, 0, 0};
int sensor_val1[5] = {0, 0, 0, 0, 0};
int digital_error = 0, prev_digital_error = 0;
int sensor_num = 0;

int l = 1, r = 2, s = 3, b = 6, e = 5;
int p = 0;

float kp = 15.0;
float kd = 0.0;
float ki = 0.0;

float correction = 0.0;

float error = 0.0;
float d_error = 0.0;
float i_error = 0.0;
int n = 0;
int m = 0;
void sensor_read() {
  analog_sensor_data[0] = analogRead(sensor_pins0);
  analog_sensor_data[1] = analogRead(sensor_pins1);
  analog_sensor_data[2] = analogRead(sensor_pins2);
  analog_sensor_data[3] = analogRead(sensor_pins3);
  analog_sensor_data[4] = analogRead(sensor_pins4);

  for (int i = 0; i < 5; i++) {
    if (analog_sensor_data[i] > sensor_treshhold[i]) {
      sensor_val[i] = 1;
    }
    else {
      sensor_val[i] = 0;
    }

  }

}
void sensor_read1() {
  analog_sensor_data[0] = analogRead(sensor_pins0);
  analog_sensor_data[1] = analogRead(sensor_pins1);
  analog_sensor_data[2] = analogRead(sensor_pins2);
  analog_sensor_data[3] = analogRead(sensor_pins3);
  analog_sensor_data[4] = analogRead(sensor_pins4);

  for (int i = 0; i < 5; i++) {
    if (analog_sensor_data[i] > sensor_treshhold[i]) {
      sensor_val1[i] = 1;
    }
    else {
      sensor_val1[i] = 0;
    }
  }
}


void junction_shift( int lpwm, int rpwm) {
  digitalWrite(left_dir, 0);
  digitalWrite(right_dir, 0);

  analogWrite(left_pwm, lpwm);
  analogWrite(right_pwm, rpwm);

  delay(520);

  analogWrite(left_pwm, 0);
  analogWrite(right_pwm, 0);
  delay(1000);

}

void right_turn() {
  digitalWrite(left_dir, 0);
  digitalWrite(right_dir, 1);

  analogWrite(left_pwm, 50);
  analogWrite(right_pwm, 50);
}
void left_turn() {
  digitalWrite(left_dir, 1);
  digitalWrite(right_dir, 0);

  analogWrite(left_pwm, 65);
  analogWrite(right_pwm, 65);
}
void u_turn() {
  digitalWrite(left_dir, 1);
  digitalWrite(right_dir, 0);

  analogWrite(left_pwm, 50);
  analogWrite(right_pwm, 50);
}


void motor_run(int ldir, int rdir, int lpwm, int rpwm) {
  digitalWrite(left_dir, ldir);
  digitalWrite(right_dir, rdir);

  analogWrite(left_pwm, lpwm + correction);
  analogWrite(right_pwm, rpwm - correction);
  Serial.print(lpwm + correction);
  Serial.print("  ");
  Serial.println(rpwm - correction);
}

void PID() {
  error = digital_error;
  correction = kp * error + kd * d_error + ki * i_error;
}
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(sensor_pins0, INPUT);
  pinMode(sensor_pins1, INPUT);
  pinMode(sensor_pins2, INPUT);
  pinMode(sensor_pins3, INPUT);
  pinMode(sensor_pins4, INPUT);

  pinMode(left_dir, OUTPUT);
  pinMode(left_pwm, OUTPUT);

  pinMode(right_dir, OUTPUT);
  pinMode(right_pwm, OUTPUT);

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {

  long duration, inches, cm;

  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  duration = pulseIn(echoPin, HIGH);

  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  //  Serial.print(inches);
  //  Serial.print("in, ");
  //  Serial.print(cm);
  //  Serial.print("cm");
  //  Serial.println();

  delay(100);

  if (cm <= 8)
  {

    sensor_val[0] = 0;
    while (sensor_val[0] != 1) {
      u_turn();
      sensor_read();
    }
  }


  sensor_read();
  //  Serial.print(sensor_val[0]);
  //  Serial.print("  ");
  //  Serial.print(sensor_val[1]);
  //  Serial.print("  ");
  //  Serial.print(sensor_val[2]);
  //  Serial.print("  ");
  //  Serial.print(sensor_val[3]);
  //  Serial.print("  ");
  //  Serial.print(sensor_val[4]);
  //  Serial.print("  ");
  //  Serial.print("\n");
  prev_digital_error = digital_error;
  digital_error = 0;
  sensor_num = 0;

  for (int i = 0; i < 5; i++) {
    if (sensor_val[i] == 1) {
      digital_error += (2 * i - 4);
      sensor_num++;
    }
  }

  if (sensor_num != 0) {
    digital_error = digital_error / sensor_num;
  }
  else {
    digital_error = prev_digital_error;
  }

  PID();
  motor_run(0, 0, 80, 80);

  if (sensor_num > 2) {
    junction_shift(80, 80);
    sensor_read1();
    if ((sensor_val1[0] == 1) && (sensor_val1[1] == 1) && (sensor_val1[2] == 1) && (sensor_val1[3] == 1) && (sensor_val1[4] == 1))
    {

      digitalWrite(12, HIGH);
      digitalWrite(13, HIGH);

      //Serial.print("end point");

      delay(4000);
      
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);

    }

    
    if (sensor_val[4] == 1)
    {
     

      sensor_val[4] = 0;
      right_turn();
      delay(300);
      do {
        right_turn();
        sensor_read();
      } while (sensor_val[4] != 1);

    analogWrite(left_pwm, 0);
    analogWrite(right_pwm, 0);
    delay(300);

    }

    else if (sensor_val[0] == 1)
    {
      sensor_read();
      if ( sensor_val[0] == 1 || sensor_val[1] == 1 || sensor_val[2] == 1 || sensor_val[3] == 1 || sensor_val[4] == 1)
      {
        sensor_read();
        motor_run(0, 0, 80, 80);
      }
      else {
       
        sensor_val[0] = 0;
        left_turn();
        delay(300);
        do {
          left_turn();
          sensor_read();
        } while (sensor_val[0] != 1);
      
      }
      analogWrite(left_pwm, 0);
    analogWrite(right_pwm, 0);
    delay(300);

    }
    
    

  }
  else if (sensor_num == 0 && ((digital_error == 0) || (digital_error == -1) || (digital_error == 1) || (digital_error == 2) || (digital_error == -2) || (digital_error == -3) || (digital_error == 3)))
  {
    
    
    junction_shift(100, 100);
    sensor_val[2] = 0;
    while (sensor_val[2] != 1) {
      u_turn();
      sensor_read();
    }
//    analogWrite(left_pwm, 0);
//    analogWrite(right_pwm, 0);
//    delay(1000);
//    
    
  }
}

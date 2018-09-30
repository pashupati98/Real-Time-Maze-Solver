#define sensor_pins0 A0
#define sensor_pins1 A1
#define sensor_pins2 A2
#define sensor_pins3 A3
#define sensor_pins4 A4

#define left_dir 7
#define left_pwm 10

#define right_dir 4
#define right_pwm 9
int flag = 0;
int analog_sensor_data[5] = {0, 0, 0, 0, 0};
int sensor_treshhold[5] = {50, 100, 35, 100, 100};
int sensor_val[5] = {0, 0, 0, 0, 0};
int sensor_val1[5] = {0, 0, 0, 0, 0};
int digital_error = 0, prev_digital_error = 0;
int sensor_num = 0;
int learn[100];
int l = 1, r = 2, s = 3, b = 6, e = 5;
int p = 0;
int follow[100];
float kp = 20.0;
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

  delay(580);

  analogWrite(left_pwm, 0);
  analogWrite(right_pwm, 0);
  delay(1000);

  //Serial.print("\t");
  //Serial.print("juncCrossed");
}

void right_turn() {
  digitalWrite(left_dir, 0);
  digitalWrite(right_dir, 1);

  analogWrite(left_pwm, 100);
  analogWrite(right_pwm, 100);
}
void left_turn() {
  digitalWrite(left_dir, 1);
  digitalWrite(right_dir, 0);

  analogWrite(left_pwm, 100);
  analogWrite(right_pwm, 100);
}
void u_turn() {
  digitalWrite(left_dir, 1);
  digitalWrite(right_dir, 0);

  analogWrite(left_pwm, 100);
  analogWrite(right_pwm, 100);
}


void motor_run(int ldir, int rdir, int lpwm, int rpwm) {
  digitalWrite(left_dir, ldir);
  digitalWrite(right_dir, rdir);

  analogWrite(left_pwm, lpwm + correction);
  analogWrite(right_pwm, rpwm - correction);
}

void PID() {
  //  if(abs(digital_error) == 0){
  //    error =
  //  }
  //  else if(abs(digital_error) == 1){
  //
  //  }
  //  else if(abs(digital_error) == 2){
  //
  //  }
  //  else if(abs(digital_error) == 3){
  //
  //  }
  //  else if(abs(digital_error) == 4){
  //
  //  }
  //  else if(abs(digital_error) == 5){
  //
  //  }
  error = digital_error;
  correction = kp * error + kd * d_error + ki * i_error;
}
void shortest_path()
{
  int i;
  int j = 0;
  n = m;
  
 
    
  for (i = 0; i < n - 1; i++)
  {

    if (learn[i] == b);
    {
      if ( learn[i - 1] == l && learn[i + 1] == r)

      {
        learn[i] = b;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
        
      }
      if ( learn[i - 1] == l &&  learn[i + 1] == s)
      {
        learn[i] = r;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
      if ( learn[i - 1] == r &&  learn[i + 1] == l)
      {
        learn[i] = b;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
      if ( learn[i - 1] == s &&  learn[i + 1] == l)
      {
        learn[i] = r;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
      if ( learn[i - 1] == s && learn[i + 1] == s)
      {
        learn[i] = b;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
      if ( learn[i - 1] == l &&  learn[i + 1] == l)
      {
        learn[i] = s;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
       if ( learn[i - 1] == r &&  learn[i + 1] == s)
      {
        learn[i] = l;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
      if ( learn[i - 1] == r &&  learn[i + 1] == r)
      {
        learn[i] = s;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }        
      if ( learn[i - 1] == s &&  learn[i + 1] == r)
      {
        learn[i] = l;
        learn[i+1]=0;
        learn[i-1]=0;
        m-=2;
      }
    }

  }    
    
    for(int i=0;i<n;i++)
    {
      if(learn[i]!=0)
      {
       follow[j]=learn[i];
       j++; 
      }
    }
    for(int i=0;i<n;i++)
    {
      learn[i]=0;

    }
    for(i=0;i<j;i++)
    {
      learn[i]=follow[i];
    }
      
  
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(A5,OUTPUT);
  digitalWrite(A5,LOW);
  digitalWrite(3, HIGH);
  pinMode(2, INPUT);
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

}

void loop() {
  // put your main code here, to run repeatedly:


  if (digitalRead(2) == HIGH)
  {
    sensor_read();
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
    motor_run(0, 0, 100, 100);

    if (sensor_num > 2) {
      //Serial.print("\t");
      //Serial.print("foundJunc");
      junction_shift(100, 100);
      
     sensor_read1();
    if ((sensor_val1[0] == 1) && (sensor_val1[1] == 1) && (sensor_val1[2] == 1) && (sensor_val1[3] == 1) && (sensor_val1[4] == 1))
      {
        digitalWrite(12, HIGH);
        digitalWrite(13, HIGH);
        
        Serial.print("end point");
        
        delay(5000);
        digitalWrite(3,LOW);
        digitalWrite(12, LOW);
        digitalWrite(13, LOW);

      }

 
      if (sensor_val[4] == 1)
      {

        //Serial.print("\t");
        //Serial.print("right_turn");
        sensor_val[4] = 0;
        right_turn();
        delay(300);
        do {
          right_turn();
          sensor_read();
        } while (sensor_val[4] != 1);
        learn[m] = r;
        Serial.print( learn[m]);
        Serial.print(" ");
        m++;
      }

      else if (sensor_val[0] == 1)
      {
        sensor_read();
        if ( sensor_val[0] == 1 || sensor_val[1] == 1 || sensor_val[2] == 1 || sensor_val[3] == 1 || sensor_val[4] == 1)
        {
          learn[m] = s;
          Serial.print( learn[m]);
          Serial.print(" ");
          m++;
          sensor_read();
          motor_run(0, 0, 100, 100);
        }
        else {
          //Serial.print("\t");
          //Serial.print("left_turn");
          learn[m] = l;
          Serial.print( learn[m]);
          Serial.print(" ");
          m++;
          sensor_val[0] = 0;
          left_turn();
          delay(300);
          do {
            left_turn();
            sensor_read();
          } while (sensor_val[0] != 1);
        }
      }

      //Serial.print("\t");
      //Serial.print("finish_turn");
      analogWrite(left_pwm, 0);
      analogWrite(right_pwm, 0);
      delay(1000);
    }
    else if (sensor_num == 0 && ((digital_error == 0) || (digital_error == -1) || (digital_error == 1) || (digital_error == 2) || (digital_error == -2) || (digital_error == -3) || (digital_error == 3)))
    {
      //Serial.print("WTf");
      learn[m] = b;
      Serial.print(learn[m]);
      Serial.print(" ");
      m++;
      junction_shift(100, 100);
      sensor_val[2] = 0;
      while (sensor_val[2] != 1) {
        u_turn();
        sensor_read();
      }
    }



    //
    //  Serial.print("\t");
    //  Serial.print(sensor_num);
    //  Serial.print("\t");
    //Serial.print(digital_error);
    //Serial.print("\n");
    //  Serial.print(sensor_val[0]);
    //  Serial.print("\t");
    //  Serial.print(sensor_val[1]);
    //  Serial.print("\t");
    //  Serial.print(sensor_val[2]);
    //  Serial.print("\t");
    //  Serial.print(sensor_val[3]);
    //  Serial.print("\t");
    //  Serial.print(sensor_val[4]);
    //  Serial.println(" ");
    //   for(int k=0 ; k<100 ; k++){
    //    Serial.print(learn[k]);
    //    Serial.print("\n");
    //   }
   
  }

  else if (digitalRead(2) == LOW)

  {
    for(int i=0;i<m;i++)
    {
      if(learn[i]==b)
      shortest_path();
      
    }
    for( int k=0;k<m;k++)
    {
      Serial.print(learn[k]);
      Serial.print("  ");
       
    }
    sensor_read();
    prev_digital_error = digital_error;
    digital_error = 0;
    sensor_num = 0;

    for (int i = 0; i < 5; i++) {
      if (sensor_val[i] == 1) {
        digital_error += 2 * i - 4;
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
    motor_run(0, 0, 100, 100);

    if (sensor_num > 2) {

      //Serial.print("\t");
      //Serial.print("foundJunc");
      junction_shift(100, 100);

      if (learn[p] == r)
      {
        p++;
        Serial.print("\t");
        Serial.print("right_turn");
        sensor_val[4] = 0;
        while (sensor_val[4] != 1) {
          right_turn();
          sensor_read();
        }
      }

      else if (learn[p] == l)
      {
        p++;
        sensor_val[0] = 0;
        while (sensor_val[0] != 1) {
          left_turn();
          sensor_read();
        }
      }
      else if ( learn[p] == s)
      {
        p++;
        sensor_read();
        motor_run(0, 0, 100, 100);
      }

      analogWrite(left_pwm, 0);
      analogWrite(right_pwm, 0);
      delay(2000);
    }

  }
}











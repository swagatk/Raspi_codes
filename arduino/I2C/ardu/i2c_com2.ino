#include <Wire.h>

#define SLAVE_ADDRESS 0x08
int number = 0;
int state = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  
  // initialize i2c slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
 delay(100);

}

// callback for received Data
void receiveData(int byteCount){
  while(Wire.available()){
    number = Wire.read();
    Serial.print("data received:");
    Serial.println(number);

    if (number == 1){
      if (state == 0){
        digitalWrite(13, HIGH); 
        state = 1;
      }
      else{
        digitalWrite(13, LOW);
        state = 0;
      }
    }
  }
}

// callback for sending data
void sendData(){
  Wire.write(number);
}

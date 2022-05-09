#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64.h"

/*
#include "PID_v1.h"
double Input, Output, Setpoint;
double Kp=1, Ki=0, Kd=0;
PID rfPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
*/

double kp = 0.00400;
double ki = 0.00400;
double kd = 0.00010;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, Setpoint;
double cumError, rateError;

double computePID(double inp);


#define ARRAY_LEN 4

char inc_char;
String rpmString;
float unmapped_rpm_command_array[ARRAY_LEN];
float mapped_rpm_command_array[ARRAY_LEN];

float unmapped_current_command_array[ARRAY_LEN];
float mapped_current_command_array[ARRAY_LEN];

bool torque_mode_flag;

unsigned long prevMillis;

std_msgs::Float64 rpm_data;
ros::NodeHandle nh;
ros::Publisher rpmPub("rpm_rover_topic", &rpm_data);

HardwareSerial RBSerial(PA3, PA2);
HardwareSerial RFSerial(PB11, PB10);
HardwareSerial LBSerial(PC11, PA0);
HardwareSerial LFSerial(PD2, PC12);
HardwareSerial motherBoardSerial(PA10, PA9);

VescUart RBmotor;
VescUart RFmotor;
VescUart LBmotor;
VescUart LFmotor;

void readNdrive(void);
void assignRpmArray(String rpmStr);
void assignCurrentArray(String currentStr);
void mapData(void);
void mapCurrent(void);

char getDir(int x);
int unmapRpm(void);
void createFeedbackMsg(int a, int b, int c, int d);

void setup() {
  
  

  RBSerial.begin(115200);
  RFSerial.begin(115200);
  LBSerial.begin(115200);
  LFSerial.begin(115200);

  motherBoardSerial.begin(9600);

  while(!RBSerial){;}
  RBmotor.setSerialPort(&RBSerial);
  
  while(!RFSerial){;}
  RFmotor.setSerialPort(&RFSerial);
  

   while(!LBSerial){;}
  LBmotor.setSerialPort(&LBSerial);

  while(!LFSerial){;}
  LFmotor.setSerialPort(&LFSerial);

  
  RBmotor.setRPM(0);
  RFmotor.setRPM(0);
  LBmotor.setRPM(0);
  LFmotor.setRPM(0);

  //rfPID.SetMode(AUTOMATIC);
  
}

void loop() {
  
  
  readNdrive();
  
    if(!torque_mode_flag){
      
      /*
      Input=RFmotor.data.rpm;
      Setpoint=mapped_rpm_command_array[2];
      rfPID.Compute();
      */
      
      
      /*
      LFmotor.setDuty(mapped_rpm_command_array[0]);
      LBmotor.setDuty(mapped_rpm_command_array[0]);
      */
      
      input = RFmotor.data.rpm;                //read from rotary encoder connected to A0
      Setpoint=mapped_rpm_command_array[2];
      output = computePID(input);
      delay(2);
      
      RBmotor.setRPM(output);
      RFmotor.setRPM(output);
      
      
      /*
      RBmotor.setRPM(mapped_rpm_command_array[2]);
      RFmotor.setRPM(mapped_rpm_command_array[2]);
      */
    }
    
    if(torque_mode_flag){
      LFmotor.setCurrent(mapped_current_command_array[0]);
      LBmotor.setCurrent(mapped_current_command_array[0]);
      RBmotor.setCurrent(mapped_current_command_array[2]);
      RFmotor.setCurrent(mapped_current_command_array[2]);
    }
}

void readNdrive(void){
  static bool receive_flag=false;
  inc_char=motherBoardSerial.read();
  delay(2);

  if(motherBoardSerial.available()>0){
    if(inc_char=='S'){
      rpmString="";
      receive_flag=true;
    }
    if(receive_flag && inc_char!='S' && inc_char!='F'){
      rpmString+=inc_char;
    }
    if(inc_char=='F'){
      
      if(rpmString[16]=='0'){
        torque_mode_flag=false;
      }  

      if(rpmString[16]=='1'){
        torque_mode_flag=true;
      }  
    
      assignRpmArray(rpmString);
      assignCurrentArray(rpmString);

      mapData();
      mapCurrent();

      


     if(RBmotor.getVescValues() && RFmotor.getVescValues()){
        createFeedbackMsg(int(500), int(500), int(RBmotor.data.rpm), int(RFmotor.data.rpm));     
        //motherBoardSerial.println('A'+String(LFmotor.data.rpm)+'B'+String(LBmotor.data.rpm)+'C');
      }

      receive_flag=false;
      rpmString="";
    }
  }
}

void assignRpmArray(String rpmStr){
  String str_buffer;
  char direction_char;
  int direction;
  for(int i=0;i<ARRAY_LEN;i++){
    direction_char=rpmStr[4*i];
    if(direction_char=='0'){
      direction=-1;
    }
    if(direction_char=='1'){
      direction=1;
    }
    for(int j=i*4;j<(i*4)+3;j++){
      str_buffer+=rpmStr[j+1];    
    }
    unmapped_rpm_command_array[i]=direction*str_buffer.toInt();
    str_buffer="";
  }
}

void assignCurrentArray(String currentStr){
  String str_buffer;
  char direction_char;
  int direction;
  for(int i=0;i<ARRAY_LEN;i++){
    direction_char=currentStr[4*i];
    if(direction_char=='0'){
      direction=-1;
    }
    if(direction_char=='1'){
      direction=1;
    }
    for(int j=i*4;j<(i*4)+3;j++){
      str_buffer+=currentStr[j+1];    
    }
    unmapped_current_command_array[i]=direction*str_buffer.toInt();
    str_buffer="";  
  }
}

void mapData(void){
  
  for(int i=0;i<ARRAY_LEN;i++){
    if(unmapped_rpm_command_array[i]<=0){
      mapped_rpm_command_array[i]=((unmapped_rpm_command_array[i]+999)*10000/999)-10000;
    }
    if(unmapped_rpm_command_array[i]>0){
      mapped_rpm_command_array[i]=unmapped_rpm_command_array[i]*10000/999;
    }
  }
  
  
  /*
  for(int i=0;i<ARRAY_LEN;i++){
    if(unmapped_rpm_command_array[i]<=0){
      mapped_rpm_command_array[i]=((unmapped_rpm_command_array[i]+999)*1/999)-1;
    }
    if(unmapped_rpm_command_array[i]>0){
      mapped_rpm_command_array[i]=unmapped_rpm_command_array[i]*1/999;
    }
  }
  */
}

void mapCurrent(void){
  for(int i=0;i<ARRAY_LEN;i++){
    if(unmapped_current_command_array[i]<=0){
      mapped_current_command_array[i]=((unmapped_current_command_array[i]+999)*6/999)-6;
    }
    if(unmapped_current_command_array[i]>0){
      mapped_current_command_array[i]=unmapped_current_command_array[i]*6/999;
    }
  }
}

char getDir(int x){
  char direction;
  if(x>0){
    direction='1';
  }
  if(x<=0){
    direction='0';
  }
  return direction;
}

int unmapRpm(int x){
  int unmapped_data;
  if(x>0){
    unmapped_data=x*999/10500;
  }
  if(x<=0){
    unmapped_data=((x+10500)*999/10500)-999;
  }
  if(unmapped_data>999){
    unmapped_data=999;
  }
  return unmapped_data;
}

void createFeedbackMsg(int a, int b, int c, int d){
  String sentString="A";
  
  sentString+=getDir(a);
  String processedStringLF = String(abs(a));
  while(processedStringLF.length()<5){
    processedStringLF = "0" + processedStringLF;
  }
  sentString+=processedStringLF;
  
  sentString+=getDir(b);
  String processedStringLB = String(abs(b));
  while(processedStringLB.length()<5){
    processedStringLB = "0" + processedStringLB;
  }
  sentString+=processedStringLB;

  sentString+=getDir(c);
  String processedStringRB = String(abs(c));
  while(processedStringRB.length()<5){
    processedStringRB = "0" + processedStringRB;
  }
  sentString+=processedStringRB;
  
  sentString+=getDir(d);
  String processedStringRF = String(abs(d));
  while(processedStringRF.length()<5){
    processedStringRF = "0" + processedStringRF;
  }
  sentString+=processedStringRF;
  
  sentString+="B";
  motherBoardSerial.println(sentString);
}

double computePID(double inp){     
        currentTime = millis();               
        elapsedTime = (double)(currentTime - previousTime);       
        
        error = Setpoint - inp;                                
        cumError += error * elapsedTime;               
        rateError = (error - lastError)/elapsedTime;   

        double out = kp*error + ki*cumError + kd*rateError;                             

        lastError = error;                                
        previousTime = currentTime;                        

        return out;                                        
}

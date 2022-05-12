#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64.h"


double lf_kp = 0.00400;
double lf_ki = 0.00400;
double lf_kd = 0.00010;
unsigned long lf_currentTime, lf_previousTime;
double lf_elapsedTime;
double lf_error;
double lf_lastError;
double lf_input, lf_output, lf_Setpoint;
double lf_cumError, lf_rateError;
double lf_computePID(double lf_inp);


double lb_kp = 0.00400;
double lb_ki = 0.00400;
double lb_kd = 0.00010;
unsigned long lb_currentTime, lb_previousTime;
double lb_elapsedTime;
double lb_error;
double lb_lastError;
double lb_input, lb_output, lb_Setpoint;
double lb_cumError, lb_rateError;
double lb_computePID(double lb_inp);


double rb_kp = 0.00400;
double rb_ki = 0.00400;
double rb_kd = 0.00010;
unsigned long rb_currentTime, rb_previousTime;
double rb_elapsedTime;
double rb_error;
double rb_lastError;
double rb_input, rb_output, rb_Setpoint;
double rb_cumError, rb_rateError;
double rb_computePID(double rb_inp);


double rf_kp = 0.00400;
double rf_ki = 0.00400;
double rf_kd = 0.00010;
unsigned long rf_currentTime, rf_previousTime;
double rf_elapsedTime;
double rf_error;
double rf_lastError;
double rf_input, rf_output, rf_Setpoint;
double rf_cumError, rf_rateError;
double rf_computePID(double rf_inp);






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
}

void loop() {

  readNdrive();
  
    if(!torque_mode_flag){
      
      lf_input = LFmotor.data.dutyCycleNow;            
      lf_Setpoint=mapped_rpm_command_array[0];
      lf_output = lf_computePID(lf_input);
    
      lb_input = LBmotor.data.dutyCycleNow;            
      lb_Setpoint=mapped_rpm_command_array[0];
      lb_output = lb_computePID(lb_input);

      rb_input = RBmotor.data.dutyCycleNow;            
      rb_Setpoint=mapped_rpm_command_array[2];
      rb_output = rb_computePID(rb_input);

      rf_input = RFmotor.data.dutyCycleNow;            
      rf_Setpoint=mapped_rpm_command_array[2];
      rf_output = rf_computePID(rf_input);


      LFmotor.setDuty(lf_output);
      LBmotor.setDuty(lb_output);
      RBmotor.setDuty(rb_output);
      RFmotor.setDuty(rf_output);
      

      /*
      LFmotor.setRPM(mapped_rpm_command_array[0]);
      LBmotor.setRPM(mapped_rpm_command_array[0]);
      RBmotor.setRPM(mapped_rpm_command_array[2]);
      RFmotor.setRPM(mapped_rpm_command_array[2]);
      */
    
    }
    
    if(torque_mode_flag){
      LFmotor.setCurrent(mapped_current_command_array[0]);
      LBmotor.setCurrent(mapped_current_command_array[1]);
      RBmotor.setCurrent(mapped_current_command_array[2]);
      RFmotor.setCurrent(mapped_current_command_array[3]);

    }
    
  //delayMicroseconds(2000);
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
    

     if(LFmotor.getVescValues() && LBmotor.getVescValues() && RBmotor.getVescValues() && RFmotor.getVescValues()){
        createFeedbackMsg(int(LFmotor.data.rpm), int(LBmotor.data.rpm), int(RBmotor.data.rpm), int(RFmotor.data.rpm));     
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
  /*
  for(int i=0;i<ARRAY_LEN;i++){
    if(unmapped_rpm_command_array[i]<=0){
      mapped_rpm_command_array[i]=(((unmapped_rpm_command_array[i]+999)*8400/999)-8400);
    }
    if(unmapped_rpm_command_array[i]>0){
      mapped_rpm_command_array[i]=(unmapped_rpm_command_array[i]*8400/999);
    }
  }
  */
  
  
  for(int i=0;i<ARRAY_LEN;i++){
    if(unmapped_rpm_command_array[i]<=0){
      mapped_rpm_command_array[i]=((unmapped_rpm_command_array[i]+999)*0.70/999)-0.70;
    }
    if(unmapped_rpm_command_array[i]>0){
      mapped_rpm_command_array[i]=unmapped_rpm_command_array[i]*0.70/999;
    }
  }
  
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
    unmapped_data=x*999/10000;
  }
  if(x<=0){
    unmapped_data=((x+10000)*999/10000)-999;
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

double lf_computePID(double lf_inp){     
        lf_currentTime = millis();               
        lf_elapsedTime = (double)(lf_currentTime - lf_previousTime);       
        
        lf_error = lf_Setpoint - lf_inp;                                
        lf_cumError += lf_error * lf_elapsedTime;               
        lf_rateError = (lf_error - lf_lastError)/lf_elapsedTime;   

        double lf_out = lf_kp*lf_error + lf_ki*lf_cumError + lf_kd*lf_rateError;                             

        lf_lastError = lf_error;                                
        lf_previousTime = lf_currentTime;                        

        return lf_out;                                        
}

double lb_computePID(double lb_inp){     
        lb_currentTime = millis();               
        lb_elapsedTime = (double)(lb_currentTime - lb_previousTime);       
        
        lb_error = lb_Setpoint - lb_inp;                                
        lb_cumError += lb_error * lb_elapsedTime;               
        lb_rateError = (lb_error - lb_lastError)/lb_elapsedTime;   

        double lb_out = lb_kp*lb_error + lb_ki*lb_cumError + lb_kd*lb_rateError;                             

        lb_lastError = lb_error;                                
        lb_previousTime = lb_currentTime;                        

        return lb_out;                                        
}

double rb_computePID(double rb_inp){     
        rb_currentTime = millis();               
        rb_elapsedTime = (double)(rb_currentTime - rb_previousTime);       
        
        rb_error = rb_Setpoint - rb_inp;                                
        rb_cumError += rb_error * rb_elapsedTime;               
        rb_rateError = (rb_error - rb_lastError)/rb_elapsedTime;   

        double rb_out = rb_kp*rb_error + rb_ki*rb_cumError + rb_kd*rb_rateError;                             

        rb_lastError = rb_error;                                
        rb_previousTime = rb_currentTime;                        

        return rb_out;                                        
}

double rf_computePID(double rf_inp){     
        rf_currentTime = millis();               
        rf_elapsedTime = (double)(rf_currentTime - rf_previousTime);       
        
        rf_error = rf_Setpoint - rf_inp;                                
        rf_cumError += rf_error * rf_elapsedTime;               
        rf_rateError = (rf_error - rf_lastError)/rf_elapsedTime;   

        double rf_out = rf_kp*rf_error + rf_ki*rf_cumError + rf_kd*rf_rateError;                             

        rf_lastError = rf_error;                                
        rf_previousTime = rf_currentTime;                        

        return rf_out;                                        
}



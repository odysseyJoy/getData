const int vcc = 52;               /*定义HC-SR04的相关管脚*/
const int trig = 50;
const int echo = 48;
const int gnd = 46;

const int tempPin = A0;           /*定义LM35CZ的信号输出管脚*/            
const int waterTempPin = A1; 

const int RGBVCC = 22;            /*定义RGB的管脚*/
const int R = 10;
const int B = 9;
const int G = 8;

const int buzzerIn = 23;          /*定义有源蜂鸣器的管脚*/
const int buzzerVCC = 25;
const int buzzerGND = 24;

const int levelIn = A2;           /*定义最低水位检测装置的管脚*/
float level = 500;

float echoTime;                   /*定义超声波测液位相关的变量*/
float filter_echoTime;            /*echoTime为回波时间，filter_echoTime为滤波后的echoTime*/
float fixed_distance;             /*waterLevel为液位，V为水的体积*/
float waterLevel;                 
int V;                            

float val;                        /*定义温度检测的相关变量*/
float temp;                       
float filter_temp;
float finaltemp;
float waterTemp;
float valOfWaterTemp;

char ch[10];                     /*用于分析指令的数组*/

void setup() {                   /*程序初始化*/
  // put your setup code here, to run once:         
  Serial.begin(9600);           /*设置不同串口的波特率*/
  Serial1.begin(115200);
  
  pinMode(vcc, OUTPUT);          /*定义每个管脚的工作模式*/
  pinMode(gnd, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  pinMode(tempPin, INPUT);
  pinMode(waterTempPin,INPUT);
  
  pinMode(RGBVCC, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  
  pinMode(buzzerIn, INPUT);
  pinMode(buzzerVCC, OUTPUT);
  pinMode(buzzerGND, OUTPUT);
  
  pinMode(levelIn, INPUT);
  
  digitalWrite(vcc, HIGH);       /*给相关管脚一个初始状态*/
  digitalWrite(gnd, LOW);
  
  digitalWrite(buzzerVCC, HIGH);
  digitalWrite(buzzerGND, LOW);
  
  digitalWrite(RGBVCC, HIGH);
  analogWrite(R, 255);
  analogWrite(G, 0);
  analogWrite(B, 255);
  
  attachInterrupt(2, allClear, FALLING);      /*启用2号中断*/
}

void loop() {                     /*循环体（C语言中的主函数main（））*/
  // put your main code here, to run repeatedly:
  getdata(); 
  
  if (lowLevel()) {
    alarm();
  }
  
  if(!lowLevel()){
    analogWrite(R, 255);
    analogWrite(G, 0);
    analogWrite(B, 255);
    digitalWrite(buzzerIn, LOW);
  }
  
  remoteControl();
}

int lowLevel(){                   /*判断是否水位过低*/
    if(level<300){
      return 1;
    }
    else{
      return 0;
    }
}

void alarm() {                   /*报警程序*/
  for (int j = 0; j < 600; j++) {
    digitalWrite(buzzerIn, LOW);
    analogWrite(R, 0);
    analogWrite(G, 255);
    analogWrite(B, 255);
    delay(250);
    analogWrite(R, 255);
    analogWrite(G, 255);
    analogWrite(B, 0);
    digitalWrite(buzzerIn, HIGH);
    delay(80);
    if(Serial1.available()>0);break;
  }
}

void allClear() {                /*清除警报*/
  digitalWrite(RGBVCC, LOW);
  digitalWrite(buzzerVCC, LOW);
}


void remoteControl(){           /*用于分析远程控制指令的解析程序*/
  if (Serial1.available() > 0){
    for(int k=0;k<9;k++){
      ch[k]=char(Serial1.read());
      delay(10);
    }
    if(strncmp(ch,"level",5)==0){
      Serial1.print("Level:");
      Serial1.print(waterLevel,2);
      Serial1.println("%");
    }
    else if(strncmp(ch,"temp",4)==0){
      Serial1.print("Temperature:");
      Serial1.print(waterTemp,2);
      Serial1.println();
    }
  }
}


void getdata(){                 /*获取所有传感器的当前反馈信号*/
  level = analogRead(levelIn);
  
  val = analogRead(tempPin);
  valOfWaterTemp = analogRead(waterTempPin);
  temp = ((val * 5000) / 1024) / 10;
  waterTemp = ((valOfWaterTemp * 5000) / 1024) / 10;
  filter_temp = temp_filter();
  
  if ((temp - filter_temp) >= 0.15) {
    finaltemp=filter_temp + 0.3;
  }
  else if (((temp - filter_temp) < 0.15) && ((filter_temp - temp) < 0.15)) {
    finaltemp=filter_temp;
  }
  else if ((filter_temp - temp) >= 0.15) {
    finaltemp=filter_temp - 0.3;
  }
  
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  echoTime = pulseIn(echo, HIGH);
  filter_echoTime = echoTime_filter();
  fixed_distance = filter_echoTime*(331.5+0.61*filter_temp)/20000;
  waterLevel = 100-fixed_distance;

  Serial.print(level);
  Serial.print(',');
  Serial.print(waterTemp);
  Serial.println();
  
  delay(100);
}

int getEchoTime() {                    
  return echoTime;
}

#define echoTime_filterNum 4                                       /*超声波测距的滑动平均滤波*/
int echoTime_filterBuffer[echoTime_filterNum + 1];
int echoTime_filter() {
  int i = 0;
  float echoTime_filterSum = 0;
  echoTime_filterBuffer[echoTime_filterNum] = getEchoTime();
  for (i = 0; i < echoTime_filterNum; i++) {
    echoTime_filterBuffer[i] = echoTime_filterBuffer[i + 1];
    echoTime_filterSum += echoTime_filterBuffer[i];
  }
  return (float)(echoTime_filterSum / echoTime_filterNum);
}

float getTemp() {
  return temp;
}

#define temp_filterNum 5                                         /*LM35CZ温度检测的滑动平均滤波*/
float temp_filterBuffer[temp_filterNum+1];
float temp_filter(){
  int i;
  float temp_filterSum = 0;
  temp_filterBuffer[temp_filterNum] = getTemp();
  for(i = 0; i < temp_filterNum; i++){
    temp_filterBuffer[i] = temp_filterBuffer[i+1];
    temp_filterSum += temp_filterBuffer[i];
  }
  return (float)(temp_filterSum/temp_filterNum);
}




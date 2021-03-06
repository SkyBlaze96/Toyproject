 /*
MQ 센서  라이브러리 사용. 아두이노 메가 보드 사용,
ESP 8266 + ESP01 어댑터 사용.
2021.11.05 최종 수정
*/

  #include <MsTimer2.h>
  #include <MQUnifiedsensor.h>
  #define         Board                   ("Arduino MEGA")
  #define         Pin                     (A0)  //Analog input 4 of your arduino
  #define         Type                    ("MQ-9") //MQ9
  #define         Voltage_Resolution      (5)
  #define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
  #define         RatioMQ9CleanAir        (9.6) //RS / R0 = 60 ppm 
  String ssid = ""; //wifi ssid
  String PASSWORD = ""; // wifi pw
  String host = ""; //server ip
  int piezo = 7;
  
  int time_cnt;         //시간 변수 설정
  float sensorValue;    // 일산화탄소 센서 값 지역변수
  float sensorArray[10];  // 일산화탄소 센서 값 평균용도 배열
  float average = 0;  //평균값
  int average_m[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //평균 저장 배열
  int reset_var = 0;    //배열 10개 초기화 변수
  
  //Declare Sensor
  MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
  void connectWifi()
  {
    String join = "AT+CWJAP=\"" + ssid + "\",\"" + PASSWORD + "\"";
    Serial.println("Connect Wifi...");
    Serial1.println(join);
    delay(5000);
    if (Serial1.find("OK"))
    {
      Serial.print("WIFI connect\n");
    } else
    {
      Serial.println("connect timeout\n");
    }
    delay(1000);
  }
void httpclient(String char_input) { //네트워크 전송함수
  delay(100);
  Serial.println("connect TCP...");
  Serial1.println("AT+CIPSTART=\"TCP\",\"" + host + "\",8087");
  delay(500);
  if (Serial.find("ERROR")) return;
  Serial.println("Send data...");
  String url = char_input;
  String cmd = "GET /Gasproject/GetSensor?gas_level=" + url + " HTTP/1.0\r\n\r\n";
  Serial1.print("AT+CIPSEND=");
  Serial1.println(cmd.length());
  if (Serial1.find(">")) {
  } else
  {
    Serial1.println("AT+CIPCLOSE");
    delay(1000);
    return;
  }
  delay(500);
  Serial1.println(cmd);
  delay(100);
  if (Serial.find("ERROR"))
    return;
  Serial1.println("AT+CIPCLOSE");
  delay(100);
}
  void average_cal(){         //배열 평균 구하는 함수
    
    float avr_sum = 0;
    for(int i=0; i<=10; i++){
      avr_sum = avr_sum + average_m[i]; 
    }
    average = avr_sum / 10;
  }
  void cal()   //일산화탄소 측정 함수!!
  {            
      sensorValue = MQ9.readSensor();
      average_m[reset_var] = sensorValue ;  //평균을 구하기위한 배열에 센서값을 넣는다.   
       if(reset_var>=10){                   //배열길이가 10이기에 10이상이 되면 배열 초기화
         reset_var = 0;
       }   
      reset_var++;                          //배열 번호 증가
  
      if(average>=250){                 //평균 농도가 250ppm이 넘으면
        average_cal();                 //평균값 계산 함수 호출
        timer();                       //타이머 누적 시작
        work_rel();                    //릴레이 동작
        } 
      else{
        average_cal();                 //평균값 계산 함수 호출
          timer_reset();               //타이머 초기화
          off_rel();                   //릴레이 OFF
        }
  }
  void timer(){                     //평균값이 일정 수치 이상 일 경우 카운트 된다.
    time_cnt = time_cnt + 1 ;
    }
  void timer_reset(){               //평균값이 일정 수치로 내려갔을 때 카운트를 리셋한다.
    time_cnt = 0 ;
    }
  void work_rel(){ //릴레이 ON, 부저 ON
    digitalWrite(8,HIGH);
    digitalWrite(9,HIGH);
    tone(piezo, 336,500);
  }
  void off_rel(){ //릴레이 OFF, 부저 OFF
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
    noTone(piezo);
  }
  void setup() {
    Serial.begin(9600); //내장 시리얼 ON
    Serial1.begin(9600); // ESP8266 활용을 위한 시리얼 ON
    pinMode(8,OUTPUT);
    pinMode(9,OUTPUT);
    MQ9.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ9.init(); 
    Serial.print("Calibrating please wait.");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
      calcR0 += MQ9.calibrate(RatioMQ9CleanAir);
    }
    MQ9.setR0(calcR0/10);
    
    if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
    if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}
    MQ9.serialDebug(true);
    MsTimer2::set(1000,cal);
    MsTimer2::start();
    connectWifi();
  }
  void loop() {
    MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
    MQ9.setA(599.65); MQ9.setB(-2.244); // MQ-9 CO가스 측정 셋팅.
    String mid = "mid_1";  // 기기 ID
    String char_input = ""; 
       if(time_cnt >=180000 && average >250 ){ 
        timer_reset();        
        char_input =String (average) +"&user_mid="+mid;
        httpclient(char_input); 
       }
  }

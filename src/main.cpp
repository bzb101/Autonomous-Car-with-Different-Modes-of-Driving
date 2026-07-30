#include <Arduino.h>
#include <ESP32Servo.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_system.h>
#include <LittleFS.h>

enum CarState {
  AutonomousDefault,
  AutonomousCheckObstacle,
  AutonomousInitialSet,
  AutonomousTakeMeasurements,
  AutonomousTurn,
  AutonomousCheckStuck,
  Infrared,
  WiFimode,
  Wait
};
CarState state = AutonomousDefault;

typedef struct
{
  int angle;
  int distance;
} angDis;

#define servo_pin 4
#define IR_pin 5
#define SDA 2
#define SCL 1
#define Speed_Left 42
#define Left_Backwards 40
#define Left_Forwards 41
#define Speed_Right 39
#define Right_Backwards 37
#define Right_Forwards 38
#define WIDTH 128
#define HEIGHT 64
#define tWSend 100
#define scanStep 30
const int Detection_Distance = 250;
const int Normal_Speed_Motors = 180;
const int sizeArray = round(180 / scanStep + 1);
const int timeWaitStuck = 15000;
const float timeTurn1Degree = 10;
const uint16_t IRpin = 5;
const uint32_t onoff = 0xFFA25D;
const uint32_t forward = 0xFF629D;
const uint32_t backward = 0xFFA857;
const uint32_t right = 0xFF22DD;
const uint32_t left = 0xFFC23D;
const uint32_t continuous = 0xFF02FD;
const char *ssid = "Car_ESP32_S3";
const char *password = "123Password";
String mode = "";
int stucks = 0;
int distance;
int timeWait;
bool isPositioning = false;
bool isIRcontinuous = false;
bool isTurn = false;
unsigned long timeStuck = 0;
unsigned long lastDisplay = 0;
unsigned long tNowSend = 0;
unsigned long tnowAuto = 0;
decode_results data;
angDis scanMap[sizeArray];
CarState changeStateTo;

Servo servo;
VL53L1X tof;
IRrecv IR(IRpin);
Adafruit_SSD1306 screen(WIDTH, HEIGHT, &Wire, -1);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void Set_Motors(int speed, bool rf, bool rb, bool lf, bool lb);
void Turn(void);
void sort(angDis *values, int size);
void checkStuck(void);
void checkIR(void);
void IRmovement(void);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void sendTelemetry(void);
void updateScreen(void);
void changeState(CarState newState);

void setup() {
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-S3 Car...");
  
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  if (!LittleFS.begin()) {
    Serial.println("Error starting LittleFS");
    while (true);
  }
  if (!tof.init()) {
    Serial.println("Tof Sensor Not Found");
    while (true);
  }
  tof.setTimeout(75);
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(50);

  servo.setPeriodHertz(50);
  servo.attach(servo_pin, 500, 2400);
  servo.write(90);

  IR.enableIRIn();

  screen.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  screen.clearDisplay();
  screen.setTextSize(1);
  screen.setTextColor(SSD1306_WHITE);

  WiFi.softAP(ssid, password, 1);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.begin();

  timeStuck = millis();

  for (int i = 37; i < 43; i++) {
    pinMode(i, OUTPUT);
  }

  Serial.println("Working!");
}

void loop() 
{
  ws.cleanupClients();

  checkIR();

  updateScreen();

  bool newMesure = false;
  if (tof.dataReady())
  {
    int d = tof.read();

    if (d > 4)
    {
      distance = d;
      newMesure = true;
    }
    Serial.println(distance);
  }

  sendTelemetry();

  if (mode != "Autonomous")
  {
    mode = "Autonomous";
  }

  switch (state) 
  {
    case AutonomousDefault:
    {
      if (distance < Detection_Distance && distance > 4) 
      {
        //state = AutonomousCheckObstacle;
        changeState(AutonomousCheckObstacle);
      } 
      else
      {
        Set_Motors(Normal_Speed_Motors, HIGH, LOW, HIGH, LOW);
      }

      break;
    }

    case AutonomousCheckObstacle:
    {
      static int posibleObstacle = 0;
      static int averageDistance = 0;

      if (distance > 4 && distance < 4000) 
      {
        averageDistance += distance;
        posibleObstacle++;
      } 

      if (posibleObstacle == 3)
      {
        averageDistance = averageDistance / 3;

        if (averageDistance < Detection_Distance)
        {
          averageDistance = 0;
          posibleObstacle = 0;
          //state = AutonomousInitialSet;
          changeState(AutonomousInitialSet);
        }
        else
        {
          averageDistance = 0;
          posibleObstacle = 0;
          //state = AutonomousDefault;
          changeState(AutonomousDefault);
        }
      }

      break;
    }

    case AutonomousInitialSet:
    {
      Set_Motors(0, LOW, LOW, LOW, LOW);
      servo.write(0);

      timeWait = 240;
      changeStateTo = AutonomousTakeMeasurements;

      //state = Wait;
      changeState(Wait);

      break;
    }

    case AutonomousTakeMeasurements:
    {
      static int counter_map;
      static int i;

      if (newMesure)
      {
        scanMap[counter_map].distance = distance;
      }
      else
      {
        break;
      }
      
      scanMap[counter_map].angle = i;

      i += scanStep;
      counter_map++;

      if (i > 180)
      {
        servo.write(90);
        sort(scanMap, sizeArray);
        i = 0;
        counter_map = 0;
        //state = AutonomousTurn;
        changeState(AutonomousTurn);
      }
      else
      {
        servo.write(i);
        timeWait = 140;
        changeStateTo = AutonomousTakeMeasurements;
        //state = Wait;
        changeState(Wait);
      }

      break;
    }

    case AutonomousTurn:
    {
      Turn();

      break;
    }

    case AutonomousCheckStuck:
    {
      checkStuck();

      break;
    }

    case Infrared:
      {
        mode = "Infrared";

        break;
      }

    case WiFimode:
      {
        mode = "WiFi Mode";

        break;
      }

    case Wait:
    {
      static bool beforeWait = true;
      static unsigned long initalTimeWait;

      if (beforeWait)
      {
        initalTimeWait = millis();
        beforeWait = false;
      }

      if (millis() - initalTimeWait <= timeWait){}
      else
      {
        //Serial.println(millis() - initalTimeWait);
        beforeWait = true;
        //state = changeStateTo;
        changeState(changeStateTo);
      }

      break;
    }
  }
  yield();
}

void Set_Motors(int speed, bool rf, bool rb, bool lf, bool lb) {
  analogWrite(Speed_Left, speed);
  analogWrite(Speed_Right, speed);
  digitalWrite(Left_Forwards, lf);
  digitalWrite(Left_Backwards, lb);
  digitalWrite(Right_Forwards, rf);
  digitalWrite(Right_Backwards, rb);
}

void sort(angDis *values, int size)
{
  for (int i = 1; i < size; i++) 
  {
    angDis key = values[i];
    int z = i - 1;

    while (z >= 0 && values[z].distance < key.distance) 
    {
      values[z + 1] = values[z];
      z--;
    }
    values[z + 1] = key;
  }
}

void Turn(void)                                                             
{
  float timeTurn = abs(scanMap[0].angle - 90) * timeTurn1Degree;
  static bool beforeTurning = true;
  static unsigned long Past_Time_Turn;

  if (scanMap[0].angle == 90) 
  {
    timeTurn = timeTurn1Degree * 180;

    if (beforeTurning)
    {
      Set_Motors(Normal_Speed_Motors, HIGH, LOW, LOW, HIGH);
      Past_Time_Turn = millis();
      beforeTurning = false;
    }
    
    if (millis() - Past_Time_Turn <= timeTurn) {}
    else
    {
      beforeTurning = true;
      //state = AutonomousCheckStuck;
      changeState(AutonomousCheckStuck);
    }
  } 
  else if (scanMap[0].angle < 90) 
  {
    if (beforeTurning)
    {
      Set_Motors(Normal_Speed_Motors, LOW, HIGH, HIGH, LOW);
      Past_Time_Turn = millis();
      beforeTurning = false;
    }

    if (millis() - Past_Time_Turn <= timeTurn) {}
    else
    {
      beforeTurning = true;
      //state = AutonomousCheckStuck;
      changeState(AutonomousCheckStuck);
    }
  } 
  else 
  {
    if (beforeTurning)
    {
      Set_Motors(Normal_Speed_Motors, HIGH, LOW, LOW, HIGH);
      Past_Time_Turn = millis();
      beforeTurning = false;
    }

    if (millis() - Past_Time_Turn <= timeTurn) {}
    else
    {
      beforeTurning = true;
      //state = AutonomousCheckStuck;
      changeState(AutonomousCheckStuck);
    }
  }
}

void checkStuck(void) 
{
  screen.fillRect(0, 0, 70, 15, SSD1306_BLACK);
  screen.setCursor(0, 0);
  screen.print(stucks);
  screen.display();

  static bool beforeTurning = true;

  if (stucks == 5 && millis() - timeStuck >= timeWaitStuck) 
  {
    screen.fillRect(70, 0, 50, 15, SSD1306_BLACK);
    screen.setCursor(70, 0);
    screen.print(millis() - timeStuck);
    screen.display();

    static unsigned long Past_Time_Turn;
    if (beforeTurning)
    {
      Set_Motors(Normal_Speed_Motors, LOW, HIGH, LOW, HIGH);
      Past_Time_Turn = millis();
      beforeTurning = false;
    }

    if (millis() - Past_Time_Turn <= 2000) {Serial.println("marcha Atrás");}
    else
    {
      stucks = 0;
      beforeTurning = true;
      timeStuck = millis();
      //state = AutonomousInitialSet;
      changeState(AutonomousInitialSet);
    }
  } 
  else if (stucks >= 5) 
  {
    screen.fillRect(70, 0, 50, 15, SSD1306_BLACK);
    screen.setCursor(70, 0);
    screen.print(millis() - timeStuck);
    screen.display();

    stucks = 0;
    timeStuck = millis();
    //state = AutonomousDefault;
    changeState(AutonomousDefault);
  }
  else
  {
    //state = AutonomousDefault;
    changeState(AutonomousDefault);
    stucks++;
  }
}

void checkIR(void) 
{
  if (IR.decode(&data)) 
  {
    if (data.value == onoff) 
    {
      if (state != Infrared) 
      {
        state = Infrared;
        Set_Motors(0, LOW, LOW, LOW, LOW);
      } 
      else 
      {
        state = AutonomousDefault;
      }
    }
    IR.resume();
  }
}

void IRmovement(void) 
{
  if (IR.decode(&data)) 
  {
    if ((data.value == forward || data.value == backward) && !isIRcontinuous) 
    {
      unsigned long Past_Time_Turn = millis();

      while (millis() - Past_Time_Turn <= 550) 
      {
        checkIR();
        if (state != Infrared) {
          break;
        }

        if (data.value == forward) 
        {
          Set_Motors(Normal_Speed_Motors, HIGH, LOW, HIGH, LOW);
        }
        if (data.value == backward) 
        {
          Set_Motors(Normal_Speed_Motors, LOW, HIGH, LOW, HIGH);
        }
      }
      Set_Motors(Normal_Speed_Motors, LOW, LOW, LOW, LOW);
    } 
    else if (data.value == left || data.value == right) 
    {
      unsigned long Past_Time_Turn = millis();

      while (millis() - Past_Time_Turn <= timeTurn1Degree * 30) 
      {
        checkIR();
        if (state != Infrared) {
          break;
        }

        if (data.value == left) 
        {
          Set_Motors(Normal_Speed_Motors, LOW, HIGH, HIGH, LOW);
        }
        if (data.value == right) 
        {
          Set_Motors(Normal_Speed_Motors, HIGH, LOW, LOW, HIGH);
        }
      }
      
      if (isIRcontinuous) 
      {
        Set_Motors(Normal_Speed_Motors, HIGH, LOW, HIGH, LOW);
      }
      else
      {
        Set_Motors(Normal_Speed_Motors, LOW, LOW, LOW, LOW);
      }
    } 
    else if (data.value == continuous) 
    {
      if (!isIRcontinuous) 
      {
        Set_Motors(Normal_Speed_Motors, HIGH, LOW, HIGH, LOW);
        isIRcontinuous = true;
      } 
      else 
      {
        Set_Motors(Normal_Speed_Motors, LOW, LOW, LOW, LOW);
        isIRcontinuous = false;
      }
    } 
    else if (data.value == onoff) 
    {
      state = AutonomousDefault;
      isIRcontinuous = false;
    }

    IR.resume();
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
  switch (type) 
  {
    case WS_EVT_CONNECT:

      screen.clearDisplay();
      screen.setCursor(0,0);
      screen.println("WS Connected");
      screen.display();
      break;
    case WS_EVT_DISCONNECT:
      Set_Motors(0, LOW, LOW, LOW, LOW);
      state = AutonomousDefault;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    default:
      break;
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) 
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
  {
    String command;
    for (size_t i = 0; i < len; i++) 
    {
      command += (char)data[i];
    }

    if (command == "MODE") 
    {
      if (state != WiFimode) 
      {
        state = WiFimode;
        Set_Motors(0, LOW, LOW, LOW, LOW);
      } 
      else 
      {
        state = AutonomousDefault;
      }
      return;
    }

    if (state != WiFimode) return;

    if (command == "F") Set_Motors(Normal_Speed_Motors, HIGH, LOW, HIGH, LOW);
    else if (command == "B") Set_Motors(Normal_Speed_Motors, LOW, HIGH, LOW, HIGH);
    else if (command == "R") Set_Motors(Normal_Speed_Motors, LOW, HIGH, HIGH, LOW);
    else if (command == "L") Set_Motors(Normal_Speed_Motors, HIGH, LOW, LOW, HIGH);
    else if (command == "S") Set_Motors(0, LOW, LOW, LOW, LOW);
  }
}

void sendTelemetry(void) 
{
  if (millis() - tNowSend >= tWSend && ws.count() > 0)
  {
    StaticJsonDocument<128> data;

    data["distance"] = distance;
    data["servo"] = servo.read();
    data["mode"] = mode;

    char buffer[128];
    size_t len = serializeJson(data, buffer);

    ws.textAll(buffer, len);

    tNowSend = millis();
  } 
}

void updateScreen(void)
{
  static String isNewMode = "";

  if (mode != isNewMode) 
  {
    screen.clearDisplay();

    screen.setCursor(35, 17);
    screen.print(mode);

    screen.setCursor(17, 55);
    screen.print("IP: ");
    screen.print(WiFi.softAPIP());

    screen.display();

    isNewMode = mode;
  }
}

void changeState(CarState newState)
{
    Serial.print(state);
    Serial.print(" -> ");
    Serial.println(newState);
    state = newState;
}

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define trigPin D0
#define echoPin D9
#define soilPin A0
#define in1 D1
#define in2 D2
#define in3 D3
#define in4 D4
#define ena D7
#define enb D8

// Servo
Servo steeringServo;

// WiFi AP
const char* ssid_ap = "RobotCar_AP";
const char* password_ap = "12345678";
ESP8266WebServer server(80);
WiFiClient client;
const char* receiverIP = "192.168.4.2";  // Receiver NodeMCU IP

// Soil + NPK
SoftwareSerial npkSerial(D5, D6);
int moistureVal = 0;
uint16_t nitrogen = 0, phosphorus = 0, potassium = 0;

// Flags
bool autoMode = false;
int speedVal = 80;

void setup() {
  Serial.begin(9600);
  npkSerial.begin(9600);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT); pinMode(enb, OUTPUT);
  analogWrite(ena, speedVal);
  analogWrite(enb, speedVal);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(soilPin, INPUT);

  steeringServo.attach(D5);
  steeringServo.write(90);

  WiFi.softAP(ssid_ap, password_ap);
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/forward", []() { moveForward(); redirectHome(); });
  server.on("/back", []() { moveBack(); redirectHome(); });
  server.on("/left", []() { turnLeft(); redirectHome(); });
  server.on("/right", []() { turnRight(); redirectHome(); });
  server.on("/stop", []() { stopCar(); redirectHome(); });
  server.on("/auto", []() { autoMode = !autoMode; stopCar(); redirectHome(); });
  server.on("/loading", handleLoadingPage);
  server.on("/scan", handleScan);
  server.on("/triggerReceiver", []() { sendTriggerToReceiver(); redirectHome(); });

  server.begin();
}

void loop() {
  server.handleClient();

  if (autoMode) {
    int leftDist = scanAtAngle(45);
    int centerDist = scanAtAngle(90);
    int rightDist = scanAtAngle(135);

    if (centerDist > 25) moveForward();
    else if (leftDist > rightDist && leftDist > 25) { turnLeft(); delay(400); stopCar(); }
    else if (rightDist > 25) { turnRight(); delay(400); stopCar(); }
    else stopCar();

    delay(500);
  }
}

// Movement
void moveForward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
void moveBack() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}
void turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
void turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}
void stopCar() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

// Ultrasonic
long getDistanceCM() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}
int scanAtAngle(int angle) {
  steeringServo.write(angle); delay(500);
  return getDistanceCM();
}

// NPK
void readNPK() {
  byte req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x05, 0xCB};
  npkSerial.write(req, 8); delay(300);
  if (npkSerial.available() >= 9) {
    byte res[9]; for (int i = 0; i < 9; i++) res[i] = npkSerial.read();
    nitrogen = (res[3] << 8) | res[4];
    phosphorus = (res[5] << 8) | res[6];
    potassium = (res[7] << 8) | res[8];
  } else nitrogen = -2000; phosphorus = 5; potassium = 100;
}

// Recommendation
String getRecommendation() {
  String seed = "Unknown", fert = "Unknown";
  if (moistureVal > 300 && nitrogen > 100 && phosphorus > 50) {
    seed = " Maize"; fert = "Urea";
  } else if (moistureVal < 300 && nitrogen < 50) {
    seed = " Millet"; fert = "Ammonium Sulphate";
  } else {
    seed = " Cassava"; fert = "NPK 15:15:15";
  }
  return "<div class='result'>🌱 Seed: <b>" + seed + "</b></div>" +
         "<div class='result'>💊 Fertilizer: <b>" + fert + "</b></div>";
}

// Trigger Receiver
void sendTriggerToReceiver() {
  HTTPClient http;
  String url = "http://" + String(receiverIP) + "/trigger";
  http.begin(client, url);
  int code = http.GET();
  http.end();
}

// HTML Pages
void handleRoot() {
  String html = R"====(
  <html><head><title>Robot Car</title><meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>body { font-family: Arial; text-align:center; padding:10px; }
  .btn { display:block; width:80%; margin:10px auto; padding:12px; font-size:18px;
  background:#3498db; color:white; border:none; border-radius:6px; text-decoration:none; }</style>
  </head><body><h2> Smart Robot Car</h2>
  <a href='/forward' class='btn'> Forward</a>
  <a href='/back' class='btn'> Back</a>
  <a href='/left' class='btn'> Left</a>
  <a href='/right' class='btn'> Right</a>
  <a href='/stop' class='btn' style='background:#e74c3c;'> Stop</a>
  <a href='/auto' class='btn' style='background:#27ae60;'> Toggle Auto</a>
  <a href='/loading' class='btn' style='background:#8e44ad;'> Scan Soil</a>
  </body></html>)====";
  server.send(200, "text/html", html);
}

void handleLoadingPage() {
  server.send(200, "text/html",
    "<html><head><meta http-equiv='refresh' content='4;url=/scan'>"
    "<style>.bar{width:1%;height:20px;background:#4CAF50;animation:load 4s forwards}@keyframes load{100%{width:100%}}</style></head>"
    "<body><h2>Scanning Soil...</h2><div style='width:90%;margin:auto;background:#ccc'><div class='bar'></div></div></body></html>");
}

void handleScan() {
  moistureVal = 63;//analogRead(soilPin);
  readNPK();

  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><style>";
  html += "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f9f9f9; }";
  html += "h2 { color: #2c3e50; margin-bottom: 20px; }";
  html += ".container { max-width: 400px; margin: auto; background: #fff; padding: 20px; box-shadow: 0 0 8px rgba(0,0,0,0.1); border-radius: 10px; }";
  html += ".result { font-size: 18px; margin-bottom: 15px; padding: 10px; background: #ecf0f1; border-radius: 6px; }";
  html += ".btn { display: block; width: 100%; padding: 12px; margin-top: 15px; background: #3498db; color: white; border: none; border-radius: 5px; font-size: 16px; text-decoration: none; }";
  html += ".btn:hover { background: #2980b9; }";
  html += "</style></head><body><div class='container'>";
  html += "<h2>Soil Scan Results</h2>";
  html += "<div class='result'> <strong>Moisture:</strong> " + String(moistureVal) + "</div>";
  html += "<div class='result'><strong>Nitrogen:</strong> " + String(nitrogen) + " mg/kg</div>";
  html += "<div class='result'><strong>Phosphorus:</strong> " + String(phosphorus) + " mg/kg</div>";
  html += "<div class='result'><strong>Potassium:</strong> " + String(potassium) + " mg/kg</div>";
  html += getRecommendation();  // Adds seed + fertilizer recommendation
  html += "<a class='btn' href='/triggerReceiver'> Plant</a>";
  html += "<a class='btn' href='/'> Back to Controls</a>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}


void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

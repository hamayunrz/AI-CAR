#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <deque>
using namespace std;

// WIFI
const char* WIFI_SSID = "vuDevices";
const char* WIFI_PASS = "Acorn1873";

WebServer server(80);

// MOTOR PINS
int ENA = 33;
int IN1 = 26;
int IN2 = 27;

int ENB = 14;
int IN3 = 32;
int IN4 = 13;

// pwm
int PWM_FREQ = 1000;
int PWM_RES = 8;

// command struct
struct MotionCommand {
    int drive;        // back wheels
    int steer;        // front wheels
    unsigned long drive_ms;
    unsigned long steer_ms;
};

deque<MotionCommand> commandQueue;

// state stuff
bool isExecuting = false;
bool steerActive = false;

MotionCommand currentCommand;

unsigned long commandStartTime = 0;
unsigned long steerStartTime = 0;

// motor control
void setMotor(int enPin, int inA, int inB, int value) {

    int pwm = constrain(abs(value), 0, 255);

    if (value > 0) {
        digitalWrite(inA, HIGH);
        digitalWrite(inB, LOW);
    }
    else if (value < 0) {
        digitalWrite(inA, LOW);
        digitalWrite(inB, HIGH);
    }
    else {
        digitalWrite(inA, LOW);
        digitalWrite(inB, LOW);
    }

    ledcWrite(enPin, pwm);
}

void setDrive(int v) {
    setMotor(ENA, IN1, IN2, v);
}

void setSteer(int v) {
    setMotor(ENB, IN3, IN4, v);
}

void stopDrive() { setMotor(ENA, IN1, IN2, 0); }
void stopSteer() { setMotor(ENB, IN3, IN4, 0); }

void stopAll() {
    stopDrive();
    stopSteer();
    Serial.println("STOP");
}

// add to queue
bool addCommand(JsonObject obj, String& err) {

    if (!obj["drive"].is<int>() || !obj["drive_ms"].is<unsigned long>()) {
        err = "need drive + drive_ms";
        return false;
    }

    MotionCommand cmd;

    cmd.drive = constrain(obj["drive"].as<int>(), -255, 255);
    cmd.drive_ms = obj["drive_ms"].as<unsigned long>();

    if (obj["steer"].is<int>())
        cmd.steer = constrain(obj["steer"].as<int>(), -255, 255);
    else
        cmd.steer = 0;

    if (obj["steer_ms"].is<unsigned long>())
        cmd.steer_ms = obj["steer_ms"].as<unsigned long>();
    else
        cmd.steer_ms = 0;

    commandQueue.push_back(cmd);
    return true;
}

// ROUTES
void handleRoot() {
    server.send(200, "text/plain", "ESP32 CAR ONLINE");
}

void handleStatus() {

    DynamicJsonDocument doc(256);

    doc["queued"] = commandQueue.size();
    doc["running"] = isExecuting;
    doc["ip"] = WiFi.localIP().toString();

    String res;
    serializeJson(doc, res);

    server.send(200, "application/json", res);
}

void handleStop() {

    commandQueue.clear();
    isExecuting = false;
    steerActive = false;

    stopAll();

    server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

void handleCommand() {

    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"no body\"}");
        return;
    }

    DynamicJsonDocument doc(1024);

    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
    }

    String err;

    if (!addCommand(doc.as<JsonObject>(), err)) {
        server.send(400, "application/json", "{\"error\":\"" + err + "\"}");
        return;
    }

    server.send(200, "application/json", "{\"status\":\"queued\"}");
}

void handleCommands() {

    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"no body\"}");
        return;
    }

    DynamicJsonDocument doc(4096);

    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
    }

    if (!doc["sequence"].is<JsonArray>()) {
        server.send(400, "application/json", "{\"error\":\"need sequence\"}");
        return;
    }

    String err;

    for (JsonObject obj : doc["sequence"].as<JsonArray>()) {
        if (!addCommand(obj, err)) {
            server.send(400, "application/json", "{\"error\":\"" + err + "\"}");
            return;
        }
    }

    server.send(200, "application/json", "{\"status\":\"sequence queued\"}");
}

// execution
void startNextCommand() {

    if (commandQueue.empty()) {
        isExecuting = false;
        stopAll();
        return;
    }

    currentCommand = commandQueue.front();
    commandQueue.pop_front();

    setDrive(currentCommand.drive);
    commandStartTime = millis();

    if (currentCommand.steer != 0 && currentCommand.steer_ms > 0) {
        setSteer(currentCommand.steer);
        steerStartTime = millis();
        steerActive = true;
    }
    else {
        stopSteer();
        steerActive = false;
    }

    Serial.printf("CMD %d | %lu ms\n", currentCommand.drive, currentCommand.drive_ms);

    isExecuting = true;
}

void updateExecutor() {

    if (!isExecuting) {
        if (!commandQueue.empty())
            startNextCommand();
        return;
    }

    unsigned long now = millis();

    if (steerActive && (now - steerStartTime >= currentCommand.steer_ms)) {
        stopSteer();
        steerActive = false;
        Serial.println("steer done");
    }

    if (now - commandStartTime >= currentCommand.drive_ms) {

        stopDrive();
        stopSteer();

        isExecuting = false;
        steerActive = false;

        if (!commandQueue.empty())
            startNextCommand();
        else
            Serial.println("done");
    }
}

// wifi
void connectWiFi() {

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("connecting ");

    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nok");
        Serial.println(WiFi.localIP());
    }
    else {
        Serial.println("\nfail");
    }
}

void setup() {

    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);

    stopAll();

    connectWiFi();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/command", HTTP_POST, handleCommand);
    server.on("/commands", HTTP_POST, handleCommands);

    server.begin();

    Serial.println("server up");
}

void loop() {
    server.handleClient();
    updateExecutor();
}
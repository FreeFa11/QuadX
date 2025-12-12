#pragma once

// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <LittleFS.h>

// Definitions
#define WebserverPort           80
#define WebsocketURL            "/CommunicationSocket"
#define DomainName              "quadx"

enum ResponseCode
{
    Okay                = 200,           
    Created             = 201,
    NoContent           = 204,
    BadRequest          = 400,
    Unauthorized        = 401,
    Forbidden           = 403,
    NotFound            = 404,
    MethodNotAllowed    = 405,
    InternalServerError = 500,
    BadGateway          = 502,
    ServiceUnavailable  = 503,
    GatewayTimeout      = 504,
    MovedPermanently    = 301,
    Found               = 302,
    NotModified         = 304
};

enum States
{
    Control,
    Calibrate,
    Connect
};

typedef struct {
    int16_t     Slider1=0, Slider2=0, Slider3=0, Slider4=0;
    int16_t     JoystickX1=0, JoystickX2=0, JoystickY1=0, JoystickY2=0;
    bool        Toggle1=false, Toggle2=false, Toggle3=false, Toggle4=false;
} ControllerData;

typedef struct {
    bool        save=false; 
    int16_t     motorA=0, motorB=0, motorC=0, motorD=0, maxthrottle=0;
    float       P=0, I=0, D=0;
    uint8_t     sensitivityS1=0, sensitivityS2=0, sensitivityS3=0, sensitivityS4=0;
    uint8_t     sensitivityX1=0, sensitivityX2=0, sensitivityY1=0, sensitivityY2=0;
} CalibrationData;


// Objects
extern QueueHandle_t ControllerQueue;
extern QueueHandle_t CalibrationQueue;
extern QueueHandle_t StateQueue;

// Declarations
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void HandleInputData(uint8_t *data, size_t len);
void JsonToCalibration(JsonDocument JSON, CalibrationData &Calib);
void CalibrationToJson(CalibrationData Calib, JsonDocument &JSON);
void ReadCalibration(CalibrationData &Calib);
void SaveCalibration(CalibrationData Calib);

class Webserver
{
public:
    Webserver();
    ~Webserver();

    void StartWiFi();
    void StartWebserver();
};
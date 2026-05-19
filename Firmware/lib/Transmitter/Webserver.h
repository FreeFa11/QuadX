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
#include "Transmitter.h"


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
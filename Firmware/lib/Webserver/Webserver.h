// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Definitions
#define WebserverPort           80
#define WebsocketURL            "/CommunicationSocket"
#define DomainName              "dronecontrol"

enum Code
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

typedef struct {
    int Slider1 = 0, Slider2 = 0, Slider3 = 0, Slider4 = 0;
    bool Toggle1 = false, Toggle2 = false, Toggle3 = false, Toggle4 = false;
    float JoystickX1 = 0, JoystickX2 = 0, JoystickY1 = 0, JoystickY2 = 0;
} ControllerData;




// Objects
#include "../../include/Webpage.h"
extern QueueHandle_t InputQueue;


// Declarations
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void HandleInputData(uint8_t *data, size_t len);

class Webserver
{
public:
    Webserver();
    ~Webserver();

    void StartWiFi();
    void StartWebserver();
};
#include <Webserver.h>


// Objects
AsyncWebServer myServer(WebserverPort);
AsyncWebSocket mySocket(WebsocketURL);
Preferences PermanentData;
QueueHandle_t ControllerQueue;
QueueHandle_t CalibrationQueue;
QueueHandle_t StateQueue;       // For switching the state



// Definitions
Webserver::Webserver(){}
Webserver::~Webserver(){}

void Webserver::StartWiFi()
{
    PermanentData.begin("Connection");
    if (PermanentData.isKey("mode"))
    {
        if (PermanentData.getString("mode") == "ap")
        {
            WiFi.mode(WIFI_MODE_AP);
            WiFi.setHostname("QuadX");
            WiFi.softAP(PermanentData.getString("ssid"), PermanentData.getString("pass"));
        }
        else
        {
            WiFi.mode(WIFI_MODE_STA);
            WiFi.setHostname("QuadX");
            WiFi.begin(PermanentData.getString("ssid"), PermanentData.getString("pass"));

            // Wait till Connection
            Serial.print("\nConnecting ");
            while(WiFi.status() != WL_CONNECTED)
            {
                Serial.print('.');
                vTaskDelay(250 / portTICK_PERIOD_MS);
            }
            Serial.println("\tConnected!!\n");

            // Print Local IP
            Serial.print("\nIP Address: ");
            Serial.println(WiFi.localIP());
            vTaskDelay(100 /portTICK_PERIOD_MS);

            // Multicast DNS 
            MDNS.begin(DomainName);
            return;
        }
    }
    else
    {
        WiFi.mode(WIFI_MODE_AP);
        WiFi.setHostname("QuadX");
        WiFi.softAP("QuadX", "letsrock");
    }
    PermanentData.end();
   
    // Wait till Connection
    Serial.print("\nConnecting ");
    while (WiFi.softAPgetStationNum() < 1)
    {
        Serial.print('.');
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    Serial.println("\tConnected!!\n");

    // Print the IP
    Serial.print("\nIP Address: ");
    Serial.println(WiFi.softAPIP());
    vTaskDelay(100 /portTICK_PERIOD_MS);
    
    // Multicast DNS 
    MDNS.begin(DomainName);
}

void Webserver::StartWebserver()
{
    myServer.serveStatic("/favicon.ico", LittleFS, "/favicon.ico");
    myServer.serveStatic("/controller-icon.svg", LittleFS, "/controller-icon.svg");
    myServer.serveStatic("/calibration-icon.svg", LittleFS, "/calibration-icon.svg");
    myServer.serveStatic("/connection-icon.svg", LittleFS, "/connection-icon.svg");

    myServer.serveStatic("/Controller.html", LittleFS, "/Controller.html");
    myServer.serveStatic("/Calibration.html", LittleFS, "/Calibration.html");
    myServer.serveStatic("/Connection.html", LittleFS, "/Connection.html");
    
    // GET Calibration Saved Data
    myServer.on("/api/getCalibration", HTTP_GET, [](AsyncWebServerRequest* request){

        JsonDocument JsonData;
        CalibrationData SavedData;
        String StringData;
        
        // Read from Preferences
        ReadCalibration(SavedData);
        CalibrationToJson(SavedData, JsonData);
        serializeJson(JsonData, StringData);
        
        request->send(ResponseCode::Okay, "application/json", StringData);
    });
    
    myServer.serveStatic("/", LittleFS, "/").setDefaultFile("Controller.html");
    myServer.addHandler(&mySocket);
    myServer.begin();
    mySocket.onEvent(onWebSocketEvent);

    // Define Queue
    ControllerQueue = xQueueCreate(2, sizeof(ControllerData));
    CalibrationQueue = xQueueCreate(2, sizeof(CalibrationData));
    StateQueue = xQueueCreate(1, sizeof(States));
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
        case WS_EVT_CONNECT:
            Serial.printf("\nPage Connected\t: #%u %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DATA:
            HandleInputData(data, len);
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("\nPage Disconnected\n");
            break;
    }
}

void HandleInputData(uint8_t *data, size_t len)
{
    JsonDocument myJSON;
    DeserializationError error = deserializeJson(myJSON, data, len);
    if (error) {
        Serial.println("JSON Deserialize Failed!!");
        return;
    }

    static States State;
    if (myJSON.size()==1)
    {
        // Change the State
        if      (myJSON["type"] == "Controller")    {State=States::Control;}
        else if (myJSON["type"] == "Calibration")   {State=States::Calibrate;}
        else if (myJSON["type"] == "Connection")    {State=States::Connect;}

        xQueueSend(StateQueue, &State, 0);
    }
    

    switch (State)
    {
        case States::Control:   {            
            ControllerData newData;
            newData.Slider1 = int16_t(myJSON["S1"]);
            newData.Slider2 = int16_t(myJSON["S2"]);
            newData.Slider3 = int16_t(myJSON["S3"]);
            newData.Slider4 = int16_t(myJSON["S4"]);
            newData.Toggle1 = bool(myJSON["T1"]);
            newData.Toggle2 = bool(myJSON["T2"]);
            newData.Toggle3 = bool(myJSON["T3"]);
            newData.Toggle4 = bool(myJSON["T4"]);
            newData.JoystickX1 = int16_t(myJSON["J1"]["X"]);
            newData.JoystickY1 = -int16_t(myJSON["J1"]["Y"]);     // Y comes inverted
            newData.JoystickX2 = int16_t(myJSON["J2"]["X"]);
            newData.JoystickY2 = -int16_t(myJSON["J2"]["Y"]);
            xQueueSend(ControllerQueue, &newData, 0);
        }   break;
            
        case States::Calibrate: {
            CalibrationData Data;
            if      (myJSON["type"] == "output")    {Data.save = false;}
            else if (myJSON["type"] == "save"  )    {Data.save = true; }
            JsonToCalibration(myJSON, Data);
            xQueueSend(CalibrationQueue, &Data, 0);
        }   break;
            
        case States::Connect:   {
            // Save data to Flash
            if (!myJSON["pass"].isNull() & !myJSON["ssid"].isNull())
            {
                PermanentData.begin("Connection");
                PermanentData.putString("mode", myJSON["mode"].as<String>());
                PermanentData.putString("ssid", myJSON["ssid"].as<String>());
                PermanentData.putString("pass", myJSON["pass"].as<String>());
                PermanentData.end();
            }

        }   break;
    }

}




// Calibration Data Conversion Functions

// motorA:        0
// motorB:        0
// motorC:        0
// motorD:        0
// maxthrottle:   0
// P:             0
// I:             0
// D:             0
// sensitivityS1: 0,
// sensitivityS2: 0,
// sensitivityS3: 0,
// sensitivityS4: 0,
// sensitivityX1: 0,
// sensitivityX2: 0,
// sensitivityY1: 0,
// sensitivityY2: 0

void JsonToCalibration(JsonDocument JSON, CalibrationData &Calib)
{
    Calib.motorA = int16_t(JSON["motorA"]);
    Calib.motorB = int16_t(JSON["motorB"]);
    Calib.motorC = int16_t(JSON["motorC"]);
    Calib.motorD = int16_t(JSON["motorD"]);
    Calib.maxthrottle = int16_t(JSON["maxthrottle"]);
    Calib.P = float(JSON["P"]);
    Calib.I = float(JSON["I"]);
    Calib.D = float(JSON["D"]);
    Calib.sensitivityS1 = uint8_t(JSON["sensitivityS1"]);
    Calib.sensitivityS2 = uint8_t(JSON["sensitivityS2"]);
    Calib.sensitivityS3 = uint8_t(JSON["sensitivityS3"]);
    Calib.sensitivityS4 = uint8_t(JSON["sensitivityS4"]);
    Calib.sensitivityX1 = uint8_t(JSON["sensitivityX1"]);
    Calib.sensitivityX2 = uint8_t(JSON["sensitivityX2"]);
    Calib.sensitivityY1 = uint8_t(JSON["sensitivityY1"]);
    Calib.sensitivityY2 = uint8_t(JSON["sensitivityY2"]);
}

void CalibrationToJson(CalibrationData Calib, JsonDocument &JSON)
{
    JSON["motorA"] = Calib.motorA;
    JSON["motorB"] = Calib.motorB;
    JSON["motorC"] = Calib.motorC;
    JSON["motorD"] = Calib.motorD;
    JSON["maxthrottle"] = Calib.maxthrottle;
    JSON["P"] = Calib.P;
    JSON["I"] = Calib.I;
    JSON["D"] = Calib.D;
    JSON["sensitivityS1"] = Calib.sensitivityS1;
    JSON["sensitivityS2"] = Calib.sensitivityS2;
    JSON["sensitivityS3"] = Calib.sensitivityS3;
    JSON["sensitivityS4"] = Calib.sensitivityS4;
    JSON["sensitivityX1"] = Calib.sensitivityX1;
    JSON["sensitivityX2"] = Calib.sensitivityX2;
    JSON["sensitivityY1"] = Calib.sensitivityY1;
    JSON["sensitivityY2"] = Calib.sensitivityY2;
}

void SaveCalibration(CalibrationData Calib)
{
    PermanentData.begin("Calibration");

    PermanentData.putShort("motorA", Calib.motorA);
    PermanentData.putShort("motorB", Calib.motorB);
    PermanentData.putShort("motorC", Calib.motorC);
    PermanentData.putShort("motorD", Calib.motorD);
    PermanentData.putShort("maxthrottle", Calib.maxthrottle);
    PermanentData.putFloat("P", Calib.P);
    PermanentData.putFloat("I", Calib.I);
    PermanentData.putFloat("D", Calib.D);
    PermanentData.putUChar("sensitivityS1", Calib.sensitivityS1);
    PermanentData.putUChar("sensitivityS2", Calib.sensitivityS2);
    PermanentData.putUChar("sensitivityS3", Calib.sensitivityS3);
    PermanentData.putUChar("sensitivityS4", Calib.sensitivityS4);
    PermanentData.putUChar("sensitivityX1", Calib.sensitivityX1);
    PermanentData.putUChar("sensitivityX2", Calib.sensitivityX2);
    PermanentData.putUChar("sensitivityY1", Calib.sensitivityY1);
    PermanentData.putUChar("sensitivityY2", Calib.sensitivityY2);

    PermanentData.end();
}

void ReadCalibration(CalibrationData &Calib)
{
    PermanentData.begin("Calibration");

    if (PermanentData.isKey("motorA"))
    {
        Calib.motorA = PermanentData.getShort("motorA");
        Calib.motorB = PermanentData.getShort("motorB");
        Calib.motorC = PermanentData.getShort("motorC");
        Calib.motorD = PermanentData.getShort("motorD");
        Calib.maxthrottle = PermanentData.getShort("maxthrottle");
        Calib.P = PermanentData.getFloat("P");
        Calib.I = PermanentData.getFloat("I");
        Calib.D = PermanentData.getFloat("D");
        Calib.sensitivityS1 = PermanentData.getUChar("sensitivityS1");
        Calib.sensitivityS2 = PermanentData.getUChar("sensitivityS2");
        Calib.sensitivityS3 = PermanentData.getUChar("sensitivityS3");
        Calib.sensitivityS4 = PermanentData.getUChar("sensitivityS4");
        Calib.sensitivityX1 = PermanentData.getUChar("sensitivityX1");
        Calib.sensitivityX2 = PermanentData.getUChar("sensitivityX2");
        Calib.sensitivityY1 = PermanentData.getUChar("sensitivityY1");
        Calib.sensitivityY2 = PermanentData.getUChar("sensitivityY2");
    }

    PermanentData.end();
}
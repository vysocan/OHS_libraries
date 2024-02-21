#ifndef BOARD_HAS_PSRAM
#error "Please enable PSRAM !!!"
#endif

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Arduino.h>
#include "epd_driver.h"
#include "libjpeg/libjpeg.h"
#include "firasans.h"
#include "esp_adc_cal.h"
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include "logo.h"

#define FILE_SYSTEM SD

#if CONFIG_IDF_TARGET_ESP32S3
#include "pcf8563.h"
#include <Wire.h>
#endif

#if CONFIG_IDF_TARGET_ESP32
#define BATT_PIN 36
#elif CONFIG_IDF_TARGET_ESP32S3
#define BATT_PIN 14
#else
#error "Platform not supported"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
#define SD_MISO 12
#define SD_MOSI 13
#define SD_SCLK 14
#define SD_CS 15
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define SD_MISO 16
#define SD_MOSI 15
#define SD_SCLK 11
#define SD_CS 42
#else
#error "Platform not supported"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3)
PCF8563_Class rtc;
#define TOUCH_SCL 17
#define TOUCH_SDA 18
#endif

#define DBG_OUTPUT_PORT Serial

const char *ssid = "ChinaNet-A179";
const char *password = "88888888";
const char *host = "esp32s3";

WebServer server(80);
static bool hasFILE_SYSTEM = false;
File uploadFile;
EventGroupHandle_t handleServer;
String pic_path;
uint8_t *framebuffer;
#define BIT_CLEAN _BV(0)
#define BIT_SHOW _BV(1)

void returnOK()
{
    server.send(200, "text/plain", "");
}

void returnFail(String msg)
{
    server.send(500, "text/plain", msg + "\r\n");
}

bool loadFromFILE_SYSTEM(String path)
{
    String dataType = "text/plain";
    if (path.endsWith("/"))
    {
        path += "index.htm";
    }

    if (path.endsWith(".src"))
    {
        path = path.substring(0, path.lastIndexOf("."));
    }
    else if (path.endsWith(".htm"))
    {
        dataType = "text/html";
    }
    else if (path.endsWith(".css"))
    {
        dataType = "text/css";
    }
    else if (path.endsWith(".js"))
    {
        dataType = "application/javascript";
    }
    else if (path.endsWith(".png"))
    {
        dataType = "image/png";
    }
    else if (path.endsWith(".gif"))
    {
        dataType = "image/gif";
    }
    else if (path.endsWith(".jpg"))
    {
        dataType = "image/jpeg";
    }
    else if (path.endsWith(".ico"))
    {
        dataType = "image/x-icon";
    }
    else if (path.endsWith(".xml"))
    {
        dataType = "text/xml";
    }
    else if (path.endsWith(".pdf"))
    {
        dataType = "application/pdf";
    }
    else if (path.endsWith(".zip"))
    {
        dataType = "application/zip";
    }

    File dataFile = FILE_SYSTEM.open(path.c_str());
    if (dataFile.isDirectory())
    {
        path += "/index.htm";
        dataType = "text/html";
        dataFile = FILE_SYSTEM.open(path.c_str());
    }

    if (!dataFile)
    {
        return false;
    }

    if (server.hasArg("download"))
    {
        dataType = "application/octet-stream";
    }

    if (server.streamFile(dataFile, dataType) != dataFile.size())
    {
        DBG_OUTPUT_PORT.println("Sent less data than expected!");
    }

    dataFile.close();
    return true;
}

void handleFileUpload()
{
    if (server.uri() != "/edit")
    {
        return;
    }
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START)
    {
        if (FILE_SYSTEM.exists((char *)upload.filename.c_str()))
        {
            FILE_SYSTEM.remove((char *)upload.filename.c_str());
        }
        uploadFile = FILE_SYSTEM.open(upload.filename.c_str(), FILE_WRITE);
        DBG_OUTPUT_PORT.print("Upload: START, filename: ");
        DBG_OUTPUT_PORT.println(upload.filename);
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
        if (uploadFile)
        {
            uploadFile.write(upload.buf, upload.currentSize);
        }
        DBG_OUTPUT_PORT.print("Upload: WRITE, Bytes: ");
        DBG_OUTPUT_PORT.println(upload.currentSize);
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
        if (uploadFile)
        {
            uploadFile.close();
        }
        DBG_OUTPUT_PORT.print("Upload: END, Size: ");
        DBG_OUTPUT_PORT.println(upload.totalSize);
    }
}

void deleteRecursive(String path)
{
    File file = FILE_SYSTEM.open((char *)path.c_str());
    if (!file.isDirectory())
    {
        file.close();
        FILE_SYSTEM.remove((char *)path.c_str());
        return;
    }

    file.rewindDirectory();
    while (true)
    {
        File entry = file.openNextFile();
        if (!entry)
        {
            break;
        }
        String entryPath = path + "/" + entry.name();
        if (entry.isDirectory())
        {
            entry.close();
            deleteRecursive(entryPath);
        }
        else
        {
            entry.close();
            FILE_SYSTEM.remove((char *)entryPath.c_str());
        }
        yield();
    }

    FILE_SYSTEM.rmdir((char *)path.c_str());
    file.close();
}

void handleDelete()
{
    if (server.args() == 0)
    {
        return returnFail("BAD ARGS");
    }
    String path = server.arg(0);
    if (path == "/" || !FILE_SYSTEM.exists((char *)path.c_str()))
    {
        returnFail("BAD PATH");
        return;
    }
    deleteRecursive(path);
    returnOK();
}

void handleCreate()
{
    if (server.args() == 0)
    {
        return returnFail("BAD ARGS");
    }
    String path = server.arg(0);
    if (path == "/" || FILE_SYSTEM.exists((char *)path.c_str()))
    {
        returnFail("BAD PATH");
        return;
    }

    if (path.indexOf('.') > 0)
    {
        File file = FILE_SYSTEM.open((char *)path.c_str(), FILE_WRITE);
        if (file)
        {
            file.write(0);
            file.close();
        }
    }
    else
    {
        FILE_SYSTEM.mkdir((char *)path.c_str());
    }
    returnOK();
}

void handleGetPath()
{
    if (server.args() == 0)
    {
        return returnFail("BAD ARGS");
    }
    pic_path = server.arg(0);
    Serial.print("get pic path : ");
    Serial.println(pic_path.c_str());
    xEventGroupSetBits(handleServer, BIT_SHOW);
    returnOK();
}

void printDirectory()
{
    if (!server.hasArg("dir"))
    {
        return returnFail("BAD ARGS");
    }
    String path = server.arg("dir");
    if (path != "/" && !FILE_SYSTEM.exists((char *)path.c_str()))
    {
        return returnFail("BAD PATH");
    }
    File dir = FILE_SYSTEM.open((char *)path.c_str());
    path = String();
    if (!dir.isDirectory())
    {
        dir.close();
        return returnFail("NOT DIR");
    }
    dir.rewindDirectory();
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/json", "");
    WiFiClient client = server.client();

    server.sendContent("[");
    for (int cnt = 0; true; ++cnt)
    {
        File entry = dir.openNextFile();
        if (!entry)
        {
            break;
        }

        String output;
        if (cnt > 0)
        {
            output = ',';
        }

        output += "{\"type\":\"";
        output += (entry.isDirectory()) ? "dir" : "file";
        output += "\",\"name\":\"";
        output += entry.path();
        output += "\"";
        output += "}";
        server.sendContent(output);
        entry.close();
    }
    server.sendContent("]");
    dir.close();
}

void handleNotFound()
{
    if (hasFILE_SYSTEM && loadFromFILE_SYSTEM(server.uri()))
    {
        return;
    }
    String message = "FILE SYSTEM Not Detected\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++)
    {
        message += " NAME:" + server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
    DBG_OUTPUT_PORT.print(message);
}

void setup()
{
    handleServer = xEventGroupCreate();
    DBG_OUTPUT_PORT.begin(115200);
    DBG_OUTPUT_PORT.setDebugOutput(true);
    DBG_OUTPUT_PORT.print("\n");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    DBG_OUTPUT_PORT.print("Connecting to ");
    DBG_OUTPUT_PORT.println(ssid);

    // Wait for connection
    uint8_t i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 20)
    { // wait 10 seconds
        delay(500);
    }
    if (i == 21)
    {
        DBG_OUTPUT_PORT.print("Could not connect to");
        DBG_OUTPUT_PORT.println(ssid);
        while (1)
        {
            delay(500);
        }
    }
    DBG_OUTPUT_PORT.print("Connected! IP address: ");
    DBG_OUTPUT_PORT.println(WiFi.localIP());

    if (MDNS.begin(host))
    {
        MDNS.addService("http", "tcp", 80);
        DBG_OUTPUT_PORT.println("MDNS responder started");
        DBG_OUTPUT_PORT.print("You can now connect to http://");
        DBG_OUTPUT_PORT.print(host);
        DBG_OUTPUT_PORT.println(".local");
    }

    server.on("/list", HTTP_GET, printDirectory);
    server.on("/edit", HTTP_DELETE, handleDelete);
    server.on("/edit", HTTP_PUT, handleCreate);
    server.on(
        "/edit", HTTP_POST, []()
        { returnOK(); },
        handleFileUpload);
    server.on("/clean", HTTP_POST, []()
              { Serial.println("Get clean msg");
              xEventGroupSetBits(handleServer,BIT_CLEAN);
              returnOK(); });
    server.on("/show", HTTP_POST, handleGetPath);
    server.onNotFound(handleNotFound);
    server.begin();
    DBG_OUTPUT_PORT.println("HTTP server started");

    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (FILE_SYSTEM.begin(SD_CS, SPI))
    {
        DBG_OUTPUT_PORT.println("FILE_SYSTEM Card initialized.");
        hasFILE_SYSTEM = true;
    }

    /* Initialize the screen */
    epd_init();
    libjpeg_init();
    framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
    if (!framebuffer)
    {
        Serial.println("alloc memory failed !!!");
        while (1)
            ;
    }
    memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
    epd_poweron();
    epd_clear();

    Rect_t area = {
        .x = 256,
        .y = 180,
        .width = logo_width,
        .height = logo_height,
    };

    epd_poweron();
    epd_clear();
    // epd_draw_grayscale_image(area, (uint8_t *)logo_data);
    epd_draw_image(area, (uint8_t *)logo_data, BLACK_ON_WHITE);
    epd_poweroff();
}

void loop()
{
    server.handleClient();
    delay(2); // allow the cpu to switch to other tasks
    EventBits_t bit = xEventGroupGetBits(handleServer);
    if (bit & BIT_CLEAN)
    {
        xEventGroupClearBits(handleServer, BIT_CLEAN);
        epd_poweron();
        epd_clear();
        epd_poweroff();
    }
    else if (bit & BIT_SHOW)
    {
        xEventGroupClearBits(handleServer, BIT_SHOW);
        epd_poweron();
        File jpg = FILE_SYSTEM.open(pic_path);
        String jpg_p;
        while (jpg.available())
            jpg_p += jpg.readString();
        Rect_t rect = {
            .x = 0,
            .y = 0,
            .width = EPD_WIDTH,
            .height = EPD_HEIGHT,
        };
        show_jpg_from_buff((uint8_t *)jpg_p.c_str(), jpg_p.length(), rect);
        Serial.printf("jpg w:%d,h:%d\r\n", rect.width, rect.height);
        epd_poweroff();
    }
}

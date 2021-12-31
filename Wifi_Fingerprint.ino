#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include "sha256.h"
#include <SPI.h>
#include <Adafruit_Fingerprint.h>

#define mySerial Serial1
#define HASH_SIZE 32
#define BLOCK_SIZE 64

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

char ssid[] = "modian";
char pass[] = "12345678";
int keyIndex = 0;

int status = WL_IDLE_STATUS;
char server[] = "192.168.43.181";

WiFiClient client;
WiFiUDP Udp;
unsigned int localPort = 8888;

static const char ntpServerName[] = "cn.pool.ntp.org";
//static const char ntpServerName[] = "modianpi.local";
const int timeZone = 0;     // Central European Time
String secret ="61b6e94028b1159be15904eeeb10e2a9";

void setup() {
  Serial.begin(9600);

// 设置指纹识别
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }
  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
  
  //连接WIFI
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.begin(9600);
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");

  // 校准时间
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);

  
  
}

void loop() {
  getFingerprintID();
  delay(50);            //don't ned to run this at full speed.  
}


uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
//      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  if (timeStatus() != timeNotSet) {
    const time_t time1 = now();
    String time_str = String(time1);
    Serial.println(time_str);
    String message = "Open" + time_str + secret;
    String signature = sha256(message);
    Serial.println(signature);
    
    Serial.println("Starting connection to server...");
    
    if (client.connect(server, 8443)) {
      Serial.println("Connected to server");
      client.println("GET /open_door?type=Arduino&timestamp=" + time_str + "&signature=" + signature + " HTTP/1.1");
      client.println("Host: " + String(server));
      client.println("Connection: close");
      client.println();
      
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
      }
    }

    
  } else {
    Serial.println("Time is not set.");
  }
    
  return finger.fingerID;
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
String sha256(String input) {
  Sha256 sha256 = Sha256();
  uint8_t *hash;
  sha256.init();
  sha256.print(input);
  hash = sha256.result();
  char str[65] = "";
  array_to_string(hash, 32, str);
  return String(str);
}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'a' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'a' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

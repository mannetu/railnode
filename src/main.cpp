// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>
#include <LiquidCrystal.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 1                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

long timeLastMsgSend = 0;

void setup() {

  //Serial.begin(115200);

  lcd.begin(16, 2);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and
  // the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
  {
    Serial.println("MCP2515 Initialized Successfully!");
    lcd.print("CAN Initialized");
  }
  else
  {
    Serial.println("Error Initializing MCP2515...");
    lcd.print("CAN Init Error");
  }

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);

  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT);

  Serial.println("MCP2515 Library Receive Example...");

}

void loop() {

  if (!digitalRead(CAN0_INT)) { // If CAN0_INT pin is low, read receive buffer

    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000){     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "ExtID 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);}
      else {
        sprintf(msgString, "StdID: 0x%.3lX       DLC: %1d  Data:", rxId, len);}

        Serial.print(msgString);
        lcd.clear();
        lcd.print(msgString);
        lcd.setCursor(0, 1);

        if((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
          sprintf(msgString, " REMOTE REQUEST FRAME");
          Serial.print(msgString);
        } else {

          for (byte i = 0; i<len; i++) {
            sprintf(msgString, "0x%.2X ", rxBuf[i]);
            Serial.print(msgString);
            lcd.print(msgString);

          }
          Serial.println();
        }
      }


  if (!digitalRead(8) && millis() - timeLastMsgSend > 2000)
  {
    // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);

    if(sndStat == CAN_OK)
    {
      Serial.print("Message Sent Successfully!  ");
      Serial.println(data[0]);
    }
    else
    {
      Serial.println("Error Sending Message...");
    }
    data[0]++;
    if (data[0] == 11) data[0] = 0;
    timeLastMsgSend = millis();
  }
}


    /*********************************************************************************************************
    END FILE
    *********************************************************************************************************/

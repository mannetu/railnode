/*
  railnode
  CAN bus node
*/

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <mcp23s08.h>

/*
  CAN address masks
*/
#define ADDR_TURN  0x400 // Turnout Set
#define ADDR_TUFB  0x500 // Turnout Feedback
#define ADDR_SIGN  0x600 // Signal Set

#define NODE_NUMBER 0


#define CAN0_INT 9                              // Set INT to pin 9
MCP_CAN CAN0(10);                               // Set CS to pin 10
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define MCP_CSPIN  8
#define MCP_ADRS 0x20
#define INTused 0
#define INTpin 2
volatile boolean turnout_switched;
mcp23s08 mcp(MCP_CSPIN,MCP_ADRS);


/*
  Receive CAN message, decide what to do, and switch GPIOs
*/

int isMessage()
{
  if (!digitalRead(CAN0_INT))  // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "ExtID 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else sprintf(msgString, "StdID: 0x%.3lX  DLC: %1d  Data:", rxId, len);
    Serial.print(msgString);

    if((rxId & 0x40000000) == 0x40000000)   // Determine if message is a remote request frame.
    {
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    }
    else
    {
      for (byte i = 0; i<len; i++)
      {
        sprintf(msgString, "0x%.2X ", rxBuf[i]);
        Serial.print(msgString);
      }
    }
    Serial.println();
    return 1;
  }
  return 0;
}

int switch_turnouts(int ch, int state) {
  switch (ch)
  {
    case 0:
      if (state == 0)
      {
        digitalWrite(3, 1);
        delay(500);
        digitalWrite(3, 0);
      }
      if (state == 1)
      {
        digitalWrite(4, 1);
        delay(500);
        digitalWrite(4, 0);
      }
      return 0;
  }
  return -1;
}

int switch_signals(int ch, int state) {
  switch (ch)
  {
    case 0:
      if (state == 0)
      {

      }
      if (state == 1)
      {

      }
      return 0;
  }
  return -1;
}

int checkMessage()
{
  if ((rxId & 0xF0) ^ (NODE_NUMBER << 4)) return -1;
  int ch = (rxId & 0xF);
  int state = rxBuf[0];
  if (((rxId & 0xF00) ^ ADDR_TURN) == 0) switch_turnouts(ch, state);
  if (((rxId & 0xF00) ^ ADDR_SIGN) == 0) switch_signals(ch, state);
  return 0;
}

/*
  Handle GPIOs for Turnout feedback
*/

void turnout_manually_switch ()
{
  turnout_switched = true;
}

int sendMessage(byte channel, byte state) {
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte data[1] = {state};
  byte sndStat = CAN0.sendMsgBuf((ADDR_TUFB + channel), 0, 1, data);

  if(sndStat == CAN_OK)
  {
    //Serial.print("Feedback-Message Sent!  ");
    //Serial.println(channel); Serial.println();
    return 0;
  }
  else
  {
    Serial.println("Error Sending Feedback-Message..."); Serial.println();
    return -1;
  }
}

void handle_turnout_switched ()
{
  detachInterrupt(INTused);//protect from further interrupts
  uint8_t register_value = 0;
  uint8_t gpio_value = 0;
  delay (100);  // de-bounce before we re-enable interrupts
  register_value |= mcp.gpioRegisterReadByte(mcp.INTF);
  gpio_value |= mcp.gpioRegisterReadByte(mcp.GPIO);
  if (register_value)
  {
    //Serial.print("interupt\n");
    //Serial.println(gpio_value, BIN);
    // which turnout caused interrupt?
    uint8_t turnout_ch = 0;
    if ((register_value & (1 << 0)) || (register_value & (1 << 1))) turnout_ch = 0;
    if ((register_value & (1 << 2)) || (register_value & (1 << 3))) turnout_ch = 1;
    if ((register_value & (1 << 4)) || (register_value & (1 << 5))) turnout_ch = 2;
    if ((register_value & (1 << 6)) || (register_value & (1 << 7))) turnout_ch = 3;

    Serial.print("Turnout ");
    Serial.print(turnout_ch, DEC);
    Serial.print(" now switched");
    if (gpio_value & (0x1 << (turnout_ch*2)))
    {
      Serial.println(" to LEFT");
      sendMessage(turnout_ch, 0);
      mcp.gpioRegisterReadByte(mcp.INTCAP);  // clear INTF by readout INTCAP
      turnout_switched = false;
      attachInterrupt(INTused, turnout_manually_switch, FALLING);
      return;
      }

    if (gpio_value & (0x2 << (turnout_ch*2)))
    {
      Serial.println(" to RIGHT");
      sendMessage(turnout_ch, 1);
      mcp.gpioRegisterReadByte(mcp.INTCAP);  // clear INTF by readout INTCAP
      turnout_switched = false;
      attachInterrupt(INTused, turnout_manually_switch, FALLING);
      return;
    }

    Serial.print(": Error! GPIOs "); Serial.println(gpio_value, BIN);
    sendMessage(turnout_ch, 0xFF);
  }
  turnout_switched = false;
  attachInterrupt(INTused, turnout_manually_switch, FALLING);
}



void setup()
{
  Serial.begin(115200);

  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT);
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and
  // the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
  Serial.println("Error Initializing MCP2515...");
  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);
  Serial.println("MCP2515 Library Receive Example...");

  // Configuring GPIO MCP23S08
  mcp.begin();
  mcp.gpioRegisterWriteByte(mcp.IOCON,0b00101000);//set interrupt on GPIO and other parameters (see datasheet)
  mcp.gpioPinMode(0xFF);// Set all pins to be inputs
  mcp.gpioRegisterWriteByte(mcp.GPPU, 0x00);// pull-up resistor for switch
  mcp.gpioRegisterWriteByte(mcp.IPOL,0xFF);// invert polarity
  mcp.gpioRegisterWriteByte(mcp.GPINTEN,0xFF);// enable all interrupt
  mcp.gpioRegisterReadByte(mcp.INTCAP);// read from interrupt capture ports to clear them
  //now prepare interrupt pin on processor
  Serial.print(mcp.gpioRegisterReadByte(mcp.GPIO));
  pinMode (INTpin, INPUT);
  digitalWrite (INTpin, HIGH);
  turnout_switched = false;
  attachInterrupt(INTused, turnout_manually_switch, FALLING);

  // Set pins for Weichen
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
}


void loop()
{
  if (isMessage()) checkMessage();
  if (turnout_switched) handle_turnout_switched();
}


/*
  END FILE
*/

#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

/*
 * Alex's configuration constants
 */

volatile TDirection dir;

#define COUNTS_PER_REV (80 / 20) // 40-42 / 11 (adjust when need be)
#define WHEEL_CIRC (11 * PI)
#define ALEX_LENGTH 20
#define ALEX_BREADTH 16

float alexDiagonal = 0.0;
float alexCirc = 0.0;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks;

// Store the revolutions on Alex's left and right wheels
volatile unsigned long leftForwardTickTurns;
volatile unsigned long rightForwardTickTurns;
volatile unsigned long leftReverseTickTurns;
volatile unsigned long rightReverseTickTurns;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;
/*
 *
 * Alex Communication Routines.
 *
 */

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTickTurns;
  statusPacket.params[5] = rightForwardTickTurns;
  statusPacket.params[6] = leftReverseTickTurns;
  statusPacket.params[7] = rightReverseTickTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(const char *format, ...)
{
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Enable the pull-up resistors on pins 19 and 18.
  // These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  // Then activate internal pull-ups by writing 1 to them

  DDRD &= ~((1 << PD2) | (1 << PD3));
  PORTD |= (1 << PD2) | (1 << PD3);
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{

  if (dir == FORWARD)
  {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD)
  {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT)
  {
    leftReverseTickTurns++;
  }
  else if (dir == RIGHT)
  {
    leftForwardTickTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD)
  {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD)
  {
    rightReverseTicks++;
  }
  else if (dir == LEFT)
  {
    rightForwardTickTurns++;
  }
  else if (dir == RIGHT)
  {
    rightReverseTickTurns++;
  }
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Configure pins 18 and 19 to be on falling edge triggered
  EICRA |= 0b10100000;
  EIMSK |= 0b00001100;
}

// INT3 ISR should call leftISR while INT2 ISR should call rightISR.

ISR(INT3_vect)
{
  leftISR();
}
ISR(INT2_vect)
{
  rightISR();
}

/*
 * Setup and start codes for serial communications
 *
 */

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 *
 */

// Clears all our counters
void clearCounters()
{

  leftForwardTicks = 0;
  leftReverseTicks = 0;
  rightForwardTicks = 0;
  rightReverseTicks = 0;

  // Store the revolutions on Alex's left and right wheels
  leftForwardTickTurns = 0;
  rightForwardTickTurns = 0;
  leftReverseTickTurns = 0;
  rightReverseTickTurns = 0;

  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
  // For movement commands, param[0] = distance, param[1] = speed.
  case COMMAND_FORWARD:
    sendOK();
    forward((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_REVERSE:
    sendOK();
    backward((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_TURN_LEFT:
    sendOK();
    ccw((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_TURN_RIGHT:
    sendOK();
    cw((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_STOP:
    sendOK();
    stop();
    break;
  case COMMAND_GET_STATS:
    sendOK();
    sendStatus();
    break;
  case COMMAND_CLEAR_STATS:
    sendOK();
    clearOneCounter(command->params[0]);
    break;
  default:
    sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
  case PACKET_TYPE_COMMAND:
    handleCommand(packet);
    break;

  case PACKET_TYPE_RESPONSE:
    break;

  case PACKET_TYPE_ERROR:
    break;

  case PACKET_TYPE_MESSAGE:
    break;

  case PACKET_TYPE_HELLO:
    break;
  }
}

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}




void setup()
{
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();

 // moveOneMotor();
 // getTickPerRotation();
}

void loop()
{

  TPacket recvPacket;

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
  {
    handlePacket(&recvPacket);
  }
  else
  {
    if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    }
  }

  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == (TDirection)STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTickTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else
    {
      if (dir == RIGHT)
      {
        if (rightReverseTickTurns >= targetTicks)
        {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
      else if (dir == (TDirection)STOP)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
  }
}

// check distance
// check angle turning

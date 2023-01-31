#ifndef MAIN_H
#define MAIN_H

#include "FreeRTOS.h"
#include "ring.h"

/*
DESCRIPTION OF THE PROTOCOL

*To implement means functions, variables and magnitude values which are present
on the protocol but function definitions and values need to be provided
in the way most convenient and functional for the specific hardware 
characteristics in which the implementation is meant to work.

*Implementation dependent means functions, variables and magnitude values which
are not present on the protocol but which 
functionality is contemplated by the protocol but which implementation can
not be encapsulated in universal fixed declarations and namings, thus
specific functions, variables and magnitude values should be created freely
in order to comply with the protocol specified functionality.

*Custom means functions, variables and magnitude values which are not present
in the protocol and which functionality is not expected by the protocol,
however a particular implementation may find convinient or necessary to
implement.

The protocol is based on the serial communication relationship there exists
between two devices (which may be implemented in any known way which can accomodate
serial communications):
1.- The encoder device where this program is loaded
2.- The encoder controler device which talks to the encoder device

The communication is done through messages which have data containing a code as 
well as parameters depending on the sent code. Codes indicate the message type and
who it sent for, the encoder controller uses messages to command the encoder
device what to do and the encoder device uses them to report information to the 
encoder controller depending on what the controller commanded. Parameters may be
sent for if the message type is required to provide further information on the 
factors to take into account for the command execution or the data being reported. 

Data may be sent along with the code to specify parameters for command execution
as well as for information being reported. Communication starts only after a 
connection is established and finishes after a disconnection occurs.

Every message containting a code should  contain the following characteristics:

1.- 0 should be interpreted as a null terminator always, henceforth no 0's 
    other than null characters should be used in the implementation. This is
    so because characters of 0 are to be interpreted as null characters and
    as such might notify there are no more characters to be parsed or buffered.
    In this way compatibility and reliability is assured with different possible
    implementations.

2.- There should be a character at the beginning and end of each payload as to 
    indicate where messages begin and end. 

    These initator/terminator characters should have a binary representation 
    different than any value which may appear in the payload. To ensure this, 
    every byte should be encoded with a special format which differentiates
    wheter bytes are initiator/terminator related or data related, the first 
    bit with a value of 1 may be used for data types and with a value of 0 for
    initiator/terminator types. This would effectively reduce the amount of 
    data that can be sent per byte to just 2^7. Shall a number bigger than 2^7
    be sent this number might be split each 7 bits and encoded according to 
    the just mentioned rule.

4.- The first byte after the initiator character should correspond to the code
    of the message.

    Encoder controller codes should only be written by the encoder device, while
    encoder device codes should only be writen by the encoder controller, this way 
    directionality is achieved, eg: Should a message be received with an encoder 
    controller code in the encoder controller then the message can be excluded 
    and be assumed as a notification of encoder controller activities,
    the same behaviour can be expected with the encoder device.


The following functionalities should be implemented:
1.- The ability to detect connection/disconnection status
2.- The ability to receive encoder device messages
3.- The ability to transmit encoder controller messages requested from a queue

4.- The ability to interrupt the cpu when the encoder is connected to an AC
    adpater
5.- The ability to interrupt the cpu when the encoder counter changes
    capturing both time and encoder counter

The naming of every procotol implementation 
MCU_ANGLE_SerialProtocolImplementation


humanInterfaceDisplayTask
encoderControllerMessagesTransmissionTask

 */

/*
displayInterface
commandTXInterface
commandRXInterface
 */

//Mathematical defs--------------------------------
#define BITS31 2147483648.0f
#define PI 3.14159265f
#define degToRads(x) ((x) * PI)/180.0

//-------------------------PROTOCOL--------------------------
//*******************************************************************

//To implement ENCODER hardware specs-------------------------
#define ENCODERINITIAL_VALUE 32767
#define ENCODERSTEPDISTANCEINMILLS 4084

//To implement Queue sizes------------------------------------
#define HUMAN_INTERFACE_DISPLAY_REQUEST_QUEUE_SIZE        20
#define MESSAGES_TX_REQUEST_QUEUE_SIZE                    20

//To implement BUFFER sizes-----------------------------------
#define ENCODER_BUFFER_SIZE 256

#if ((ENCODER_BUFFER_LEN - 1) & ENCODER_BUFFER_LEN) == 0
#else
#warning ENCODER_BUFFER NOT POWER OF 2
#endif

//To implement TIMING ----------------------------------------
#define ENCODER_TASK_DELAY_MS 50
#define BATTERY_FREE_TASK_DELAY_MS 20000

//Protocol communication message codes-----------
typedef enum Protocol_Communication_Message_Codes_t{
  forEncoderDevicePCMCode_encoderStart = 1,
  forEncoderDevicePCMCode_encoderStop = 2,
  forEncoderDevicePCMCode_encoderBattery = 3,
  forEncoderControllerPCMCode_encoderData = 64,
  forEncoderControllerPCMCode_encoderStart = 65,
  forEncoderControllerPCMCode_encoderBattery = 66,
  forEncoderControllerPCMCode_encoderStop = 67
}Protocol_Communication_Message_Codes_t;

//Queue handles----------------------------------
extern QueueHandle_t humanInterfaceDisplayRequestQueue;
extern QueueHandle_t messagesTXRequestQueue;

//Queue structures----------------------------------------
typedef enum humanInterfaceDisplayRequest_Codes{
  humanInterfaceDisplayRequestCode_startup,
  humanInterfaceDisplayRequestCode_connectionStatus,
  humanInterfaceDisplayRequestCode_batteryLevel,
  humanInterfaceDisplayRequestCode_chargingStatus
} humanInterfaceDisplayRequest_Codes;

typedef struct humanInterfaceDisplayRequest_Data{
  humanInterfaceDisplayRequest_Codes humanInterfaceDisplayRequest_Code;
  uint32_t displayValue;  
}humanInterfaceDisplayRequest_Data;

typedef enum messagesTXRequest_Codes{
  messagesTXRequestCode_encoderData,
  messagesTXRequestCode_start,
  messagesTXRequestCode_Battery,
  messagesTXRequestCode_Stop
}messagesTXRequest_Codes;

typedef struct messagesTXRequest_Data{
  messagesTXRequest_Codes messageTXRequest_Code;
  uint16_t traveledDistanceOrBattery;
  uint16_t meanPropulsiveVelocity;
  uint16_t peakVelocity;
} messagesTXRequest_Data;

//Tasks Handles-----------------------------------
extern TaskHandle_t encoderTaskHandle;
extern TaskHandle_t batteryFreeTaskHandle;
extern TaskHandle_t batteryWaitTaskHandle;

//Task structures---------------------------------
typedef struct encoderTaskParamTypes_t{
  uint32_t minDistToTravel;
  uint32_t desiredCounterDirection;
  uint32_t desiredRepDirection;
}encoderTaskParamTypes_t;

//Ring buffer handles-----------------------------
extern ring_t encoder_ring;

//Ring buffer structures--------------------------
typedef struct encoderValues_t{
  uint16_t encoderCounter;
  uint32_t inputCapture;
} encoderValues_t;

//Buffers-----------------------------------------
extern encoderValues_t encoder_buffer[ENCODER_BUFFER_SIZE];

//encoderTask variables---------------------------
extern uint8_t (*goingDesiredCountDir[2])(uint32_t, uint32_t);
extern uint8_t (*hasTraveledMinDist[2])(uint32_t, uint32_t);
extern uint8_t (*hasReturnedToInitial[2])(uint32_t, uint32_t);
extern uint8_t (*newMaxRomDetected[2])(uint32_t,uint32_t);

//ISR variables-----------------------------------
extern volatile encoderValues_t encInterruptValues;
extern volatile uint16_t encoderTimerCounter;
extern volatile uint32_t capturedTime;
extern volatile uint32_t overflowCounter;

//Helper functions--------------------------------
void sendToMessagesTXRequestQueue(messagesTXRequest_Codes code,
				  uint16_t traveledDistanceOrBattery,
				  uint16_t meanPropulsiveVelocity,
				  uint16_t peakVelocity);
void sendToHumanInterfaceDisplayRequestQueue(humanInterfaceDisplayRequest_Codes code,
					     uint32_t displayValue);
void createTask(TaskFunction_t pvTaskCode,
		const char *const pcName,
		configSTACK_DEPTH_TYPE usStackDepth,
		void *pvParameters,
		UBaseType_t uxPriority,
		TaskHandle_t *taskHandle);
void deleteTask(TaskHandle_t *pxTask);
void encodeTwoBytes(char *twoByteBuffer, uint32_t numberToEncode);
uint16_t decodeTwoBytes(uint8_t msb, uint8_t lsb);
void encodeOneByte(char *oneByteBuffer, unsigned int numberToEncode);
uint8_t decodeOneByte(uint8_t byte);

//To implement helper functions
void initializeTimers(void);
void stopTimers(void);
void turnOnEncoderSensors(void);
void turnOffEncoderSensors(void);
uint32_t readBattery(void);

//EncoderHelper functions-------------------------
uint8_t descendente(uint32_t a, uint32_t b);
uint8_t ascendente(uint32_t a, uint32_t b);
uint8_t minDistTraveledDes(uint32_t counter, uint32_t minDistToTravel);
uint8_t minDistTraveledAs(uint32_t counter, uint32_t minDistToTravel);
uint8_t returnedToInitialDes(uint32_t counter, uint32_t minDistToTravel);
uint8_t returnedToInitialAs(uint32_t counter, uint32_t minDistToTravel);
uint8_t maxRomAs(uint32_t counter, uint32_t lastMaxDist);
uint8_t maxRomDes(uint32_t counter, uint32_t lastMaxDist);

//Processes---------------------------------------
void encoderTask(void *args);
void batteryFreeTask(void *args);
void batteryWaitTask(void *args);

//To implement processes
void humanInterfaceDisplayRequestTask(void *args);

//*******************************************************************

//-----------------PROTOCOL IMPLEMENTATION DEPENDENT-----------------
//*******************************************************************

//TX/RX protocol delimiters----------------------
#define PROTOCOL_INITIATOR 0b01111100 // '|'
#define PROTOCOL_TERMINATOR PROTOCOL_INITIATOR

//Command RXInterfaceProcess handles-------------
extern QueueSetHandle_t communicationQueueSet;//CHANGE
extern SemaphoreHandle_t communicationSemaphore;//CHANGE

//Implementation dependent proceses--------------
void communicationTask(void *args);//CHANGE

//Interrupt handlers-----------------------------
void tim1_cc_isr(void);
void exti15_10_isr(void);

//*******************************************************************

//---------------------------CUSTOM----------------------------------
//*******************************************************************
#define SEMAPHORE_SIZE                 1
#define COMMUNICATION_QUEUE_SET_SIZE  HUMAN_INTERFACE_DISPLAY_REQUEST_QUEUE_SIZE \
  + SEMAPHORE_SIZE

#define UART_RX_BUFFER_SIZE 500//CHANGE
#define PARSE_BUFFER_SIZE   70//CHANGE

//Custom Buffers---------------------------------
extern char receiveBuffer[UART_RX_BUFFER_SIZE];//CHANGE
extern char parseBuffer[PARSE_BUFFER_SIZE];//CHANGE

//Custom Buffers positions-----------------------
extern uint32_t receiveBufferPos;//CHANGE

//Custom helper functions------------------------
void printStringUART(const char myString[]);

void cleanAdvanceBuffer(char *buffer, uint32_t *bufferPosition, uint32_t bufferLength);
void getLine(void);

//Custom interrupt handlers----------------------
void tim1_up_isr(void);
void usart1_isr(void);
//*******************************************************************


/* DWT (Data Watchpoint and Trace) registers, only exists on ARM Cortex with a DWT unit */
#define KIN1_DWT_CONTROL             (*((volatile uint32_t*)0xE0001000))
/*!< DWT Control register */
#define KIN1_DWT_CYCCNTENA_BIT       (1UL<<0)
/*!< CYCCNTENA bit in DWT_CONTROL register */
#define KIN1_DWT_CYCCNT              (*((volatile uint32_t*)0xE0001004))
/*!< DWT Cycle Counter register */
#define KIN1_DEMCR                   (*((volatile uint32_t*)0xE000EDFC))
/*!< DEMCR: Debug Exception and Monitor Control Register */
#define KIN1_TRCENA_BIT              (1UL<<24)
/*!< Trace enable bit in DEMCR register */

#define KIN1_InitCycleCounter()			\
  KIN1_DEMCR |= KIN1_TRCENA_BIT
/*!< TRCENA: Enable trace and debug block DEMCR (Debug Exception and Monitor Control Register */
 
#define KIN1_ResetCycleCounter()		\
  KIN1_DWT_CYCCNT = 0
/*!< Reset cycle counter */
 
#define KIN1_EnableCycleCounter()		\
  KIN1_DWT_CONTROL |= KIN1_DWT_CYCCNTENA_BIT
/*!< Enable cycle counter */
 
#define KIN1_DisableCycleCounter()		\
  KIN1_DWT_CONTROL &= ~KIN1_DWT_CYCCNTENA_BIT
/*!< Disable cycle counter */
 
#define KIN1_GetCycleCounter()			\
  KIN1_DWT_CYCCNT
/*!< Read cycle counter register */

#endif



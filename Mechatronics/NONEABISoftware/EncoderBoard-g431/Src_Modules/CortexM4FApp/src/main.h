#ifndef MAIN_H
#define MAIN_H

//Mathematical defs--------------------------
#define BITS31 2147483648.0f
#define PI 3.14159265f
#define degToRads(x) ((x) * PI)/180.0

//UART RX------------------------------------
#define UART_RX_BUFFER_LEN 256
extern char receiveBuffer[UART_RX_BUFFER_LEN];

//Queue handles----------------
/*
extern QueueHandle_t uartTXQueue;
extern QueueHandle_t lcdQueue;
*/

//Queue sizes------------------
#define LCD_QUEUE_SIZE                20
#define TX_QUEUE_SIZE                 20

//Queue structures---------------------------
typedef enum messageTypes_t{
			    encoderData,
			    encoderStart,
			    encoderStop,
			    connectedStatus,
			    batteryLevel,
			    chargingStatus
}messageTypes_t;

typedef struct uartTXData_t{
  messageTypes_t messageType;
  uint16_t traveledDistanceOrADC;
  uint16_t meanPropulsiveVelocity;
  uint16_t peakVelocity;
} uartTXData_t;

typedef struct lcdData_t{
  messageTypes_t messageType;
  uint32_t displayValue;  
}lcdData_t;

//Tasks Handles------------------------------
/*
extern TaskHandle_t encoderTaskHandle;
extern TaskHandle_t uartRXTaskHandle;
extern TaskHandle_t adcFreeTaskHandle;
extern TaskHandle_t adcWaitTaskHandle;
*/

//Task structures----------------------------
typedef struct encoderTaskParamTypes_t{
  uint32_t minDistToTravel;
  uint16_t desiredCounterDirection;
  uint16_t desiredRepDirection;

}encoderTaskParamTypes_t;

//Helper functions---------------------------
void sendToUARTTXQueue(messageTypes_t messageType,
		       uint16_t traveledDistanceOrADC,
		       uint16_t meanPropulsiveVelocity,
		       uint16_t peakVelocity);
void sendToLCDQueue(messageTypes_t messageType,
		    uint32_t displayValue);
void printStringUART(const char myString[]);

void configure_I2C_IndependentClock_Source(void);
void enable_I2C_Clock(void);
void disable_I2C_Clock(void);
void configure_I2C_Periphereal(void);
void enable_I2C_GPIO_Clock(void);
void configure_I2C_GPIO_Source(void);

#endif

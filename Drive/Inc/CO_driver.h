/*
 * CAN module driver for ST STM32F103 microcontroller.
 */

#ifndef CO_DRIVER_H
#define CO_DRIVER_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Critical sections */
#define CO_LITTLE_ENDIAN
#define CO_LOCK_CAN_SEND()      __set_PRIMASK(1);
#define CO_UNLOCK_CAN_SEND()    __set_PRIMASK(0);

#define CO_LOCK_EMCY()          __set_PRIMASK(1);
#define CO_UNLOCK_EMCY()        __set_PRIMASK(0);

#define CO_LOCK_OD()            __set_PRIMASK(1);
#define CO_UNLOCK_OD()          __set_PRIMASK(0);

/* Data types */
typedef _Bool bool_t;

typedef float                   float32_t;
typedef long double             float64_t;
typedef char                    char_t;
typedef unsigned char           oChar_t;
typedef unsigned char           domain_t;

/* Return values */
typedef enum{
	CO_ERROR_NO                 = 0,
	CO_ERROR_ILLEGAL_ARGUMENT   = -1,
	CO_ERROR_OUT_OF_MEMORY      = -2,
	CO_ERROR_TIMEOUT            = -3,
	CO_ERROR_ILLEGAL_BAUDRATE   = -4,
	CO_ERROR_RX_OVERFLOW        = -5,
	CO_ERROR_RX_PDO_OVERFLOW    = -6,
	CO_ERROR_RX_MSG_LENGTH      = -7,
	CO_ERROR_RX_PDO_LENGTH      = -8,
	CO_ERROR_TX_OVERFLOW        = -9,
	CO_ERROR_TX_PDO_WINDOW      = -10,
	CO_ERROR_TX_UNCONFIGURED    = -11,
	CO_ERROR_PARAMETERS         = -12,
	CO_ERROR_DATA_CORRUPT       = -13,
	CO_ERROR_CRC                = -14
}CO_ReturnError_t;

/* CAN receive message structure as aligned in CAN module. */
typedef struct{
	uint32_t    ident;          /* Standard Identifier */
	uint32_t    ExtId;          /* Specifies the extended identifier */
	uint8_t     IDE;            /* Specifies the type of identifier for the
																 message that will be received */
	uint8_t     RTR;            /* Remote Transmission Request bit */
	uint8_t     DLC;            /* Data length code (bits 0...3) */
	uint8_t     data[8];        /* 8 data bytes */
	uint8_t     FMI;            /* Specifies the index of the filter the message
																 stored in the mailbox passes through */
}CO_CANrxMsg_t;


/* Received message object */
typedef struct{
	uint16_t            ident;
	uint16_t            mask;
	void               *object;
	void              (*pFunct)(void *object, const CO_CANrxMsg_t *message);
}CO_CANrx_t;


/* Transmit message object. */
typedef struct{
	uint32_t            ident;
	uint8_t             DLC;
	uint8_t             data[8];
	volatile uint8_t    bufferFull;
	volatile uint8_t    syncFlag;
}CO_CANtx_t;/* ALIGN_STRUCT_DWORD; */


/* CAN module object. */
typedef struct{
	CAN_HandleTypeDef  *hcan;
	int32_t             CANbaseAddress;
	CO_CANrx_t         *rxArray;
	uint16_t            rxSize;
	CO_CANtx_t         *txArray;
	uint16_t            txSize;
	volatile bool_t     CANnormal;
	volatile uint8_t    useCANrxFilters;
	volatile uint8_t    bufferInhibitFlag;
	volatile uint8_t    firstCANtxMessage;
	volatile uint16_t   CANtxCount;
	uint32_t            errOld;
	void               *em;
}CO_CANmodule_t;

/* Functions */
void CO_CANsetConfigurationMode(int32_t CANbaseAddress);

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule);

CO_ReturnError_t CO_CANmodule_init(
	CO_CANmodule_t         *CANmodule,
	int32_t                 CANbaseAddress,
	CO_CANrx_t              rxArray[],
	uint16_t                rxSize,
	CO_CANtx_t              txArray[],
	uint16_t                txSize,
	uint16_t                CANbitRate);
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule);

CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        int8_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message));
				
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        int8_t                  rtr,
        uint8_t                 noOfBytes,
        int8_t                  syncFlag);
int8_t getFreeTxBuff(CO_CANmodule_t *CANmodule);
				
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);
				
static void CO_CANsendToModule(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer, uint8_t transmit_mailbox);

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule);

void CO_CANverifyErrors(CO_CANmodule_t *CANmodule);
				
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);

void CO_CANinterrupt_Rx(CO_CANmodule_t *CANmodule);

void CO_CANinterrupt_Tx(CO_CANmodule_t *CANmodule);

#endif

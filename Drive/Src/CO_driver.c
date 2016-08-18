/*
 * CAN module driver for ST STM32F103 microcontroller.
 */

#include "CANopen.h"
#include "CO_Emergency.h"

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress){
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    CANmodule->CANnormal = true;
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
	CO_CANmodule_t         *CANmodule,
	int32_t                 CANbaseAddress,
	CO_CANrx_t              rxArray[],
	uint16_t                rxSize,
	CO_CANtx_t              txArray[],
	uint16_t                txSize,
	uint16_t                CANbitRate)
{
	CAN_FilterConfTypeDef sFilterConfig;
	int i;
	HAL_StatusTypeDef result;

	/* verify arguments */
	if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
		return CO_ERROR_ILLEGAL_ARGUMENT;
	} 
	
	CANmodule->hcan = &hcan;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = false;
	CANmodule->bufferInhibitFlag = 0;
	CANmodule->firstCANtxMessage = 1;
	CANmodule->CANtxCount = 0;
	CANmodule->errOld = 0;
	CANmodule->em = 0; 
	
	for (i = 0; i < rxSize; i++)
	{
	  CANmodule->rxArray[i].ident = 0;
	  CANmodule->rxArray[i].pFunct = 0;
	}
	for (i = 0; i < txSize; i++)
	{
	  CANmodule->txArray[i].bufferFull = 0;
	} 
	
	/* Init CAN */
	if (CANmodule->hcan->State != HAL_CAN_STATE_RESET) {
		HAL_CAN_DeInit(CANmodule->hcan);
	}
	CANmodule->hcan->Instance = CAN1;
	switch (CANbitRate)
	{
		case 1000: CANmodule->hcan->Init.Prescaler = 2;
			break;
		case 500: CANmodule->hcan->Init.Prescaler = 4;
			break;
		default:
		case 250: CANmodule->hcan->Init.Prescaler = 8;
			break;
		case 125: CANmodule->hcan->Init.Prescaler = 16;
			break;
		case 100: CANmodule->hcan->Init.Prescaler = 20;
			break;
		case 50: CANmodule->hcan->Init.Prescaler = 40;
			break;
		case 20: CANmodule->hcan->Init.Prescaler = 100;
			break;
		case 10: CANmodule->hcan->Init.Prescaler = 200;
			break;
	}
  CANmodule->hcan->Init.Mode = CAN_MODE_NORMAL;
  CANmodule->hcan->Init.SJW = CAN_SJW_4TQ;
  CANmodule->hcan->Init.BS1 = CAN_BS1_12TQ;
  CANmodule->hcan->Init.BS2 = CAN_BS2_5TQ;
  CANmodule->hcan->Init.TTCM = DISABLE;
  CANmodule->hcan->Init.ABOM = DISABLE;
  CANmodule->hcan->Init.AWUM = DISABLE;
  CANmodule->hcan->Init.NART = ENABLE;
  CANmodule->hcan->Init.RFLM = DISABLE;
  CANmodule->hcan->Init.TXFP = DISABLE;
  result = HAL_CAN_Init(CANmodule->hcan);
	if (result != HAL_OK) {
		return CO_ERROR_TIMEOUT;
	}
	
	memset(&sFilterConfig, 0, sizeof (CAN_FilterConfTypeDef));
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(CANmodule->hcan, &sFilterConfig); 
	
	HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	
	HAL_CAN_Receive_IT(CANmodule->hcan, CAN_FIFO0);
	
	return CO_ERROR_NO; 
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
  HAL_CAN_DeInit(CANmodule->hcan);
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        int8_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_CANrx_t *rxBuffer;
    uint16_t RXF, RXM;

    if (!CANmodule || !object || !pFunct || index >= CANmodule->rxSize)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
		
    rxBuffer = CANmodule->rxArray + index;

    //Configure object variables
    rxBuffer->object = object;
    rxBuffer->pFunct = pFunct;


    //CAN identifier and CAN mask, bit aligned with CAN module registers
    RXF = (ident & 0x07FF) << 2;
    if (rtr) RXF |= 0x02;
    RXM = (mask & 0x07FF) << 2;
    RXM |= 0x02;

    //configure filter and mask
    if (RXF != rxBuffer->ident || RXM != rxBuffer->mask)
    {
        rxBuffer->ident = RXF;
        rxBuffer->mask = RXM;
    }

    return CO_ERROR_NO;
} 

/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        int8_t                  rtr,
        uint8_t                 noOfBytes,
        int8_t                  syncFlag)
{
    uint32_t TXF;
    CO_CANtx_t *buffer;
	
    if (!CANmodule || CANmodule->txSize <= index) return 0;

    //get specific buffer
    buffer = &CANmodule->txArray[index];

    //CAN identifier, bit aligned with CAN module registers

    TXF = ident << 21;
    TXF &= 0xFFE00000;
    if (rtr) TXF |= 0x02;

    //write to buffer
    buffer->ident = TXF;
    buffer->DLC = noOfBytes;
    buffer->bufferFull = 0;
    buffer->syncFlag = syncFlag ? 1 : 0;

    return buffer;
} 

/******************************************************************************/
int8_t getFreeTxBuff(CO_CANmodule_t *CANmodule)
{
    uint8_t txBuff = CAN_TXMAILBOX_0;
	
    //if (CAN_TransmitStatus(CANmodule->hcan->Instance, txBuff) == CAN_TxStatus_Ok)
    for (txBuff = CAN_TXMAILBOX_0; txBuff <= (CAN_TXMAILBOX_2 + 1); txBuff++)
    {
        switch (txBuff)
        {
        case (CAN_TXMAILBOX_0 ):
            if (CANmodule->hcan->Instance->TSR & CAN_TSR_TME0 )
                return txBuff;
            else
                break;
        case (CAN_TXMAILBOX_1 ):
            if (CANmodule->hcan->Instance->TSR & CAN_TSR_TME1 )
                return txBuff;
            else
                break;
        case (CAN_TXMAILBOX_2 ):
            if (CANmodule->hcan->Instance->TSR & CAN_TSR_TME2 )
                return txBuff;
            else
                break;
				default:
						break;
        }
    }
    return -1;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CO_ReturnError_t err = CO_ERROR_NO;
	int8_t txBuff;

	/* Verify overflow */
	if(buffer->bufferFull)
	{
		if(!CANmodule->firstCANtxMessage)/* don't set error, if bootup message is still on buffers */
			CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
		err = CO_ERROR_TX_OVERFLOW;
	}

	CO_LOCK_CAN_SEND();
	//if CAN TB buffer0 is free, copy message to it
	 txBuff = getFreeTxBuff(CANmodule);
 // #error change this - use only one buffer for transmission - see generic driver
	if(txBuff != -1 && CANmodule->CANtxCount == 0)
	{
			CANmodule->bufferInhibitFlag = buffer->syncFlag;
			CO_CANsendToModule(CANmodule, buffer, txBuff);
	}
	//if no buffer is free, message will be sent by interrupt
	else
	{
			buffer->bufferFull = 1;
			CANmodule->CANtxCount++;
			__HAL_CAN_ENABLE_IT(CANmodule->hcan, CAN_IT_TME);
	}
	CO_UNLOCK_CAN_SEND();

	return err;
} 

/******************************************************************************/
static void CO_CANsendToModule(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer, uint8_t transmit_mailbox)
{
	CanTxMsgTypeDef* pTxMsg = CANmodule->hcan->pTxMsg;
	int i;

  pTxMsg->IDE = CAN_ID_STD;
	pTxMsg->DLC = buffer->DLC;
  for (i = 0; i < 8; i++) pTxMsg->Data[i] = buffer->data[i];
	pTxMsg->StdId = ((buffer->ident) >> 21);
  pTxMsg->RTR = CAN_RTR_DATA;

	HAL_CAN_Transmit_IT(CANmodule->hcan);
  __HAL_CAN_ENABLE_IT(CANmodule->hcan, CAN_IT_TME);
} 

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
	uint32_t tpdoDeleted = 0U;

	CO_LOCK_CAN_SEND();
	/* Abort message from CAN module, if there is synchronous TPDO.
	 * Take special care with this functionality. */
	if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
			/* clear TXREQ */
			CANmodule->bufferInhibitFlag = false;
			tpdoDeleted = 1U;
	}
	/* delete also pending synchronous TPDOs in TX buffers */
	if(CANmodule->CANtxCount != 0U){
			uint16_t i;
			CO_CANtx_t *buffer = &CANmodule->txArray[0];
			for(i = CANmodule->txSize; i > 0U; i--){
					if(buffer->bufferFull){
							if(buffer->syncFlag){
									buffer->bufferFull = false;
									CANmodule->CANtxCount--;
									tpdoDeleted = 2U;
							}
					}
					buffer++;
			}
	}
	CO_UNLOCK_CAN_SEND();


	if(tpdoDeleted != 0U){
			CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
	}
}

/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
   uint32_t err;
   CO_EM_t* em = (CO_EM_t*)CANmodule->em;

   err = CANmodule->hcan->Instance->ESR;
   // if(CAN_REG(CANmodule->hcan->Instance, C_INTF) & 4) err |= 0x80;

   if(CANmodule->errOld != err)
   {
      CANmodule->errOld = err;

      //CAN RX bus overflow
      if(CANmodule->hcan->Instance->RF0R & 0x08)
      {
         CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
         CANmodule->hcan->Instance->RF0R &=~0x08;//clear bits
      }

      //CAN TX bus off
      if(err & 0x04) CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
      else           CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

      //CAN TX or RX bus passive
      if(err & 0x02)
      {
         if(!CANmodule->firstCANtxMessage) CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
      }
      else
      {
        // int16_t wasCleared;
        /* wasCleared = */CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
        /* if(wasCleared == 1) */CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
      }


      //CAN TX or RX bus warning
      if(err & 0x01)
      {
         CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
      }
      else
      {
         CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
      }
   }
} 

/******************************************************************************/
// Interrupt from Receiver
void CO_CANinterrupt_Rx(CO_CANmodule_t *CANmodule) {
	CanRxMsgTypeDef* pRxMsg = CANmodule->hcan->pRxMsg;
	CO_CANrxMsg_t CanRxMsg;
	HAL_CAN_Receive_IT(CANmodule->hcan, CAN_FilterFIFO0);
	{
		uint16_t i;
		uint8_t msgMatched = 0;
		
		CanRxMsg.ident = pRxMsg->StdId;
		CanRxMsg.ExtId = pRxMsg->ExtId;
		CanRxMsg.IDE = pRxMsg->IDE;
		CanRxMsg.RTR = pRxMsg->RTR;
		CanRxMsg.DLC = pRxMsg->DLC;
		for (i = 0; i < 8; i++) CanRxMsg.data[i] = pRxMsg->Data[i];
		CanRxMsg.FMI = pRxMsg->FMI;		
		
		CO_CANrx_t *msgBuff = CANmodule->rxArray;
		for (i = 0; i < CANmodule->rxSize; i++)
		{
				uint16_t msg = (CanRxMsg.ident << 2) | (CanRxMsg.RTR ? 2 : 0);
				if (((msg ^ msgBuff->ident) & msgBuff->mask) == 0)
				{
						msgMatched = 1;
						break;
				}
				msgBuff++;
		}
		//Call specific function, which will process the message
		if (msgMatched && msgBuff->pFunct)
			msgBuff->pFunct(msgBuff->object, &CanRxMsg);
	} 
}

/******************************************************************************/
// Interrupt from Transeiver
void CO_CANinterrupt_Tx(CO_CANmodule_t *CANmodule)
{

  int8_t txBuff;
	/* Clear interrupt flag */
	__HAL_CAN_DISABLE_IT(CANmodule->hcan, CAN_IT_TME);
	/* First CAN message (bootup) was sent successfully */
	CANmodule->firstCANtxMessage = 0;
	/* clear flag from previous message */
	CANmodule->bufferInhibitFlag = 0;
	/* Are there any new messages waiting to be send */
	if(CANmodule->CANtxCount > 0)
	{
			uint16_t i;             /* index of transmitting message */

			/* first buffer */
			CO_CANtx_t *buffer = CANmodule->txArray;
			/* search through whole array of pointers to transmit message buffers. */
			for(i = CANmodule->txSize; i > 0; i--)
			{
					/* if message buffer is full, send it. */
					if(buffer->bufferFull)
					{
							buffer->bufferFull = 0;
							CANmodule->CANtxCount--;
	txBuff = getFreeTxBuff(CANmodule);    //VJ
							/* Copy message to CAN buffer */
							CANmodule->bufferInhibitFlag = buffer->syncFlag;
							CO_CANsendToModule(CANmodule, buffer, txBuff);
							break;                      /* exit for loop */
					}
					buffer++;
			}/* end of for loop */

			/* Clear counter if no more messages */
			if(i == 0) CANmodule->CANtxCount = 0;
	}
}

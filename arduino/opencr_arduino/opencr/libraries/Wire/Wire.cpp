/*
   TwoWire.cpp - A Wire-like wrapper for SlowSoftI2CMaster.

   It is derived from the Arduino Wire library and for this reason is
   published under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation; either version 2.1 of
   the License, or (at your option) any later version.
*/

#include <Wire.h>

#ifdef USE_SLOW_SOFT_I2C_MASTER

//=============================================================================
// Software I2C - master only
//=============================================================================
TwoWire::TwoWire(uint8_t sda, uint8_t scl) {
  si2c = new SlowSoftI2CMaster(sda, scl);
}

void TwoWire::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  error = 0;
  transmitting = false;

  si2c->i2c_init();
}

void TwoWire::begin(uint8_t sda_pin, uint8_t scl_pin) {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  error = 0;
  transmitting = false;

  si2c->i2c_init(sda_pin, scl_pin);
}

void  TwoWire::end(void) {
  free(si2c);
  si2c = NULL;
}

void  TwoWire::setClock(uint32_t frequency) {
  UNUSED(frequency);  // Change at some point?
}

void  TwoWire::beginTransmission(uint8_t address) {
  if (transmitting) {
    error = (si2c->i2c_rep_start((address<<1)|I2C_WRITE) ? 0 : 2);
  } else {
    error = (si2c->i2c_start((address<<1)|I2C_WRITE) ? 0 : 2);
  }
  // indicate that we are transmitting
  transmitting = 1;
}

void  TwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

uint8_t  TwoWire::endTransmission(uint8_t sendStop)
{
  uint8_t transError = error;
  if (sendStop) {
    si2c->i2c_stop();
    transmitting = 0;
  }
  error = 0;
  return transError;
}

//  This provides backwards compatibility with the original
//  definition, and expected behaviour, of endTransmission
//
uint8_t  TwoWire::endTransmission(void)
{
  return endTransmission(true);
}

size_t  TwoWire::write(uint8_t data) {
  if (si2c->i2c_write(data)) {
    return 1;
  } else {
    if (error == 0) error = 3;
    return 0;
  }
}

size_t  TwoWire::write(const uint8_t *data, size_t quantity) {
  size_t trans = 0;
  for(size_t i = 0; i < quantity; ++i){
    trans += write(data[i]);
  }
  return trans;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
        uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  uint8_t localerror = 0;
  if (isize > 0) {
    // send internal address; this mode allows sending a repeated start to access
    // some devices' internal registers. This function is executed by the hardware
    // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)
    beginTransmission(address);
    // the maximum size of internal address is 3 bytes
    if (isize > 3){
      isize = 3;
    }
    // write internal register address - most significant byte first
    while (isize-- > 0)
      write((uint8_t)(iaddress >> (isize*8)));
    endTransmission(false);
  }
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  localerror = !si2c->i2c_rep_start((address<<1) | I2C_READ);
  if (error == 0 && localerror) error = 2;
  // perform blocking read into buffer
  for (uint8_t cnt=0; cnt < quantity; cnt++)
    rxBuffer[cnt] = si2c->i2c_read(cnt == quantity-1);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = quantity;
  if (sendStop) {
    transmitting = 0;
    si2c->i2c_stop();
  }
  return quantity;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}


uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

int TwoWire::available(void) {
  return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }
  return value;
}

int TwoWire::peek(void) {
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }
  return value;
}

void TwoWire::flush(void) {
}

TwoWire Wire(14, 15);


#else
//=============================================================================
// Hardware I2C
//=============================================================================
#ifdef WIRE_USE_DEBUG_IO_PINS
#include <digitalWriteFast.h>
#define debugDigitalWrite(pin, value) digitalWriteFast((pin), (value))
#else
#define debugDigitalWrite(pin, value)
#endif


#define I2C_TIMEOUT_ADDR    ((uint32_t)10000)       /* 10 s  */
#define I2C_TIMEOUT_BUSY    ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_DIR     ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_RXNE    ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_STOPF   ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_TC      ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_TCR     ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_TXIS    ((uint32_t)25)          /* 25 ms */
#define I2C_TIMEOUT_FLAG    ((uint32_t)25)          /* 25 ms */



TwoWire::TwoWire(I2C_HandleTypeDef *hi2c) {
  hi2c_ = hi2c; // remember the i2c object
  
  // Init some of the I2C Handle init structure. 
  hi2c_->Init.OwnAddress1     = 0x00;  // Lets init the whole structure.
  hi2c_->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c_->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c_->Init.OwnAddress2     = 0xFF;
  hi2c_->Init.OwnAddress2Masks  = I2C_OA2_NOMASK;
  hi2c_->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c_->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  user_onRequest = NULL;
  user_onReceive = NULL;

  frequency_ = 100000;
  state_ = 0; // Begin has not been called. 
  sendStop_ = 1;      // assume last call did a stop...
}


void TwoWire::begin(void) {
  hi2c_->Init.OwnAddress1 = 0x00;

  rxBufferIndex = 0;
  rxBufferLength = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
  error = 0;
  slave_mode = 0;
  transmitting = false;
  state_ = 1;
#ifdef WIRE_USE_DEBUG_IO_PINS
  pinMode(WIRE_DEBUG_RECEIVE_FROM, OUTPUT);
  digitalWriteFast(WIRE_DEBUG_RECEIVE_FROM, LOW);
  pinMode(WIRE_DEBUG_END_TRANSFER, OUTPUT);
  digitalWriteFast(WIRE_DEBUG_END_TRANSFER, LOW);
#endif
  setClock(frequency_);   // set the clock.
}

void TwoWire::begin(uint8_t address) {
  // TODO: Implement slave mode. 
  hi2c_->Init.OwnAddress1 = address << 1;  // 7 bit passed in
  rxBufferIndex = 0;
  rxBufferLength = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
  error = 0;
  slave_mode = 1;  // Need to see how to update slave address and direction with init?
  transmitting = false;
  state_ = 1;
  setClock(frequency_);   // set the clock.

  // Setup to get interrupt when we get a slave address match
  __HAL_I2C_ENABLE_IT(hi2c_, I2C_IT_ADDRI);
}


void  TwoWire::end(void) {
  HAL_I2C_DeInit(hi2c_);
  state_ = 0;
}

void  TwoWire::setClock(uint32_t frequency) {
  if (state_) {
    // BUGBUG: Need to figure this out more...
    if     (frequency <  400000 ) hi2c_->Init.Timing = 0x20404768; // Ask for something < 400K use 100K
    else if(frequency < 1000000 ) hi2c_->Init.Timing = 0x6000030D; // Ask for someting under 1000K so use 400K
    else                          hi2c_->Init.Timing = 0x00200922; // 0x50310001; // Ask for 1000K+ try to give 1000K

    HAL_StatusTypeDef hal_status;
    if ((hal_status = HAL_I2C_Init(hi2c_)) != HAL_OK) {  // Maybe should check for success...
      //Serial.printf("HAL_I2C_Init failed: %d\n", (int)hal_status);
    }
    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(hi2c_,I2C_ANALOGFILTER_ENABLE);
  } else {
    frequency_ = frequency; // remember it away. 
  }
}

void  TwoWire::beginTransmission(uint8_t address) {
  device_address = address << 1; // save away the address, preshift it
  transmitting = 1;
  txBufferLength = 0;
}

void  TwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}


uint8_t  TwoWire::endTransmission(uint8_t sendStop)
{
  // Not sure how to control if send stop or not?
  if (transmitting) {
    //status = HAL_I2C_Master_Transmit(hi2c_, device_address<<1, txBuffer, txBufferLength, WIRE_TX_TIMEOUT);
    uint8_t *pData = txBuffer;
    uint16_t Size = txBufferLength;
  
    if(hi2c_->State != HAL_I2C_STATE_READY) return HAL_BUSY;

    // If our last call did a sendStop we should not do something if bus is busy
    if(sendStop_ && (__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_BUSY) == SET)) return HAL_BUSY;

    /* Process Locked */
    __HAL_LOCK(hi2c_);
    
    hi2c_->State = HAL_I2C_STATE_MASTER_BUSY_TX;
    hi2c_->ErrorCode   = HAL_I2C_ERROR_NONE;
    
    // Now lets build the CR2 register. 
    // Mode will depend on if sendStop is set
    transferConfig(device_address,txBufferLength, sendStop? I2C_AUTOEND_MODE : I2C_SOFTEND_MODE, 
        I2C_GENERATE_START_WRITE);

    sendStop_ = sendStop; // remember for the next call...

    while(Size > 0) {
      /* Wait until TXIS flag is set */
      if(waitOnTXISFlagUntilTimeout(WIRE_TX_TIMEOUT) != HAL_OK) {
        if(hi2c_->ErrorCode == HAL_I2C_ERROR_AF) {
          return HAL_ERROR; 
        } else {
          return HAL_TIMEOUT;
        }
      }

      /* Write data to TXDR */
      hi2c_->Instance->TXDR = (*pData++);
      Size--;
    }
    
    // Check to see which flag we should get at the end...
    if (sendStop) {
      if(waitOnFlagUntilTimeout( I2C_FLAG_STOPF, I2C_TIMEOUT_STOPF) != HAL_OK) {
        if(hi2c_->ErrorCode == HAL_I2C_ERROR_AF) {
          return HAL_ERROR; 
        } else {
          return HAL_TIMEOUT;
        }
      }
      __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_STOPF);
  
      /* Clear Configuration Register 2 */
      I2C_RESET_CR2(hi2c_);
    } else {
      // not generating stop so wait until Transfer complete
      debugDigitalWrite(WIRE_DEBUG_END_TRANSFER, HIGH);
      if(waitOnFlagUntilTimeout(I2C_FLAG_TC, WIRE_TX_TIMEOUT) != HAL_OK) {
        debugDigitalWrite(WIRE_DEBUG_END_TRANSFER, LOW);
        debugDigitalWrite(WIRE_DEBUG_END_TRANSFER, HIGH);
        debugDigitalWrite(WIRE_DEBUG_END_TRANSFER, LOW);
        if(hi2c_->ErrorCode == HAL_I2C_ERROR_AF) {
          return HAL_ERROR; 
        } else {
          return HAL_TIMEOUT;
        }
      }
      debugDigitalWrite(WIRE_DEBUG_END_TRANSFER, LOW);
      // Not sure if we need to reset CR2?  or wait for stop condtion? 
      // Assuming not and next transfer will do it.
    }
  
    hi2c_->State = HAL_I2C_STATE_READY;    
  
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c_);

    transmitting = false; 
  }
  return 0; 
}

//  This provides backwards compatibility with the original
//  definition, and expected behaviour, of endTransmission
//
uint8_t  TwoWire::endTransmission(void)
{
  return endTransmission(true);
}

size_t  TwoWire::write(uint8_t data) {
  if (transmitting || slave_mode) {
    if (txBufferLength >= BUFFER_LENGTH+1) {
      setWriteError();
      return 0;
    }
    txBuffer[txBufferLength++] = data;
    return 1;
  }
  return 0;
}

size_t  TwoWire::write(const uint8_t *data, size_t quantity) {
  size_t trans = 0;
  for(size_t i = 0; i < quantity; ++i){
    trans += write(data[i]);
  }
  return trans;
}

#ifdef LATER
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
        uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
}
#endif

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
  // Hardware I2C... 
  rxBufferIndex = 0;
  rxBufferLength = 0;
  uint8_t *data_in_ptr = rxBuffer;
  if (quantity > sizeof(rxBuffer)) {
    quantity = sizeof(rxBuffer);
  }
  uint8_t count_left_to_read = quantity;

  if(hi2c_->State == HAL_I2C_STATE_READY)
  {    
    // If last call did a sendStop then we should not continue if the BUS is busy
    if(sendStop_ && (__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_BUSY) == SET))
    {
      //Serial.println("TwoWire::requestFrom Busy");
      return 0; //HAL_BUSY;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c_);
    
    hi2c_->State = HAL_I2C_STATE_MASTER_BUSY_RX;
    hi2c_->ErrorCode   = HAL_I2C_ERROR_NONE;
    
    transferConfig(address<<1,  quantity, sendStop? I2C_AUTOEND_MODE : I2C_SOFTEND_MODE, 
        I2C_GENERATE_START_READ);
    sendStop_ = sendStop; // remember for the next call...
    
    do
    {
      /* Wait until RXNE flag is set */
      HAL_StatusTypeDef hstatus;
      if((hstatus = waitOnRXNEFlagUntilTimeout(I2C_TIMEOUT_RXNE)) != HAL_OK) {
          //Serial.printf("TwoWire::requestFrom RXNE %d %d\n", hstatus, count_left_to_read);
          rxBufferLength = quantity - count_left_to_read;  // BUGBUG: seeing what was actually read

          return 0; // an error
      }       
      
      /* Write data to RXDR */

      debugDigitalWrite(WIRE_DEBUG_RECEIVE_FROM, HIGH);
      (*data_in_ptr++) =hi2c_->Instance->RXDR;
      count_left_to_read--;
      debugDigitalWrite(WIRE_DEBUG_RECEIVE_FROM, LOW);

    } while(count_left_to_read > 0);
    

    if (sendStop) {
      /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set */
      if(waitOnFlagUntilTimeout( I2C_FLAG_STOPF, I2C_TIMEOUT_STOPF) != HAL_OK) {
        //Serial.println("TwoWire::requestFrom STOPF");
        return 0; // no bytes
      }
      
      /* Clear STOP Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_STOPF);
      
      /* Clear Configuration Register 2 */
      I2C_RESET_CR2(hi2c_);
    } else {
      // not generating stop so wait until Transfer compelte
      if(waitOnFlagUntilTimeout(I2C_FLAG_TC, WIRE_TX_TIMEOUT) != HAL_OK) {
        if(hi2c_->ErrorCode == HAL_I2C_ERROR_AF) {
          return HAL_ERROR; 
        } else {
          return HAL_TIMEOUT;
        }
      }
      // Not sure if we need to reset CR2?  or wait for stop condtion? 
      // Assuming not and next transfer will do it.
    }    
    hi2c_->State = HAL_I2C_STATE_READY;    
    
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c_);
    rxBufferLength = quantity;
    
    return quantity;
  }
  //Serial.println("TwoWire::requestFrom not Ready");
  return 0;  // An error so say we did not receive anything.  
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}


uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

int TwoWire::available(void) {
  return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }
  return value;
}

int TwoWire::peek(void) {
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }
  return value;
}

void TwoWire::flush(void) {
}


void TwoWire::onReceive(void (*function)(int numBytes)) {
  user_onReceive = function;
}

void TwoWire::onRequest(void (*function)(void)) {
  user_onRequest = function;
}

//=============================================================================
// Callback functions from HAL I2C
//=============================================================================

//-------------------------------------
// Process Address match
//-------------------------------------
void HAL_I2C_SlaveAddrCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == Wire.hi2c_) {
    Wire.processAddrCallback();
  } else if (hi2c == Wire1.hi2c_ )
    Wire1.processAddrCallback();  
}

// Process Address select callback
void TwoWire::processAddrCallback(void) {
  // lets check to see they are writing data to us or wanting data from us
  if (__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_DIR) == SET) {
    // We received request from other side, so, lets call of to our 
    // users call back to let them put data into TX buffer.  
    // Not sure if we should clear buffer first?  Will start off doing so
    txBufferIndex = 0;
    txBufferLength = 0;
    if (user_onRequest) {
      (*user_onRequest)();
      __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_TXE); // Clear out anything that was previously in TXDR
      HAL_I2C_Slave_Transmit_IT(hi2c_, txBuffer, txBufferLength);
    }

  } else {
    // They are sending us data setup to do receive...
    HAL_I2C_Slave_Receive_IT(hi2c_, rxBuffer, sizeof(rxBuffer));
  }
}


//-------------------------------------
// Process slave receiving data complete
//-------------------------------------
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // map this handle to which Wire object
  if (hi2c == Wire.hi2c_) {
    Wire.processRXCallback();
  } else if (hi2c == Wire1.hi2c_ )
    Wire1.processRXCallback();  
}

void TwoWire::processRXCallback(void) {
  // Lets see if we can figure out what happened...
  rxBufferLength = (uint32_t)(hi2c_->pBuffPtr-rxBuffer);
  rxBufferIndex = 0;  // setup the index to first one
//  Serial.printf("ProcessRXCallback: %x %x %x %d\n", (uint32_t)hi2c_->pBuffPtr,
//    hi2c_->XferSize, hi2c_->XferCount, rxBufferLength);

  if (user_onReceive) {
    (*user_onReceive)(rxBufferLength);
  }
  // Setup to try to receive next message.
  HAL_I2C_Slave_Receive_IT(hi2c_, rxBuffer, sizeof(rxBuffer));
}

//-------------------------------------
// Process slave transmit data complete
//-------------------------------------
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // map this handle to which Wire object
  if (hi2c == Wire.hi2c_) {
    Wire.processTXCallback();
  } else if (hi2c == Wire1.hi2c_ )
    Wire1.processTXCallback();  
}

void TwoWire::processTXCallback(void) {
  // Lets see if we can figure out what happened...
  txBufferLength = (uint32_t)(hi2c_->pBuffPtr-txBuffer);
  
  HAL_I2C_Slave_Transmit_IT(hi2c_, txBuffer, txBufferLength);
}

//-------------------------------------
// Process HAL errors detected
//-------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  UNUSED(hi2c);
  //Serial.println("HAL_I2C_ErrorCallback called");
}


//=============================================================================
// Helper functions from stm32fxx_hal_i2c.c
//=============================================================================
HAL_StatusTypeDef TwoWire::waitOnTXISFlagUntilTimeout(uint32_t Timeout)  
{  
  uint32_t tickstart = HAL_GetTick();
  
  while(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_TXIS) == RESET)
  {
    /* Check if a NACK is detected */
    if(isAcknowledgeFailed(Timeout) != HAL_OK)
    {
      return HAL_ERROR;
    }
    
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        hi2c_->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
        hi2c_->State= HAL_I2C_STATE_READY;
        
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c_);
        
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;      
}


HAL_StatusTypeDef TwoWire::waitOnRXNEFlagUntilTimeout(uint32_t Timeout)
{  
  uint32_t tickstart = 0x00;
  tickstart = HAL_GetTick();
  
  while(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_RXNE) == RESET)
  {
    /* Check if a NACK is detected */
    if(isAcknowledgeFailed(Timeout) != HAL_OK)
    {
      //Serial.println("NACK");
      return HAL_ERROR;
    }
    
    // See if it was set during the wait for ACK...
    if(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_RXNE) != RESET) 
    {
      break;  // Data is available so break out of loop
    }

    /* Check if a STOPF is detected */
    if(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_STOPF) == SET)
    {
      /* Clear STOP Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_STOPF);
      
      /* Clear Configuration Register 2 */
      I2C_RESET_CR2(hi2c_);
      
      hi2c_->ErrorCode = HAL_I2C_ERROR_NONE;
      hi2c_->State= HAL_I2C_STATE_READY;
      
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c_);
      
      //Serial.println("STOPF");
      return HAL_ERROR;
    }
    
    /* Check for the Timeout */
    if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
    {
      hi2c_->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
      hi2c_->State= HAL_I2C_STATE_READY;
      
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c_);
      
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}



HAL_StatusTypeDef TwoWire::waitOnFlagUntilTimeout(uint32_t flag, uint32_t Timeout)
{  
  uint32_t tickstart = 0x00;
  tickstart = HAL_GetTick();
  
  while(__HAL_I2C_GET_FLAG(hi2c_, flag) == RESET)
  {
    /* Check if a NACK is detected */
    if(isAcknowledgeFailed(Timeout) != HAL_OK)
    {
      return HAL_ERROR;
    }
    
    /* Check for the Timeout */
    if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
    {
      hi2c_->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
      hi2c_->State= HAL_I2C_STATE_READY;
      
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c_);
      
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}


/**
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  hi2c_ : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef TwoWire::isAcknowledgeFailed(uint32_t Timeout)
{
  uint32_t tickstart = 0x00;
  tickstart = HAL_GetTick();
  
  if(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_AF) == SET)
  {
    /* Wait until STOP Flag is reset */
    /* AutoEnd should be initiate after AF */
    while(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_STOPF) == RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          hi2c_->State= HAL_I2C_STATE_READY;
          /* Process Unlocked */
          __HAL_UNLOCK(hi2c_);
          return HAL_TIMEOUT;
        }
      }
    }
    
    /* Clear NACKF Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_AF);
    
    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_STOPF);
    
    /* Flush TX register if not empty */
    if(__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_TXE) == RESET)
    {
      __HAL_I2C_CLEAR_FLAG(hi2c_, I2C_FLAG_TXE);
    }
    
    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c_);
    
    hi2c_->ErrorCode = HAL_I2C_ERROR_AF;
    hi2c_->State= HAL_I2C_STATE_READY;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c_);
    
    return HAL_ERROR;
  }
  return HAL_OK;
}

void TwoWire::transferConfig(uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
  uint32_t tmpreg = 0;
  
   /* Get the CR2 register value */
  tmpreg = hi2c_->Instance->CR2;
  
  /* clear tmpreg specific bits */
  tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));
  
  /* update tmpreg */
  tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16 ) & I2C_CR2_NBYTES) | \
    (uint32_t)Mode | (uint32_t)Request);
  
  /* update CR2 register */
  hi2c_->Instance->CR2 = tmpreg;  
}  

//=============================================================================
// Set up IRQ handlers for the Wire objects...
// Note: Should probably be in the HAL driver file, but this way only happens
// if app includes wire. 
//=============================================================================
#if 0
void I2C1_EV_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler(&Wire.i2c_handle_);
}

void I2C2_EV_IRQHandler(void)  {
  HAL_I2C_EV_IRQHandler(&Wire1.i2c_handle_);
}

void I2C1_ER_IRQHandler(void) {
  HAL_I2C_ER_IRQHandler(&Wire.i2c_handle_);
}

void I2C2_ER_IRQHandler(void)  {
  HAL_I2C_ER_IRQHandler(&Wire1.i2c_handle_);
}
#endif

//=============================================================================
// Define our hardware I2C objects
//=============================================================================

TwoWire Wire(&drv_i2c_handles[0]);
TwoWire Wire1(&drv_i2c_handles[1]);

#endif
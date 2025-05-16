#include "STM32H7_SPIComms.h"

volatile DMA_RxBuffer_t rxDMABuffer;


STM32H7_SPIComms::STM32H7_SPIComms(volatile rxData_t* _ptrRxData, volatile txData_t* _ptrTxData, SPI_TypeDef* _spiType) :
	ptrRxData(_ptrRxData),
	ptrTxData(_ptrTxData),
	spiType(_spiType)
{
    spiHandle.Instance = spiType;
    ptrRxDMABuffer = &rxDMABuffer;

    irqNss = EXTI4_IRQn;
    irqDMAtx = DMA1_Stream0_IRQn;
    irqDMArx = DMA1_Stream1_IRQn;
}

STM32H7_SPIComms::~STM32H7_SPIComms() {

}

void STM32H7_SPIComms::init() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(spiHandle.Instance == SPI2)
    {
    	// Interrupt pin is the NSS pin
        // Configure GPIO pin : PA_4

        #ifdef SPI2
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        #else
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        #endif

        printf("Initialising SPI2 slave\n");

        spiHandle.Init.Mode           			= SPI_MODE_SLAVE;
        spiHandle.Init.Direction      			= SPI_DIRECTION_2LINES;
        spiHandle.Init.DataSize       			= SPI_DATASIZE_8BIT;
        spiHandle.Init.CLKPolarity    			= SPI_POLARITY_LOW;
        spiHandle.Init.CLKPhase       			= SPI_PHASE_1EDGE;
        spiHandle.Init.NSS            			= SPI_NSS_HARD_INPUT;
        spiHandle.Init.FirstBit       			= SPI_FIRSTBIT_MSB;
        spiHandle.Init.TIMode         			= SPI_TIMODE_DISABLE;
        spiHandle.Init.CRCCalculation 			= SPI_CRCCALCULATION_DISABLE;
        spiHandle.Init.CRCPolynomial  			= 0x0;
        spiHandle.Init.NSSPMode 				= SPI_NSS_PULSE_DISABLE;
        spiHandle.Init.NSSPolarity 				= SPI_NSS_POLARITY_LOW;
        spiHandle.Init.FifoThreshold 			= SPI_FIFO_THRESHOLD_01DATA;
        spiHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        spiHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        spiHandle.Init.MasterSSIdleness 		= SPI_MASTER_SS_IDLENESS_00CYCLE;
        spiHandle.Init.MasterInterDataIdleness 	= SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
        spiHandle.Init.MasterReceiverAutoSusp 	= SPI_MASTER_RX_AUTOSUSP_DISABLE;
        spiHandle.Init.MasterKeepIOState 		= SPI_MASTER_KEEP_IO_STATE_DISABLE;
        spiHandle.Init.IOSwap 					= SPI_IO_SWAP_DISABLE;

        HAL_SPI_Init(&this->spiHandle);

    	// Peripheral clock enable
    	__HAL_RCC_SPI2_CLK_ENABLE();

		printf("Initialising GPIO for SPI\n");

	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    /**SPI1 GPIO Configuration
	    PA4     ------> SPI1_NSS
	    PA5     ------> SPI1_SCK
	    PA6     ------> SPI1_MISO
	    PA7     ------> SPI1_MOSI
	    */
    	GPIO_InitStruct = {0};
	    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        printf("Initialising DMA for SPI\n");

        hdma_spi_tx.Instance 					= DMA1_Stream0;
        hdma_spi_tx.Init.Request 				= DMA_REQUEST_SPI2_TX;
        hdma_spi_tx.Init.Direction 				= DMA_MEMORY_TO_PERIPH;
        hdma_spi_tx.Init.PeriphInc 				= DMA_PINC_DISABLE;
        hdma_spi_tx.Init.MemInc 				= DMA_MINC_ENABLE;
        hdma_spi_tx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
        hdma_spi_tx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
        hdma_spi_tx.Init.Mode 					= DMA_CIRCULAR;
        hdma_spi_tx.Init.Priority 				= DMA_PRIORITY_LOW;
        hdma_spi_tx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;

        HAL_DMA_Init(&hdma_spi_tx);
        __HAL_LINKDMA(&spiHandle, hdmatx, hdma_spi_tx);

        hdma_spi_rx.Instance 					= DMA1_Stream1;
        hdma_spi_rx.Init.Request 				= DMA_REQUEST_SPI2_RX;
        hdma_spi_rx.Init.Direction 				= DMA_PERIPH_TO_MEMORY;
        hdma_spi_rx.Init.PeriphInc 				= DMA_PINC_DISABLE;
        hdma_spi_rx.Init.MemInc 				= DMA_MINC_ENABLE;
        hdma_spi_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
        hdma_spi_rx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
        hdma_spi_rx.Init.Mode 					= DMA_CIRCULAR;
        hdma_spi_rx.Init.Priority 				= DMA_PRIORITY_LOW;
        hdma_spi_rx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;

        HAL_DMA_Init(&hdma_spi_rx);
        __HAL_LINKDMA(&spiHandle, hdmarx, hdma_spi_rx);

        printf("Initialising DMA for Memory to Memory transfer\n");

        hdma_memtomem.Instance 					= DMA1_Stream2;
        hdma_memtomem.Init.Request 				= DMA_REQUEST_MEM2MEM;
        hdma_memtomem.Init.Direction 			= DMA_MEMORY_TO_MEMORY;
        hdma_memtomem.Init.PeriphInc 			= DMA_PINC_ENABLE;
        hdma_memtomem.Init.MemInc 				= DMA_MINC_ENABLE;
        hdma_memtomem.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
        hdma_memtomem.Init.MemDataAlignment 	= DMA_MDATAALIGN_BYTE;
        hdma_memtomem.Init.Mode 				= DMA_NORMAL;
        hdma_memtomem.Init.Priority 			= DMA_PRIORITY_LOW;
        hdma_memtomem.Init.FIFOMode 			= DMA_FIFOMODE_ENABLE;
        hdma_memtomem.Init.FIFOThreshold 		= DMA_FIFO_THRESHOLD_FULL;
        hdma_memtomem.Init.MemBurst 			= DMA_MBURST_SINGLE;
        hdma_memtomem.Init.PeriphBurst 			= DMA_PBURST_SINGLE;

        HAL_DMA_Init(&hdma_memtomem);
    }
}

void STM32H7_SPIComms::start() {
    // Register the NSS (slave select) interrupt
    NssInterrupt = new ModuleInterrupt<STM32H7_SPIComms>(
        irqNss,
        this,
        &STM32H7_SPIComms::handleNssInterrupt
    );
    HAL_NVIC_SetPriority(irqNss, Config::spiNssIrqPriority, 0);
    HAL_NVIC_EnableIRQ(irqNss);

    // Register the DMA Rx interrupt
    dmaRxInterrupt = new ModuleInterrupt<STM32H7_SPIComms>(
        irqDMArx,
        this,
        &STM32H7_SPIComms::handleRxInterrupt
    );
    HAL_NVIC_SetPriority(irqDMArx, Config::spiDmaRxIrqPriority, 0);
    HAL_NVIC_EnableIRQ(irqDMArx);

    // Register the DMA Tx interrupt
    dmaTxInterrupt = new ModuleInterrupt<STM32H7_SPIComms>(
        irqDMAtx,
        this,
        &STM32H7_SPIComms::handleTxInterrupt
    );
    HAL_NVIC_SetPriority(irqDMAtx, Config::spiDmaTxIrqPriority, 0); // TX needs higher priority than RX
    HAL_NVIC_EnableIRQ(irqDMAtx);

    // Initialize the data buffers
    std::fill(std::begin(ptrTxData->txBuffer), std::end(ptrTxData->txBuffer), 0);
    std::fill(std::begin(ptrRxData->rxBuffer), std::end(ptrRxData->rxBuffer), 0);
    std::fill(std::begin(ptrRxDMABuffer->buffer[0].rxBuffer), std::end(ptrRxDMABuffer->buffer[0].rxBuffer), 0);
    std::fill(std::begin(ptrRxDMABuffer->buffer[1].rxBuffer), std::end(ptrRxDMABuffer->buffer[1].rxBuffer), 0);

    ptrTxData->header = Config::pruData;

    // Start the multi-buffer DMA SPI communication
    dmaStatus = startMultiBufferDMASPI(
        (uint8_t*)ptrTxData->txBuffer,
        (uint8_t*)ptrTxData->txBuffer,
        (uint8_t*)ptrRxDMABuffer->buffer[0].rxBuffer,
        (uint8_t*)ptrRxDMABuffer->buffer[1].rxBuffer,
        Config::dataBuffSize
    );

    // Check for DMA initialization errors
    if (dmaStatus != HAL_OK) {
        printf("DMA SPI error\n");
    }
}

HAL_StatusTypeDef STM32H7_SPIComms::startMultiBufferDMASPI(uint8_t *pTxBuffer0, uint8_t *pTxBuffer1,
                                                   uint8_t *pRxBuffer0, uint8_t *pRxBuffer1,
                                                   uint16_t Size)
{
    /* Check Direction parameter */
    assert_param(IS_SPI_DIRECTION_2LINES(spiHandle.Init.Direction));

    if (spiHandle.State != HAL_SPI_STATE_READY)
    {
        return HAL_BUSY;
    }

    if ((pTxBuffer0 == NULL) || (pRxBuffer0 == NULL) || (Size == 0UL))
    {
        return HAL_ERROR;
    }

    /* If secondary Tx or Rx buffer is not provided, use the primary buffer */
    if (pTxBuffer1 == NULL)
    {
        pTxBuffer1 = pTxBuffer0;
    }

    if (pRxBuffer1 == NULL)
    {
        pRxBuffer1 = pRxBuffer0;
    }

    /* Lock the process */
    __HAL_LOCK(&spiHandle);

    /* Set the transaction information */
    spiHandle.State       = HAL_SPI_STATE_BUSY_TX_RX;
    spiHandle.ErrorCode   = HAL_SPI_ERROR_NONE;
    spiHandle.TxXferSize  = Size;
    spiHandle.TxXferCount = Size;
    spiHandle.RxXferSize  = Size;
    spiHandle.RxXferCount = Size;

    /* Init unused fields in handle to zero */
    spiHandle.RxISR       = NULL;
    spiHandle.TxISR       = NULL;

    /* Set Full-Duplex mode */
    SPI_2LINES(&spiHandle);

    /* Reset the Tx/Rx DMA bits */
    CLEAR_BIT(spiHandle.Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    /* Adjust XferCount according to DMA alignment / Data size */
    if (spiHandle.Init.DataSize <= SPI_DATASIZE_8BIT)
    {
        if (hdma_spi_tx.Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
        {
            spiHandle.TxXferCount = (spiHandle.TxXferCount + 1UL) >> 1UL;
        }
        if (hdma_spi_rx.Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
        {
            spiHandle.RxXferCount = (spiHandle.RxXferCount + 1UL) >> 1UL;
        }
    }
    else if (spiHandle.Init.DataSize <= SPI_DATASIZE_16BIT)
    {
        if (hdma_spi_tx.Init.MemDataAlignment == DMA_MDATAALIGN_WORD)
        {
            spiHandle.TxXferCount = (spiHandle.TxXferCount + 1UL) >> 1UL;
        }
        if (hdma_spi_rx.Init.MemDataAlignment == DMA_MDATAALIGN_WORD)
        {
            spiHandle.RxXferCount = (spiHandle.RxXferCount + 1UL) >> 1UL;
        }
    }

    /* Configure Tx DMA with Multi-Buffer */
    hdma_spi_tx.XferHalfCpltCallback = NULL;
    hdma_spi_tx.XferCpltCallback     = NULL;
    hdma_spi_tx.XferErrorCallback    = NULL;

    if (HAL_OK != HAL_DMAEx_MultiBufferStart_IT(&hdma_spi_tx,
                                                (uint32_t)pTxBuffer0,
                                                (uint32_t)&spiHandle.Instance->TXDR,
                                                (uint32_t)pTxBuffer1,
                                                spiHandle.TxXferCount))
    {
        __HAL_UNLOCK(&spiHandle);
        return HAL_ERROR;
    }

    /* Configure Rx DMA with Multi-Buffer */
    hdma_spi_rx.XferHalfCpltCallback = NULL;
    hdma_spi_rx.XferCpltCallback     = NULL;
    hdma_spi_rx.XferErrorCallback    = NULL;

    if (HAL_OK != HAL_DMAEx_MultiBufferStart_IT(&hdma_spi_rx,
                                                (uint32_t)&spiHandle.Instance->RXDR,
                                                (uint32_t)pRxBuffer0,
                                                (uint32_t)pRxBuffer1,
                                                spiHandle.RxXferCount))
    {
        (void)HAL_DMA_Abort(&hdma_spi_tx);
        __HAL_UNLOCK(&spiHandle);
        return HAL_ERROR;
    }

    /* Configure SPI TSIZE for full transfer or circular mode */
    if (hdma_spi_rx.Init.Mode == DMA_CIRCULAR || hdma_spi_tx.Init.Mode == DMA_CIRCULAR)
    {
        MODIFY_REG(spiHandle.Instance->CR2, SPI_CR2_TSIZE, 0UL);
    }
    else
    {
        MODIFY_REG(spiHandle.Instance->CR2, SPI_CR2_TSIZE, Size);
    }

    /* Enable Tx and Rx DMA Requests */
    SET_BIT(spiHandle.Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    /* Enable SPI error interrupt */
    __HAL_SPI_ENABLE_IT(&spiHandle, (SPI_IT_OVR | SPI_IT_UDR | SPI_IT_FRE | SPI_IT_MODF));

    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(&spiHandle);

    if (spiHandle.Init.Mode == SPI_MODE_MASTER)
    {
        SET_BIT(spiHandle.Instance->CR1, SPI_CR1_CSTART);
    }

    __HAL_UNLOCK(&spiHandle);

    return HAL_OK;
}

int STM32H7_SPIComms::DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
  uint32_t tmpisr_dma;
  int interrupt;

  /* calculate DMA base and stream number */
  DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  tmpisr_dma  = regs_dma->ISR;

  if(IS_DMA_STREAM_INSTANCE(hdma->Instance) != 0U)  /* DMA1 or DMA2 instance */
  {
    /* Transfer Error Interrupt management ***************************************/
    if ((tmpisr_dma & (DMA_FLAG_TEIF0_4 << (hdma->StreamIndex & 0x1FU))) != 0U)
    {
      if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TE) != 0U)
      {
        /* Disable the transfer error interrupt */
        ((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_TE);

        /* Clear the transfer error flag */
        regs_dma->IFCR = DMA_FLAG_TEIF0_4 << (hdma->StreamIndex & 0x1FU);

        /* Update error code */
        hdma->ErrorCode |= HAL_DMA_ERROR_TE;
        interrupt =  DMA_OTHER;
      }
    }
    /* FIFO Error Interrupt management ******************************************/
    if ((tmpisr_dma & (DMA_FLAG_FEIF0_4 << (hdma->StreamIndex & 0x1FU))) != 0U)
    {
      if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_FE) != 0U)
      {
        /* Clear the FIFO error flag */
        regs_dma->IFCR = DMA_FLAG_FEIF0_4 << (hdma->StreamIndex & 0x1FU);

        /* Update error code */
        hdma->ErrorCode |= HAL_DMA_ERROR_FE;
        interrupt =  DMA_OTHER;
      }
    }
    /* Direct Mode Error Interrupt management ***********************************/
    if ((tmpisr_dma & (DMA_FLAG_DMEIF0_4 << (hdma->StreamIndex & 0x1FU))) != 0U)
    {
      if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_DME) != 0U)
      {
        /* Clear the direct mode error flag */
        regs_dma->IFCR = DMA_FLAG_DMEIF0_4 << (hdma->StreamIndex & 0x1FU);

        /* Update error code */
        hdma->ErrorCode |= HAL_DMA_ERROR_DME;
        interrupt =  DMA_OTHER;
      }
    }
    /* Half Transfer Complete Interrupt management ******************************/
    if ((tmpisr_dma & (DMA_FLAG_HTIF0_4 << (hdma->StreamIndex & 0x1FU))) != 0U)
    {
      if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) != 0U)
      {
        /* Clear the half transfer complete flag */
        regs_dma->IFCR = DMA_FLAG_HTIF0_4 << (hdma->StreamIndex & 0x1FU);

        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        if((((DMA_Stream_TypeDef   *)hdma->Instance)->CR & DMA_SxCR_CIRC) == 0U)
        {
          /* Disable the half transfer interrupt */
          ((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_HT);
        }

      }
      interrupt = DMA_HALF_TRANSFER;
    }
    /* Transfer Complete Interrupt management ***********************************/
    if ((tmpisr_dma & (DMA_FLAG_TCIF0_4 << (hdma->StreamIndex & 0x1FU))) != 0U)
    {
      if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != 0U)
      {
        /* Clear the transfer complete flag */
        regs_dma->IFCR = DMA_FLAG_TCIF0_4 << (hdma->StreamIndex & 0x1FU);

        if(HAL_DMA_STATE_ABORT == hdma->State)
        {
          /* Disable all the transfer interrupts */
          ((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
          ((DMA_Stream_TypeDef   *)hdma->Instance)->FCR &= ~(DMA_IT_FE);

          if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
          {
            ((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_HT);
          }

          /* Clear all interrupt flags at correct offset within the register */
          regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);

          /* Change the DMA state */
          hdma->State = HAL_DMA_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hdma);

          interrupt = DMA_TRANSFER_COMPLETE;
        }

        if((((DMA_Stream_TypeDef   *)hdma->Instance)->CR & DMA_SxCR_CIRC) == 0U)
        {
          /* Disable the transfer complete interrupt */
          ((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_TC);

          /* Change the DMA state */
          hdma->State = HAL_DMA_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hdma);
        }
        interrupt =  2;
      }
    }
  }

  return interrupt;
}

int STM32H7_SPIComms::getActiveDMAmemory(DMA_HandleTypeDef *hdma)
{
    DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)hdma->Instance;

    return (dmaStream->CR & DMA_SxCR_CT) ? 1 : 0;
}

void STM32H7_SPIComms::handleNssInterrupt()
{
	// SPI packet has been fully received
	// Flag the copy the RX buffer if new WRITE data has been received
	// DMA copy is performed during the servo thread update
	if (newWriteData)
	{
		copyRXbuffer = true;
		newWriteData = false;
	}
}

void STM32H7_SPIComms::handleTxInterrupt()
{
	DMA_IRQHandler(&hdma_spi_tx);
	HAL_NVIC_EnableIRQ(irqDMAtx);
}

void STM32H7_SPIComms::handleRxInterrupt()
{
    // Handle the interrupt and determine the type of interrupt
    interruptType = DMA_IRQHandler(&hdma_spi_rx);

    RxDMAmemoryIdx = getActiveDMAmemory(&hdma_spi_rx);

    if (interruptType == DMA_HALF_TRANSFER) // Use the HTC interrupt to check the packet being received
    {
        switch (ptrRxDMABuffer->buffer[RxDMAmemoryIdx].header)
        {
            case Config::pruRead:
                // No action needed for PRU_READ.
            	dataCallback(true);
                break;

            case Config::pruWrite:
            	// Valid PRU_WRITE header, flag RX data transfer.
            	dataCallback(true);
            	newWriteData = true;
                RXbufferIdx = RxDMAmemoryIdx;
                break;

            default:
            	dataCallback(false);
                break;
        }
    }
    else if (interruptType == DMA_TRANSFER_COMPLETE) // Transfer complete interrupt
    {
        // Placeholder for transfer complete handling if needed in the future.
    }
    else // Other interrupt sources
    {
        printf("DMA SPI Rx error\n");
    }

    HAL_NVIC_EnableIRQ(irqDMArx);
}

void STM32H7_SPIComms::tasks() {

	if (copyRXbuffer == true)
    {
	    uint8_t* srcBuffer = (uint8_t*)ptrRxDMABuffer->buffer[RXbufferIdx].rxBuffer;
	    uint8_t* destBuffer = (uint8_t*)ptrRxData->rxBuffer;

	    __disable_irq();

	    dmaStatus = HAL_DMA_Start(
	    							&hdma_memtomem,
									(uint32_t)srcBuffer,
									(uint32_t)destBuffer,
									Config::dataBuffSize
	    							);

	    if (dmaStatus == HAL_OK) {
	        dmaStatus = HAL_DMA_PollForTransfer(&hdma_memtomem, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
	    }

	    __enable_irq();
	    HAL_DMA_Abort(&hdma_memtomem);
		copyRXbuffer = false;
    }
}


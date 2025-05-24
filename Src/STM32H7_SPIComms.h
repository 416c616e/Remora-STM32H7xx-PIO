#ifndef STM32H7_SPIComms_H
#define STM32H7_SPIComms_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma.h"

#include <memory>
#include <algorithm>

#include "remora/remora.h"
#include "remora/comms/commsInterface.h"
#include "remora/modules/moduleInterrupt.h"

class STM32H7_SPIComms : public CommsInterface {
private:
    volatile rxData_t*  		ptrRxData;
    volatile txData_t*  		ptrTxData;
    volatile DMA_RxBuffer_t* 	ptrRxDMABuffer;
    SPI_TypeDef*        		spiType;

    uint8_t						RxDMAmemoryIdx;
    uint8_t						RXbufferIdx;
    bool						copyRXbuffer;

	ModuleInterrupt<STM32H7_SPIComms>*	NssInterrupt;
    ModuleInterrupt<STM32H7_SPIComms>*	dmaTxInterrupt;
	ModuleInterrupt<STM32H7_SPIComms>*	dmaRxInterrupt;
	IRQn_Type					irqNss;
	IRQn_Type					irqDMArx;
	IRQn_Type					irqDMAtx;

    SPI_HandleTypeDef   		spiHandle;
    DMA_HandleTypeDef   		hdma_spi_tx;
    DMA_HandleTypeDef   		hdma_spi_rx;
    DMA_HandleTypeDef   		hdma_memtomem;
    HAL_StatusTypeDef   		dmaStatus;

    uint8_t						interruptType;

    bool						newWriteData;

	HAL_StatusTypeDef startMultiBufferDMASPI(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint16_t);
	int getActiveDMAmemory(DMA_HandleTypeDef*);

	int DMA_IRQHandler(DMA_HandleTypeDef *);
	void handleRxInterrupt(void);
	void handleTxInterrupt(void);
	void handleNssInterrupt(void);

public:
	STM32H7_SPIComms(volatile rxData_t*, volatile txData_t*, SPI_TypeDef*);
	virtual ~STM32H7_SPIComms();

    void init(void);
    void start(void);
    void tasks(void);
};

#endif

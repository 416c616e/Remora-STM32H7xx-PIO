#include "SoftwareSPI.h"

void SoftwareSPI::delay() {
    for (volatile uint32_t i = 0; i < delayTicks; i++) {
        __asm volatile("nop");
    }
}


// Constructor WITHOUT chip select (CS handled externally)
SoftwareSPI::SoftwareSPI(const std::string& mosiPin, const std::string& misoPin, const std::string& clkPin,
		uint8_t mode = 0, BitOrder bitOrder, ByteOrder byteOrder)
    : mosi(mosiPin, OUTPUT),
      miso(misoPin, INPUT),
      clk(clkPin, OUTPUT),
      cs(nullptr),
      delayTicks(200),
      bitOrder(bitOrder),
      byteOrder(byteOrder) {
	setSPIMode(mode);
}

// Constructor WITH chip select
SoftwareSPI::SoftwareSPI(const std::string& mosiPin, const std::string& misoPin, const std::string& clkPin, const std::string& csPin,
		uint8_t mode = 0, BitOrder bitOrder, ByteOrder byteOrder)
    : mosi(mosiPin, OUTPUT),
      miso(misoPin, INPUT, PULLUP),
      clk(clkPin, OUTPUT),
      cs(new Pin(csPin, OUTPUT)), // Dynamically allocate CS pin
      delayTicks(200),
      bitOrder(bitOrder),
      byteOrder(byteOrder)
{
	setSPIMode(mode);
	cs->set(true); // Set CS high (inactive state)
}

SoftwareSPI::~SoftwareSPI() {
    if (cs) {
        delete cs; // Clean up CS pin if used
    }
}

void SoftwareSPI::begin() {
    mosi.setAsOutput();
    miso.setAsInput();
    clk.setAsOutput();
    clk.set(cpol); // Set clock idle state
    if (cs) cs->set(true); // Ensure CS starts high
}

void SoftwareSPI::end() {
    mosi.setAsInput();
    miso.setAsInput();
    clk.setAsInput();
    if (cs) cs->set(true); // Set CS high when stopping
}

void SoftwareSPI::setClockDivider(uint32_t div) {
    delayTicks = div;
}

void SoftwareSPI::setSPIMode(uint8_t mode) {
    cpol = (mode & 0x02) != 0; // CPOL is bit 1
    cpha = (mode & 0x01) != 0; // CPHA is bit 0
}

void SoftwareSPI::transfer(uint8_t* data, size_t length) {
    if (cs) cs->set(false); // Pull CS low to start transaction

    for (size_t byteIdx = 0; byteIdx < length; byteIdx++) {
        size_t index = (byteOrder == MSB_FIRST_BYTE) ? byteIdx : (length - 1 - byteIdx);
        uint8_t received = 0;
        uint8_t byteToSend = data[index];

        for (uint8_t i = 0; i < 8; i++) {
            uint8_t bitPos = (bitOrder == MSB_FIRST) ? (7 - i) : i;

            clk.set(!cpol); // Start clock low if CPOL=0, high if CPOL=1
            delay();
            mosi.set((byteToSend & (1 << bitPos)) ? true : false); // Set data bit

            clk.set(cpol); // Generate clock edge
            delay();

            if (cpha) { // Read on second clock edge if CPHA=1
                received |= (miso.get() ? 1 : 0) << bitPos;
            }
            clk.set(!cpol);
            delay();

            if (!cpha) { // Read on first clock edge if CPHA=0
                received |= (miso.get() ? 1 : 0) << bitPos;
            }
        }
        clk.set(cpol);

        data[index] = received;
    }

    if (cs) cs->set(true); // Pull CS high to end transaction
}

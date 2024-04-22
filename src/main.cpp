#include <Arduino.h>

#include "MCP3x6x.h"

const uint8_t MCP_SPI_MISO = 12u;
const uint8_t MCP_SPI_MOSI = 11u;
const uint8_t MCP_SPI_SCK  = 13u;
const uint8_t MCP_SPI_SS   = 10u;

SPIClass mySPI(&sercom1, MCP_SPI_MISO, MCP_SPI_SCK, MCP_SPI_MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
const SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);

MCP3561 mcp(MCP_SPI_SS, &mySPI, spiSettings, MCP_SPI_MOSI, MCP_SPI_MISO, MCP_SPI_SCK);

bool irq_flag = false;

void handler() { irq_flag = true; }

int channels[]              = {MCP3x6x_DIFFA, MCP3x6x_DIFFB, MCP3x6x_OFFSET};
const size_t channels_count = sizeof(channels) / sizeof(int);

void setup() {
  Serial.begin(115200);

  using namespace MCP3x6x;
  mcp.begin(CONTINUOUS, CONVERSION, ID_SGNEXT_DATA);
  mcp.attachIRQ(8, handler);
  mcp.setClockSelection(INTERN);
  mcp.calibrate();
  mcp.setAveraging(OSR_512);

  mcp.enableGain(GAIN_2);
  mcp.enableOffset(0x00);

  for (size_t i = 0; i < channels_count; i++) {
    mcp.enableScanChannel(channels[i]);
  }
}

double res() { return mcp.getReference() / mcp.getMaxValue(); }

void loop() {
  if (irq_flag && Serial) {
    mcp.IRQ_handler();
    irq_flag = false;

    Serial.print("mcp");
    Serial.print(String(mcp.getLatestChannel()));
    Serial.print(":\t");
    Serial.print(mcp.result.raw[mcp.getLatestChannel()] * res() / 1000 / 1000, 2);

    Serial.println();
  }
}

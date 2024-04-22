// SPDX-License-Identifier: MIT

/**
 * @file test_MCP3462.cpp
 * @author Stefan Herold (stefan.herold@posteo.de)
 * @brief
 * @version 0.0.2
 * @date 2023-10-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifdef PIO_NATIVE_TESTING
#  include <ArduinoFake.h>
#else
#  include <Arduino.h>
#endif

#include <unity.h>

#include "MCP3x6x.hpp"
#include "test_Settings.h"

void setup(void) {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);

  runUnityTests();
}

void loop(void) {}

int runUnityTests(void) {
  UNITY_BEGIN();

  // Settings
  RUN_TEST(test_Settings);
  RUN_TEST(test_Sizes);

  return UNITY_END();
}

void suiteSetUp(void) {
#ifdef PIO_NATIVE_TESTING
  using namespace fakeit;

  ArduinoFakeReset();
  When(Method(ArduinoFake(), sei)).AlwaysReturn();
  When(Method(ArduinoFake(), cli)).AlwaysReturn();
  When(Method(ArduinoFake(), digitalWrite)).AlwaysReturn();
  When(Method(ArduinoFake(), pinMode)).AlwaysReturn();
  When(Method(ArduinoFake(), attachInterrupt)).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), begin, void(void))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), end, void(void))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), beginTransaction, void(SPISettings))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), endTransaction, void(void))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), transfer, byte(uint8_t))).AlwaysReturn();
  When(OverloadedMethod(ArduinoFake(SPI), transfer, void(void *, size_t))).AlwaysReturn();
#endif
}

void suiteTearDown(void) {}

void setUp(void) {
  // TODO reset settings
}

void tearDown(void) {}

// actual test cases

void test_Settings(void) {
  MCP3464 mcp;

  /*
    // checks if Settings class is initialized with correct default values.
    TEST_ASSERT_EQUAL_CHAR(0xC0, mcp._config0.raw);
    TEST_ASSERT_EQUAL_CHAR(0x0C, mcp._config1.raw);
    TEST_ASSERT_EQUAL_CHAR(0x8B, mcp._config2.raw);
    TEST_ASSERT_EQUAL_CHAR(0x00, mcp._config3.raw);
    TEST_ASSERT_EQUAL_CHAR(0x73, mcp._irq.raw);
    TEST_ASSERT_EQUAL_CHAR(0x01, mcp._mux.raw);
    TEST_ASSERT_EQUAL_CHAR(0xA5, mcp._lock.raw);
  */
}

void test_Sizes(void) {
  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::Config0));
  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::Config1));
  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::Config2));
  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::Config3));
  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::Irq));
  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::Mux));
  TEST_ASSERT_EQUAL_size_t(3, sizeof(MCP3x6x::Scan));
  TEST_ASSERT_EQUAL_size_t(3, sizeof(MCP3x6x::Timer));
  TEST_ASSERT_EQUAL_size_t(3, sizeof(MCP3x6x::Offset));
  TEST_ASSERT_EQUAL_size_t(3, sizeof(MCP3x6x::Gain));
  TEST_ASSERT_EQUAL_size_t(2, sizeof(MCP3x6x::Crccfg));

  TEST_ASSERT_EQUAL_size_t(1, sizeof(MCP3x6x::status_t));

  TEST_ASSERT_EQUAL_size_t(4, sizeof(MCP3x6x::Adcdata));
}

#pragma once

#include <stdint.h>

/**
 * Base class for a kind of module which:
 *  - Needs to be setup once via a call to setup().
 *  - Needs to be invoked once per "processing cycle" via a call to loop().
 *  - Needs performance statistics gathered about performance during loop execution.
 *  - Needs periodic, summary performance statistics reported.
 * 
 * Usage:
 * Each module exists as a singleton. Somewhere, modules are
 * brought into existence by calling, typically, their
 * singleton() method. As each module comes into existence,
 * This TModule class keeps a list of them, in creation order.
 * 
 * After all modules are brought into existence, you call the
 * doSetup() method to invoke the setup() method for all registered
 * modules. Then in the main processing loop, you invoke the
 * doLoop() method to invoked the loop() method for all registered
 * modules.
 */

class TModule {
 public:
  // A list of all possible modules.
  typedef enum MODULE {
    ALARM,
    ALERT,
    MOTOR_CURRENT,
    PANEL_SELECTOR,
    RELAY,
    ROBOCLAW,
    ROS_CLIENT,
    SD,
    SERVER,
    SONAR,
    TEMPERATURE,
    TIME_OF_FLIGHT,
    NUMBER_MODULES // The number of all possible modules.
  } MODULE;

  // Call loop() for all registered modules.
  static void doLoop();

  // Call setup() for all registered modules.
  static void doSetup();

  // Perform regular, cyclic work for the module.
  virtual void loop() = 0;

  // Return module name.
  virtual const char* name() = 0;

  // Perform one-time setup for the module.
  virtual void setup() = 0;

 protected:
  TModule();

 private:
  // Define slots for gathering statistics for the module.
  typedef enum SLOT {
    MIN,
    MAX,
    SUM,
    NUMBER_SLOTS, // Number of slots to reserve for statistics.
    NUMBER_READINGS = 1000 // Number of statistical readings to gather before generating a summary report.
  } SLOT;

  // Reset the statistics for a module.
  static void resetReadings();

  // A list of all registered modules.
  static TModule* g_allModules[];

  // Index to next entry in the list of all registered modules.
  static uint8_t g_nextModuleNumber;
  
  // Index to the next statistic reading for the module.
  static int g_nextReadingNumber;

  // Statistics gathered for all registered modules.
  static float g_readings[NUMBER_MODULES][NUMBER_SLOTS];
};
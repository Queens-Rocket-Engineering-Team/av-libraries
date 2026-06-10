#ifndef AIM_CONFIG_STORE_H
#define AIM_CONFIG_STORE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "AimFileSystem.h"

/**
 * @brief Atomic JSON configuration store.
 * 
 * Provides methods to serialize/deserialize application-specific
 * configurations using ArduinoJson documents, ensuring power-loss
 * resilience through atomic renames.
 */
class AimConfigStore {
 public:
  explicit AimConfigStore(AimFileSystem& fs);

  /**
   * @brief Loads a JSON configuration file into a JsonDocument.
   * @param path The path to the config file (e.g., "/config.json").
   * @param doc The JsonDocument to populate.
   * @return true if loaded and parsed successfully.
   */
  bool load(const char* path, JsonDocument& doc);

  /**
   * @brief Atomically saves a JsonDocument to flash.
   * @param path The destination path.
   * @param doc The JsonDocument to serialize.
   * @return true if saved successfully.
   */
  bool save(const char* path, const JsonDocument& doc);

 private:
  AimFileSystem& _fs;
};

#endif // AIM_CONFIG_STORE_H

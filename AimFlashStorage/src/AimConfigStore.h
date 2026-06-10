#ifndef AIM_CONFIG_STORE_H
#define AIM_CONFIG_STORE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "AimFileSystem.h"

/**
 * @brief Detailed result of a config load attempt.
 *
 * NOT_PRESENT (no file, or empty file) is a normal state — e.g. first boot
 * or after a storage reset — and callers should fall back to compiled
 * defaults without treating it as an error.
 */
enum class AimConfigLoad : uint8_t {
  OK = 0,
  NOT_PRESENT,
  READ_FAILED,
  PARSE_FAILED,
  STORAGE_NOT_READY
};

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
   * @brief Loads a JSON config file, distinguishing missing-file from
   *        read and parse failures. See AimConfigLoad.
   */
  AimConfigLoad loadDetailed(const char* path, JsonDocument& doc);

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

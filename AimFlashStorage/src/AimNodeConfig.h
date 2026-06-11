#ifndef AIM_NODE_CONFIG_H
#define AIM_NODE_CONFIG_H

#include <Arduino.h>
#include <cstdint>
#include "AimConfigStore.h"
#include "AimFileSystem.h"

struct AimNodeCfg {
    char    boardName[32];
    uint8_t canId;
};

class AimNodeConfig {
public:
    AimNodeConfig(AimConfigStore& store, AimFileSystem& fs);

    // Overlays stored boardName/canId onto `out` (caller pre-fills compiled defaults).
    // NOT_PRESENT is normal on first boot — out is untouched.
    AimConfigLoad load(AimNodeCfg& out);

    // Persists boardName/canId, read-modify-writing to preserve other sections.
    bool save(const AimNodeCfg& cfg);

    // Removes boardName/canId so compiled defaults apply on next boot.
    // Falls back to deleting the file if the JSON is unreadable.
    bool reset();

    // Writes telemetry{cols, headers} into the config file if absent.
    AimConfigLoad ensureSchema(uint8_t cols, const char* const* headers);

    // Prints stored config between "[CFG]" / "[/CFG]" markers (always valid JSON).
    void print(Print& out);

private:
    static constexpr char kConfigPath[] = "/config.json";
    AimConfigStore& _store;
    AimFileSystem&  _fs;

    static void writeSchema(JsonDocument& doc, uint8_t cols, const char* const* headers);
};

#endif // AIM_NODE_CONFIG_H

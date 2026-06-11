#include "AimNodeConfig.h"

#include <cstring>
#include <ArduinoJson.h>
#include <logger.h>

constexpr char AimNodeConfig::kConfigPath[];

AimNodeConfig::AimNodeConfig(AimConfigStore& store, AimFileSystem& fs)
    : _store(store), _fs(fs) {}

void AimNodeConfig::writeSchema(JsonDocument& doc, uint8_t cols, const char* const* headers) {
    JsonObject telemetry = doc["telemetry"].to<JsonObject>();
    telemetry["cols"] = cols;
    JsonArray arr = telemetry["headers"].to<JsonArray>();
    for (uint8_t i = 0U; i < cols; i++) {
        arr.add(headers[i]);
    }
}

AimConfigLoad AimNodeConfig::load(AimNodeCfg& out) {
    JsonDocument doc;
    const AimConfigLoad status = _store.loadDetailed(kConfigPath, doc);
    if (status != AimConfigLoad::OK) {
        return status;
    }
    if (doc["boardName"].is<const char*>()) {
        strlcpy(out.boardName, doc["boardName"], sizeof(out.boardName));
    }
    if (doc["canId"].is<uint8_t>()) {
        out.canId = doc["canId"];
    }
    return AimConfigLoad::OK;
}

bool AimNodeConfig::save(const AimNodeCfg& cfg) {
    JsonDocument doc;
    const AimConfigLoad loadStatus = _store.loadDetailed(kConfigPath, doc);
    if (loadStatus == AimConfigLoad::STORAGE_NOT_READY) {
        return false;
    }
    if (loadStatus != AimConfigLoad::OK) {
        doc.clear();
    }
    doc["boardName"] = cfg.boardName;
    doc["canId"]     = cfg.canId;
    return _store.save(kConfigPath, doc);
}

bool AimNodeConfig::reset() {
    JsonDocument doc;
    const AimConfigLoad loadStatus = _store.loadDetailed(kConfigPath, doc);
    if (loadStatus == AimConfigLoad::NOT_PRESENT) {
        return true;
    }
    if (loadStatus == AimConfigLoad::STORAGE_NOT_READY) {
        return false;
    }
    if (loadStatus != AimConfigLoad::OK) {
        return _fs.removeFile(kConfigPath);
    }
    doc.remove("boardName");
    doc.remove("canId");
    return _store.save(kConfigPath, doc);
}

AimConfigLoad AimNodeConfig::ensureSchema(uint8_t cols, const char* const* headers) {
    JsonDocument doc;
    const AimConfigLoad loadStatus = _store.loadDetailed(kConfigPath, doc);
    if (loadStatus == AimConfigLoad::READ_FAILED  ||
        loadStatus == AimConfigLoad::PARSE_FAILED ||
        loadStatus == AimConfigLoad::STORAGE_NOT_READY) {
        return loadStatus;
    }
    if (loadStatus == AimConfigLoad::OK && doc["telemetry"].is<JsonObject>()) {
        return AimConfigLoad::OK;
    }
    writeSchema(doc, cols, headers);
    return _store.save(kConfigPath, doc) ? AimConfigLoad::OK : AimConfigLoad::READ_FAILED;
}

void AimNodeConfig::print(Print& out) {
    out.println("[CFG]");
    JsonDocument doc;
    if (_store.loadDetailed(kConfigPath, doc) == AimConfigLoad::OK) {
        serializeJson(doc, out);
        out.println();
    } else {
        out.println("{}");
    }
    out.println("[/CFG]");
}

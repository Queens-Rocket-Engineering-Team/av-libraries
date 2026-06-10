#include "AimConfigStore.h"
#include <logger.h>

AimConfigStore::AimConfigStore(AimFileSystem& fs) : _fs(fs) {}

bool AimConfigStore::load(const char* path, JsonDocument& doc) {
  if (!_fs.isReady()) {
    return false;
  }

  lfs_t* lfs = _fs.getLfs();
  lfs_file_t file;

  int err = lfs_file_open(lfs, &file, path, LFS_O_RDONLY);
  if (err != LFS_ERR_OK) {
    return false;
  }

  lfs_soff_t size = lfs_file_size(lfs, &file);
  if (size <= 0) {
    lfs_file_close(lfs, &file);
    return false;
  }

  char buf[512];

  if ((size_t)size >= sizeof(buf)) {
    LOG_ERROR("Config file %s too large (%d bytes)", path, (int)size);
    lfs_file_close(lfs, &file);
    return false;
  }

  lfs_ssize_t read = lfs_file_read(lfs, &file, buf, size);
  lfs_file_close(lfs, &file);

  if (read != size) {
    LOG_ERROR("Config read failed for %s", path);
    return false;
  }

  buf[read] = '\0';

  DeserializationError error = deserializeJson(doc, buf);
  if (error) {
    LOG_ERROR("JSON parse failed for %s: %s", path, error.c_str());
    return false;
  }

  return true;
}

bool AimConfigStore::save(const char* path, const JsonDocument& doc) {
  if (!_fs.isReady()) {
    return false;
  }

  char buf[512];

  size_t jsonSize = serializeJson(doc, buf, sizeof(buf));
  if (jsonSize == 0 || jsonSize >= sizeof(buf)) {
    LOG_ERROR("Config too large to serialize for %s", path);
    return false;
  }

  lfs_t* lfs = _fs.getLfs();
  lfs_file_t file;

  char tmpPath[64];
  snprintf(tmpPath, sizeof(tmpPath), "%s.tmp", path);

  int err = lfs_file_open(
      lfs,
      &file,
      tmpPath,
      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC
  );

  if (err != LFS_ERR_OK) {
    LOG_ERROR("Config tmp open failed for %s", tmpPath);
    return false;
  }

  lfs_ssize_t written = lfs_file_write(lfs, &file, buf, jsonSize);
  lfs_file_close(lfs, &file);

  if (written != (lfs_ssize_t)jsonSize) {
    LOG_ERROR("Config write failed for %s", tmpPath);
    lfs_remove(lfs, tmpPath);
    return false;
  }

  err = lfs_rename(lfs, tmpPath, path);
  if (err != LFS_ERR_OK) {
    LOG_ERROR("Config rename failed %s -> %s", tmpPath, path);
    lfs_remove(lfs, tmpPath);
    return false;
  }

  return true;
}

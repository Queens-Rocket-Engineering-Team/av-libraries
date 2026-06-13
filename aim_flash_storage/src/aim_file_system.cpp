#include "aim_file_system.h"
#include <logger.h>
#include <string.h>

AimFileSystem::AimFileSystem(AimBlockDevice* device)
    : _device(device),
      _mounted(false) {
  
  // Configure lfs_config
  memset(&_lfs_cfg, 0, sizeof(_lfs_cfg));
  _lfs_cfg.context = this;
  _lfs_cfg.read = &AimFileSystem::lfs_read;
  _lfs_cfg.prog = &AimFileSystem::lfs_prog;
  _lfs_cfg.erase = &AimFileSystem::lfs_erase;
  _lfs_cfg.sync = &AimFileSystem::lfs_sync;
}

AimFileSystem::~AimFileSystem() {
  end();
}

bool AimFileSystem::begin() {
  if (_mounted) end();

  if (!_device->begin()) {
    LOG_ERROR("Flash device hardware begin failed");
    return false;
  }

  fillGeometry();

  // Validate geometry before calling into littlefs to avoid assertions
  if (_lfs_cfg.read_size == 0 || _lfs_cfg.block_count == 0) {
    LOG_ERROR("Flash geometry invalid (read=%u, blocks=%u)", _lfs_cfg.read_size, _lfs_cfg.block_count);
    return false;
  }

  // Try mounting
  int err = lfs_mount(&_lfs, &_lfs_cfg);
  if (err) {
    LOG_WARN("Flash mount failed (err=%d), attempting format", err);
    if (format()) {
      err = 0;
    } else {
      LOG_ERROR("Flash format failed");
    }
  }

  if (err) return false;
  _mounted = true;
  LOG_INFO("Flash mounted: %u blocks of %u bytes", _lfs_cfg.block_count, _lfs_cfg.block_size);

  return true;
}

void AimFileSystem::fillGeometry() {
  _lfs_cfg.read_size = _device->read_size();
  _lfs_cfg.prog_size = _device->prog_size();
  _lfs_cfg.block_size = _device->block_size();
  _lfs_cfg.block_count = _device->block_count();
  _lfs_cfg.block_cycles = _device->block_cycles();
  _lfs_cfg.cache_size = _device->cache_size();
  _lfs_cfg.lookahead_size = _device->lookahead_size();
}

void AimFileSystem::end() {
  if (_mounted) {
    lfs_unmount(&_lfs);
    _mounted = false;
  }
}

uint32_t AimFileSystem::getUsedSize() {
  if (!_mounted) return 0;
  lfs_ssize_t size = lfs_fs_size(&_lfs);
  if (size < 0) return 0;
  return (uint32_t)size * _lfs_cfg.block_size;
}

bool AimFileSystem::removeFile(const char* path) {
  if (!_mounted) return false;
  return lfs_remove(&_lfs, path) == 0;
}

bool AimFileSystem::format() {
  end();

  // If the device isn't started or geometry is missing, attempt to fetch it now.
  if (_lfs_cfg.read_size == 0 || _lfs_cfg.block_count == 0) {
    if (!_device->begin()) {
      LOG_ERROR("Flash format aborted: device begin failed");
      return false;
    }
    fillGeometry();
  }

  if (_lfs_cfg.read_size == 0 || _lfs_cfg.block_count == 0) {
    LOG_ERROR("Flash format aborted: invalid geometry");
    return false;
  }

  LOG_INFO("Flash formatting...");
  int err = lfs_format(&_lfs, &_lfs_cfg);
  if (err) {
    LOG_ERROR("Flash format execution failed (err=%d)", err);
    return false;
  }

  // Remount so the system can continue using the flash immediately
  err = lfs_mount(&_lfs, &_lfs_cfg);
  if (err == LFS_ERR_OK) {
    _mounted = true;
  } else {
    LOG_ERROR("Flash mount after format failed (err=%d)", err);
    return false;
  }

  return true;
}

// littlefs static wrappers
int AimFileSystem::lfs_read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) {
  return static_cast<AimFileSystem*>(c->context)->_device->read(c, block, off, buffer, size);
}

int AimFileSystem::lfs_prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) {
  return static_cast<AimFileSystem*>(c->context)->_device->prog(c, block, off, buffer, size);
}

int AimFileSystem::lfs_erase(const struct lfs_config* c, lfs_block_t block) {
  return static_cast<AimFileSystem*>(c->context)->_device->erase(c, block);
}

int AimFileSystem::lfs_sync(const struct lfs_config* c) {
  return static_cast<AimFileSystem*>(c->context)->_device->sync(c);
}

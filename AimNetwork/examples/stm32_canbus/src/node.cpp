#include "node.h"

bool nodeHandleCanPacket(const aim::Pkt& pkt, uint32_t networkNowMs, AimNetwork& aim) {
  (void)pkt;
  (void)networkNowMs;
  (void)aim;

  // NODE EXTENSION POINT: add node-specific packet handling here when this
  return true;
}

void nodeUpdate(uint32_t schedulerNowMs) {
  (void)schedulerNowMs;

  // NODE EXTENSION POINT: add recurring node-local behavior here.
}
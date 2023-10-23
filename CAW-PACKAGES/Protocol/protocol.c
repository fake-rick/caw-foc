#include "protocol.h"

#include <math.h>
#include <string.h>

#include "endian.h"
#include "log.h"

// |cmd|action(0:set 1:get)|version|accuracy(0:int32)||data(4bytes)|
/**
 * @description: 解包数据
 * @param {PROTOCOL_T*} protocol
 * @param {uint8_t*} data
 * @return {*}
 */
void PROTOCOL_Unpack(PROTOCOL_T* protocol, const uint8_t* buf) {
  memcpy(protocol, buf, sizeof(PROTOCOL_T) - sizeof(float));
  int32_t data =
      (int32_t)(EndianUInt32(*((uint32_t*)(buf + PROTOCOL_DATA_OFFSET))));
  if (protocol->accuracy > 0) {
    protocol->data = (float)data / (pow(10, protocol->accuracy));
  } else {
    protocol->data = data;
  }
  // CAW_LOG_DEBUG("recv data => %f", protocol->data);
}

void PROTOCOL_Pack(uint8_t cmd, uint8_t action, uint8_t accuracy, float data,
                   uint8_t* buf) {
  buf[0] = cmd;
  buf[1] = action;
  buf[2] = PROTOCOL_VERSION;
  buf[3] = accuracy;
  *((uint32_t*)(buf + PROTOCOL_DATA_OFFSET)) =
      (uint32_t)EndianUInt32(data * pow(10, accuracy));
}
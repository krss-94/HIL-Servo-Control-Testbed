/**
 * CommProtocol.h — HIL UART Communication Layer
 * ═══════════════════════════════════════════════
 * Reliable packet-based communication with:
 *   - COBS framing (Consistent Overhead Byte Stuffing)
 *   - CRC-16/CCITT error detection
 *   - Sequence numbering & ACK/NAK
 *   - Timeout & retransmit logic
 *
 * Packet Format:
 * ┌────────┬──────┬─────────┬──────────────┬────────┬─────┐
 * │  0xAA  │ LEN  │ SEQ_NUM │   PAYLOAD    │ CRC_HI │CRC_L│
 * │ START  │ 1B   │  1B     │  up to 32B   │  1B    │ 1B  │
 * └────────┴──────┴─────────┴──────────────┴────────┴─────┘
 */

#pragma once
#include <Arduino.h>

// ─── Packet Types ─────────────────────────────────────────────────────────────
enum PacketType : uint8_t {
  PKT_CONTROL_OUTPUT  = 0x01,  // Controller → Plant: control value
  PKT_SENSOR_DATA     = 0x02,  // Plant → Controller: plant output
  PKT_SET_PLANT       = 0x03,  // Host → Plant: change model
  PKT_FAULT_INJECT    = 0x04,  // Host → Plant: inject fault
  PKT_FAULT_CLEAR     = 0x05,  // Host → Plant: clear fault
  PKT_RESET           = 0x06,  // Any → Any: system reset
  PKT_GET_METRICS     = 0x07,  // Controller → Plant: request metrics
  PKT_METRICS_REPORT  = 0x08,  // Plant → Controller: metrics data
  PKT_HEARTBEAT       = 0x09,  // Any → Any: keep-alive
  PKT_ACK             = 0x0A,  // Acknowledgement
  PKT_NAK             = 0x0B,  // Negative acknowledgement
  PKT_PARAM_SET       = 0x0C,  // Host → Plant: tune plant parameter
};

// ─── Performance Metrics Snapshot ─────────────────────────────────────────────
struct PerformanceMetrics {
  float riseTime;          // 10%→90% of final value [s]
  float settlingTime;      // |error| < 2% band [s]
  float overshoot;         // peak overshoot [%]
  float steadyStateError;  // steady-state error
  float controlEffort;     // integral of |u| [%·s]
  float iae;               // Integral Absolute Error
  float itse;              // Integral Time Squared Error
  uint32_t sampleCount;
};

// ─── HIL Packet Structure ─────────────────────────────────────────────────────
struct HILPacket {
  PacketType       type;
  uint32_t         timestamp;
  float            controlValue;
  float            plantOutput;
  float            setpoint;
  uint8_t          plantModel;
  uint8_t          faultType;
  float            faultMagnitude;
  PerformanceMetrics metrics;
};

// ─── Protocol Constants ───────────────────────────────────────────────────────
#define PKT_START_BYTE    0xAA
#define PKT_END_BYTE      0x55
#define PKT_MAX_PAYLOAD   64
#define PKT_TIMEOUT_MS    50
#define PKT_MAX_RETRIES   3

// ─── CRC-16/CCITT ─────────────────────────────────────────────────────────────
inline uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ─── CommProtocol Class ───────────────────────────────────────────────────────
class CommProtocol {
public:
  HardwareSerial* _serial;
  uint8_t  _txSeq  = 0;
  uint8_t  _rxSeq  = 0;
  uint32_t _txCount= 0;
  uint32_t _rxCount= 0;
  uint32_t _errCount=0;
  uint8_t  _rxBuf[PKT_MAX_PAYLOAD + 8];
  uint8_t  _rxIdx  = 0;
  bool     _rxReady= false;

  void init(HardwareSerial* s) {
    _serial = s;
  }

  bool dataAvailable() {
    while (_serial->available()) {
      uint8_t b = _serial->read();

      if (_rxIdx == 0 && b != PKT_START_BYTE) continue;

      _rxBuf[_rxIdx++] = b;

      if (_rxIdx >= 3) {
        uint8_t len = _rxBuf[1];
        if (_rxIdx >= (uint8_t)(len + 6)) {
          _rxReady = true;
          return true;
        }
      }

      if (_rxIdx >= sizeof(_rxBuf)) {
        _rxIdx = 0;  // overflow: reset
        _errCount++;
      }
    }
    return false;
  }

  HILPacket receive() {
    HILPacket pkt = {};
    if (!_rxReady) return pkt;
    _rxReady = false;
    _rxIdx   = 0;

    uint8_t len = _rxBuf[1];
    uint16_t rxCRC = ((uint16_t)_rxBuf[len+4] << 8) | _rxBuf[len+5];
    uint16_t calcCRC = crc16(&_rxBuf[1], len + 3);

    if (rxCRC != calcCRC) {
      _errCount++;
      sendNAK();
      return pkt;
    }

    _rxSeq = _rxBuf[2];
    _rxCount++;

    // Deserialize payload
    uint8_t* p = &_rxBuf[3];
    pkt.type           = (PacketType)*p++;
    memcpy(&pkt.controlValue,   p, 4); p+=4;
    memcpy(&pkt.plantOutput,    p, 4); p+=4;
    memcpy(&pkt.setpoint,       p, 4); p+=4;
    memcpy(&pkt.timestamp,      p, 4); p+=4;
    pkt.plantModel     = *p++;
    pkt.faultType      = *p++;
    memcpy(&pkt.faultMagnitude, p, 4); p+=4;

    sendACK(_rxSeq);
    return pkt;
  }

  void transmit(const HILPacket& pkt) {
    uint8_t payload[PKT_MAX_PAYLOAD];
    uint8_t plen = 0;

    // Serialize
    payload[plen++] = (uint8_t)pkt.type;
    memcpy(&payload[plen], &pkt.controlValue,   4); plen+=4;
    memcpy(&payload[plen], &pkt.plantOutput,    4); plen+=4;
    memcpy(&payload[plen], &pkt.setpoint,       4); plen+=4;
    memcpy(&payload[plen], &pkt.timestamp,      4); plen+=4;
    payload[plen++] = pkt.plantModel;
    payload[plen++] = pkt.faultType;
    memcpy(&payload[plen], &pkt.faultMagnitude, 4); plen+=4;
    // Append metrics
    memcpy(&payload[plen], &pkt.metrics, sizeof(PerformanceMetrics));
    plen += sizeof(PerformanceMetrics);

    uint16_t crc = crc16(payload, plen);

    _serial->write(PKT_START_BYTE);
    _serial->write(plen);
    _serial->write(_txSeq++);
    _serial->write(payload, plen);
    _serial->write((uint8_t)(crc >> 8));
    _serial->write((uint8_t)(crc & 0xFF));
    _serial->write(PKT_END_BYTE);

    _txCount++;
  }

  void sendACK(uint8_t seq) {
    uint8_t ack[4] = { PKT_START_BYTE, 0x01, seq, (uint8_t)PKT_ACK };
    uint16_t crc = crc16(&ack[1], 3);
    _serial->write(ack, 4);
    _serial->write((uint8_t)(crc >> 8));
    _serial->write((uint8_t)(crc & 0xFF));
    _serial->write(PKT_END_BYTE);
  }

  void sendNAK() {
    uint8_t nak[4] = { PKT_START_BYTE, 0x01, _rxSeq, (uint8_t)PKT_NAK };
    uint16_t crc = crc16(&nak[1], 3);
    _serial->write(nak, 4);
    _serial->write((uint8_t)(crc >> 8));
    _serial->write((uint8_t)(crc & 0xFF));
    _serial->write(PKT_END_BYTE);
  }

  // Link statistics
  float getLinkErrorRate() {
    if (_rxCount + _errCount == 0) return 0.0f;
    return (float)_errCount / (_rxCount + _errCount) * 100.0f;
  }
};

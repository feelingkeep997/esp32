// ============================================================================
// mavlink_compat.h —— 精简版 MAVLink v1 实现（仅覆盖本项目用到的消息）
//
// 职责:
//   不依赖官方 mavlink 库，自行实现 MAVLink v1 的组包/解包、CRC 与消息 pack/decode。
//   供 mavlink.cpp 使用，也可被 espnow.cpp 引用（取 MAVLINK_MAX_PACKET_LEN）。
//
// 关键逻辑:
//   - 帧格式：0xFE | len | seq | sysid | compid | msgid | payload | crc(2)
//     （MAVLink v1，帧头 6 字节 + 载荷 + 2 字节 CRC）；
//   - CRC 采用 CRC-16/MCRF4XX（查表 mavlink_crc_x25_table），并按消息 ID 取种子；
//   - mavlink_parse_char() 是逐字节状态机，跨调用保存状态（mavlink_status_t）。
//
// 输入/输出:
//   *_pack()  把字段写入 mavlink_message_t（不序列化）；
//   mavlink_msg_to_send_buffer() 把消息序列化成字节流；
//   mavlink_parse_char() 逐字节输入，组包完成时返回 true 并填充 mavlink_message_t。
//
// 边界情况与潜在风险:
//   - 消息 ID 存在冲突/占位：SCALED_IMU(26) 与 ACTUATOR_CONTROL_TARGET(26)、
//     SET_ACTUATOR_CONTROL_TARGET(26) 相同；且 mavlink_msg_scaled_imu_pack 里
//     msgid 直接硬编码为 26。与标准地面站交互时需以实际用到的消息为准。
//   - CRC 种子表只覆盖"本项目用到的消息"，未列出的消息种子返回 0，会导致 CRC 校验失败。
//   - pack 函数只填 msg->payload 与 len，不填 seq；seq 由调用方/静态变量维护（当前未递增）。
//   - 载荷字段布局按 MAVLink 的"字段重排"规则手工排列，改动消息定义需同步改 pack 与 decode。
//   - decode 函数按固定偏移读取 payload，若收到的消息 len 不足会读到越界数据（未做长度校验）。
//   - 若要获得完整协议支持，应替换为官方 mavlink/c_library_v2。
// ============================================================================

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define MAVLINK_MAX_PACKET_LEN 280
#define MAVLINK_COMM_0 0

// Message IDs
#define MAVLINK_MSG_ID_HEARTBEAT              0
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE     245
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION    31
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW        35
#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET 26
#define MAVLINK_MSG_ID_SCALED_IMU             26  // Note: conflicts, use raw
#define MAVLINK_MSG_ID_PARAM_VALUE            22
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST     21
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ     20
#define MAVLINK_MSG_ID_PARAM_SET              23
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST   43
#define MAVLINK_MSG_ID_MISSION_COUNT          44
#define MAVLINK_MSG_ID_SERIAL_CONTROL         126
#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET    82
#define MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET 26
#define MAVLINK_MSG_ID_LOG_REQUEST_DATA       119
#define MAVLINK_MSG_ID_LOG_DATA               120
#define MAVLINK_MSG_ID_COMMAND_LONG           76
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION      148
#define MAVLINK_MSG_ID_COMMAND_ACK            77

#define MAVLINK_MSG_ID_SCALED_IMU_REAL        26

// MAVLink enums
#define MAV_TYPE_QUADROTOR 2
#define MAV_AUTOPILOT_GENERIC 3
#define MAV_COMP_ID_AUTOPILOT1 1
#define MAV_MODE_FLAG_SAFETY_ARMED 128
#define MAV_MODE_FLAG_STABILIZE_ENABLED 4
#define MAV_MODE_FLAG_AUTO_ENABLED 16
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED 64
#define MAV_STATE_STANDBY 3
#define MAV_VTOL_STATE_UNDEFINED 0
#define MAV_LANDED_STATE_ON_GROUND 1
#define MAV_LANDED_STATE_IN_AIR 2
#define MAV_PARAM_TYPE_REAL32 9
#define MAV_MISSION_TYPE_MISSION 0
#define SERIAL_CONTROL_DEV_SHELL 4
#define SERIAL_CONTROL_FLAG_MULTI 2
#define ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE 7
#define MAV_CMD_REQUEST_MESSAGE 512
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#define MAV_CMD_DO_SET_MODE 176
#define MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT 2
#define MAV_PROTOCOL_CAPABILITY_MAVLINK2 1
#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_UNSUPPORTED 1

// Field lengths
#define MAVLINK_MSG_ID_PARAM_SET_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_ID_SERIAL_CONTROL_FIELD_DATA_LEN 70

typedef struct {
	uint16_t len;
	uint8_t incompat_flags;
	uint8_t compat_flags;
	uint8_t seq;
	uint8_t sysid;
	uint8_t compid;
	uint8_t msgid;
	uint8_t payload[MAVLINK_MAX_PACKET_LEN - 12];
	uint16_t checksum;
} mavlink_message_t;

typedef struct {
	uint8_t msg_received;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t buffer_pos;
	uint8_t current_rx_seq;
	uint8_t parse_state;
	uint8_t magic;
	uint16_t packet_len;
	uint16_t packet_rx_drop_count;
	uint16_t crc_offset;
} mavlink_status_t;

// CRC table (MAVLink CRC-16/MCRF4XX)
static const uint16_t mavlink_crc_x25_table[256] = {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x7487,0x4444,0x5425,
	0xa5ca,0xb5eb,0x8588,0x95a9,0xe56e,0xf54f,0xc52c,0xd50d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

// CRC seed values per message ID (simplified - only for messages we use)
static inline uint16_t mavlink_crc_seed(uint8_t msgid) {
	// MAVLink uses per-message CRC seeds. This is a simplified table.
	// In the real library, each message has its own seed.
	switch (msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: return 50;
		case MAVLINK_MSG_ID_PARAM_VALUE: return 220;
		case MAVLINK_MSG_ID_PARAM_SET: return 168;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: return 214;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: return 159;
		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: return 246;
		case MAVLINK_MSG_ID_COMMAND_LONG: return 152;
		case MAVLINK_MSG_ID_COMMAND_ACK: return 9;
		case MAVLINK_MSG_ID_SERIAL_CONTROL: return 189;
		case MAVLINK_MSG_ID_MISSION_COUNT: return 114;
		case MAVLINK_MSG_ID_LOG_DATA: return 134;
		case MAVLINK_MSG_ID_AUTOPILOT_VERSION: return 231;
		default: return 0;
	}
}

static inline uint16_t mavlink_crc_accumulate(const uint8_t *data, uint16_t len, uint16_t crc) {
	for (uint16_t i = 0; i < len; i++) {
		crc = (crc << 8) ^ mavlink_crc_x25_table[((crc >> 8) ^ data[i]) & 0xFF];
	}
	return crc;
}

// Pack a message into a buffer for sending
static inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg) {
	// MAVLink v1 format: 0xFE | len | seq | sysid | compid | msgid | payload | crc
	uint16_t payload_len = msg->len;
	buf[0] = 0xFE; // MAVLink v1 magic
	buf[1] = (uint8_t)payload_len;
	buf[2] = msg->seq;
	buf[3] = msg->sysid;
	buf[4] = msg->compid;
	buf[5] = msg->msgid;
	memcpy(&buf[6], msg->payload, payload_len);
	// CRC
	uint16_t crc = mavlink_crc_accumulate(&buf[1], 5 + payload_len, mavlink_crc_seed(msg->msgid));
	buf[6 + payload_len] = (uint8_t)(crc & 0xFF);
	buf[6 + payload_len + 1] = (uint8_t)(crc >> 8);
	return 6 + payload_len + 2; // header + payload + crc
}

// Parse a single byte - returns true when a complete message is received
static inline bool mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *msg, mavlink_status_t *status) {
	(void)chan;
	// Simplified MAVLink v1 parser
	if (status->parse_state == 0) {
		if (c == 0xFE) {
			status->buffer[0] = c;
			status->parse_state = 1;
			status->buffer_pos = 1;
		}
	} else if (status->parse_state == 1) {
		status->buffer[1] = c;
		status->packet_len = c + 8; // payload + 6 header + 2 crc
		status->parse_state = 2;
		status->buffer_pos = 2;
	} else {
		status->buffer[status->buffer_pos++] = c;
		if (status->buffer_pos >= status->packet_len) {
			// Complete packet
			uint16_t payload_len = status->buffer[1];
			uint16_t crc = mavlink_crc_accumulate(&status->buffer[1], 5 + payload_len, mavlink_crc_seed(status->buffer[5]));
			uint16_t recv_crc = status->buffer[6 + payload_len] | (status->buffer[6 + payload_len + 1] << 8);
			if (crc == recv_crc) {
				msg->len = payload_len;
				msg->seq = status->buffer[2];
				msg->sysid = status->buffer[3];
				msg->compid = status->buffer[4];
				msg->msgid = status->buffer[5];
				memcpy(msg->payload, &status->buffer[6], payload_len);
				status->parse_state = 0;
				status->msg_received = 1;
				return true;
			}
			status->parse_state = 0;
		}
	}
	return false;
}

// ============================================================================
// Message pack functions
// ============================================================================

static inline void mavlink_msg_heartbeat_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t custom_mode, uint8_t system_status) {
	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 9;
	memset(msg->payload, 0, sizeof(msg->payload));
	msg->payload[0] = type;
	msg->payload[1] = autopilot;
	msg->payload[2] = base_mode;
	msg->payload[3] = custom_mode;
	msg->payload[4] = system_status;
	msg->payload[5] = 3; // mavlink version 3
}

static inline void mavlink_msg_extended_sys_state_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t vtol_state, uint8_t landed_state) {
	msg->msgid = MAVLINK_MSG_ID_EXTENDED_SYS_STATE;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 2;
	msg->payload[0] = vtol_state;
	msg->payload[1] = landed_state;
}

static inline void mavlink_msg_attitude_quaternion_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, float q0, float q1, float q2, float q3, float rollspeed, float pitchspeed, float yawspeed,
	const float *offset) {
	(void)offset;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 32;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	memcpy(p, &q0, 4); p += 4;
	memcpy(p, &q1, 4); p += 4;
	memcpy(p, &q2, 4); p += 4;
	memcpy(p, &q3, 4); p += 4;
	memcpy(p, &rollspeed, 4); p += 4;
	memcpy(p, &pitchspeed, 4); p += 4;
	memcpy(p, &yawspeed, 4); p += 4;
}

static inline void mavlink_msg_rc_channels_raw_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, uint8_t port, uint16_t c0, uint16_t c1, uint16_t c2, uint16_t c3,
	uint16_t c4, uint16_t c5, uint16_t c6, uint16_t c7, uint8_t rssi) {
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 22;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	*p++ = port;
	memcpy(p, &c0, 2); p += 2;
	memcpy(p, &c1, 2); p += 2;
	memcpy(p, &c2, 2); p += 2;
	memcpy(p, &c3, 2); p += 2;
	memcpy(p, &c4, 2); p += 2;
	memcpy(p, &c5, 2); p += 2;
	memcpy(p, &c6, 2); p += 2;
	memcpy(p, &c7, 2); p += 2;
	*p = rssi;
}

static inline void mavlink_msg_actuator_control_target_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, uint8_t group, const float *controls) {
	msg->msgid = MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 41;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	*p++ = group;
	memcpy(p, controls, 32);
}

static inline void mavlink_msg_scaled_imu_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro,
	int16_t xmag, int16_t ymag, int16_t zmag, int16_t temp) {
	msg->msgid = 26;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 24;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	memcpy(p, &xacc, 2); p += 2;
	memcpy(p, &yacc, 2); p += 2;
	memcpy(p, &zacc, 2); p += 2;
	memcpy(p, &xgyro, 2); p += 2;
	memcpy(p, &ygyro, 2); p += 2;
	memcpy(p, &zgyro, 2); p += 2;
	memcpy(p, &xmag, 2); p += 2;
	memcpy(p, &ymag, 2); p += 2;
	memcpy(p, &zmag, 2); p += 2;
	memcpy(p, &temp, 2);
}

static inline void mavlink_msg_param_value_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index) {
	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 25;
	uint8_t *p = msg->payload;
	memcpy(p, &param_value, 4); p += 4;
	memcpy(p, &param_type, 1); p += 1;
	memcpy(p, &param_count, 2); p += 2;
	memcpy(p, &param_index, 2); p += 2;
	memset(p, 0, 16);
	strncpy((char*)p, param_id, 16);
}

static inline void mavlink_msg_mission_count_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t target_sys, uint8_t target_comp, uint16_t count, uint8_t mission_type, uint8_t opaque) {
	(void)opaque;
	msg->msgid = MAVLINK_MSG_ID_MISSION_COUNT;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 5;
	uint8_t *p = msg->payload;
	*p++ = target_sys;
	*p++ = target_comp;
	memcpy(p, &count, 2); p += 2;
	*p = mission_type;
}

static inline void mavlink_msg_serial_control_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t *data, uint8_t a, uint8_t b) {
	(void)a; (void)b;
	msg->msgid = MAVLINK_MSG_ID_SERIAL_CONTROL;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 79;
	uint8_t *p = msg->payload;
	*p++ = device;
	*p++ = flags;
	memcpy(p, &timeout, 2); p += 2;
	memcpy(p, &baudrate, 4); p += 4;
	*p++ = count;
	memset(p, 0, 70);
	memcpy(p, data, count);
}

static inline void mavlink_msg_log_data_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint16_t id, uint32_t ofs, uint8_t count, const uint8_t *data) {
	msg->msgid = MAVLINK_MSG_ID_LOG_DATA;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 90;
	uint8_t *p = msg->payload;
	memcpy(p, &id, 2); p += 2;
	memcpy(p, &ofs, 4); p += 4;
	*p++ = count;
	memset(p, 0, 90);
	memcpy(p, data, count);
}

static inline void mavlink_msg_autopilot_version_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version,
	uint32_t board_version, uint16_t flight_custom_version, uint16_t middleware_custom_version,
	uint16_t os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, uint8_t uid2) {
	(void)uid2;
	msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 60;
	uint8_t *p = msg->payload;
	memset(p, 0, 60);
	memcpy(p, &capabilities, 8); p += 8;
	memcpy(p, &flight_sw_version, 4); p += 4;
	memcpy(p, &middleware_sw_version, 4); p += 4;
	memcpy(p, &os_sw_version, 4); p += 4;
	memcpy(p, &board_version, 4); p += 4;
	memcpy(p, &flight_custom_version, 2); p += 2;
	memcpy(p, &middleware_custom_version, 2); p += 2;
	memcpy(p, &os_custom_version, 2); p += 2;
	memcpy(p, &vendor_id, 2); p += 2;
	memcpy(p, &product_id, 2); p += 2;
	memcpy(p, &uid, 8);
}

static inline void mavlink_msg_command_ack_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_sys, uint8_t target_comp) {
	msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 9;
	uint8_t *p = msg->payload;
	memcpy(p, &command, 2); p += 2;
	*p++ = result;
	*p++ = progress;
	memcpy(p, &result_param2, 4); p += 4;
	*p = target_sys;
}

// ============================================================================
// Decode functions
// ============================================================================

typedef struct { uint8_t target_system; uint16_t param_index; char param_id[16]; } mavlink_param_request_read_t;
typedef struct { uint8_t target_system; } mavlink_param_request_list_t;
typedef struct { uint8_t target_system; float param_value; uint8_t param_type; char param_id[16]; } mavlink_param_set_t;
typedef struct { uint8_t target_system; uint8_t target_component; } mavlink_mission_request_list_t;
typedef struct { uint8_t target_system; uint16_t count; uint8_t mission_type; } mavlink_mission_count_t;
typedef struct { uint8_t target_system; uint8_t device; uint16_t flags; uint16_t timeout; uint32_t baudrate; uint8_t count; uint8_t data[70]; } mavlink_serial_control_t;
typedef struct { uint8_t target_system; uint8_t target_component; uint8_t type_mask; float q[4]; float body_roll_rate; float body_pitch_rate; float body_yaw_rate; float thrust; } mavlink_set_attitude_target_t;
typedef struct { uint8_t target_system; uint8_t target_component; uint8_t group_mlx; float controls[8]; } mavlink_set_actuator_control_target_t;
typedef struct { uint16_t id; uint32_t ofs; uint16_t count; } mavlink_log_request_data_t;
typedef struct { uint8_t target_system; uint16_t command; float param1; float param2; float param3; float param4; float param5; float param6; float param7; } mavlink_command_long_t;
typedef struct { int16_t x, y, z, r; uint16_t buttons; int16_t target; } mavlink_manual_control_t;

static inline void mavlink_msg_param_request_list_decode(const mavlink_message_t *msg, mavlink_param_request_list_t *m) {
	m->target_system = msg->payload[0];
}

static inline void mavlink_msg_param_request_read_decode(const mavlink_message_t *msg, mavlink_param_request_read_t *m) {
	m->target_system = msg->payload[0];
	m->param_index = msg->payload[1] | (msg->payload[2] << 8);
	memset(m->param_id, 0, 16);
	memcpy(m->param_id, &msg->payload[3], 16);
}

static inline void mavlink_msg_param_set_decode(const mavlink_message_t *msg, mavlink_param_set_t *m) {
	memcpy(&m->param_value, &msg->payload[0], 4);
	m->param_type = msg->payload[4];
	memset(m->param_id, 0, 16);
	memcpy(m->param_id, &msg->payload[5], 16);
	m->target_system = msg->payload[21];
}

static inline void mavlink_msg_mission_request_list_decode(const mavlink_message_t *msg, mavlink_mission_request_list_t *m) {
	m->target_system = msg->payload[0];
	m->target_component = msg->payload[1];
}

static inline void mavlink_msg_serial_control_decode(const mavlink_message_t *msg, mavlink_serial_control_t *m) {
	m->device = msg->payload[0];
	m->flags = msg->payload[1] | (msg->payload[2] << 8);
	memcpy(&m->timeout, &msg->payload[3], 2);
	memcpy(&m->baudrate, &msg->payload[5], 4);
	m->count = msg->payload[9];
	memcpy(m->data, &msg->payload[10], 70);
	m->target_system = 0;
}

static inline void mavlink_msg_set_attitude_target_decode(const mavlink_message_t *msg, mavlink_set_attitude_target_t *m) {
	m->target_system = msg->payload[0];
	m->target_component = msg->payload[1];
	m->type_mask = msg->payload[2];
	memcpy(m->q, &msg->payload[4], 16);
	memcpy(&m->body_roll_rate, &msg->payload[20], 4);
	memcpy(&m->body_pitch_rate, &msg->payload[24], 4);
	memcpy(&m->body_yaw_rate, &msg->payload[28], 4);
	memcpy(&m->thrust, &msg->payload[32], 4);
}

static inline void mavlink_msg_set_actuator_control_target_decode(const mavlink_message_t *msg, mavlink_set_actuator_control_target_t *m) {
	memcpy(m->controls, &msg->payload[5], 32);
	m->target_system = msg->payload[0];
	m->target_component = msg->payload[1];
}

static inline void mavlink_msg_log_request_data_decode(const mavlink_message_t *msg, mavlink_log_request_data_t *m) {
	memcpy(&m->id, &msg->payload[0], 2);
	memcpy(&m->ofs, &msg->payload[2], 4);
	memcpy(&m->count, &msg->payload[6], 2);
}

static inline void mavlink_msg_command_long_decode(const mavlink_message_t *msg, mavlink_command_long_t *m) {
	memcpy(&m->param1, &msg->payload[0], 4);
	memcpy(&m->param2, &msg->payload[4], 4);
	memcpy(&m->param3, &msg->payload[8], 4);
	memcpy(&m->param4, &msg->payload[12], 4);
	memcpy(&m->param5, &msg->payload[16], 4);
	memcpy(&m->param6, &msg->payload[20], 4);
	memcpy(&m->param7, &msg->payload[24], 4);
	m->command = msg->payload[28] | (msg->payload[29] << 8);
	m->target_system = msg->payload[30];
}

static inline void mavlink_msg_manual_control_decode(const mavlink_message_t *msg, mavlink_manual_control_t *m) {
	memcpy(&m->x, &msg->payload[0], 2);
	memcpy(&m->y, &msg->payload[2], 2);
	memcpy(&m->z, &msg->payload[4], 2);
	memcpy(&m->r, &msg->payload[6], 2);
	m->buttons = msg->payload[8] | (msg->payload[9] << 8);
	memcpy(&m->target, &msg->payload[10], 2);
}

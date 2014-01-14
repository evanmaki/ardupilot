// MESSAGE RALLY_LAND_CMD_ACK PACKING

#define MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK 180

typedef struct __mavlink_rally_land_cmd_ack_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t success; ///< Success (1 if command RALLY_LAND_CMD successully executed, 0 otherwise)
} mavlink_rally_land_cmd_ack_t;

#define MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN 3
#define MAVLINK_MSG_ID_180_LEN 3

#define MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_CRC 184
#define MAVLINK_MSG_ID_180_CRC 184



#define MAVLINK_MESSAGE_INFO_RALLY_LAND_CMD_ACK { \
	"RALLY_LAND_CMD_ACK", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rally_land_cmd_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rally_land_cmd_ack_t, target_component) }, \
         { "success", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rally_land_cmd_ack_t, success) }, \
         } \
}


/**
 * @brief Pack a rally_land_cmd_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param success Success (1 if command RALLY_LAND_CMD successully executed, 0 otherwise)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_land_cmd_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, success);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#else
	mavlink_rally_land_cmd_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.success = success;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif
}

/**
 * @brief Pack a rally_land_cmd_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param success Success (1 if command RALLY_LAND_CMD successully executed, 0 otherwise)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_land_cmd_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, success);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#else
	mavlink_rally_land_cmd_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.success = success;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif
}

/**
 * @brief Encode a rally_land_cmd_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rally_land_cmd_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_land_cmd_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rally_land_cmd_ack_t* rally_land_cmd_ack)
{
	return mavlink_msg_rally_land_cmd_ack_pack(system_id, component_id, msg, rally_land_cmd_ack->target_system, rally_land_cmd_ack->target_component, rally_land_cmd_ack->success);
}

/**
 * @brief Encode a rally_land_cmd_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rally_land_cmd_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_land_cmd_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rally_land_cmd_ack_t* rally_land_cmd_ack)
{
	return mavlink_msg_rally_land_cmd_ack_pack_chan(system_id, component_id, chan, msg, rally_land_cmd_ack->target_system, rally_land_cmd_ack->target_component, rally_land_cmd_ack->success);
}

/**
 * @brief Send a rally_land_cmd_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param success Success (1 if command RALLY_LAND_CMD successully executed, 0 otherwise)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rally_land_cmd_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, success);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK, buf, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK, buf, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif
#else
	mavlink_rally_land_cmd_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.success = success;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK, (const char *)&packet, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK, (const char *)&packet, MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif
#endif
}

#endif

// MESSAGE RALLY_LAND_CMD_ACK UNPACKING


/**
 * @brief Get field target_system from rally_land_cmd_ack message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_rally_land_cmd_ack_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from rally_land_cmd_ack message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_rally_land_cmd_ack_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field success from rally_land_cmd_ack message
 *
 * @return Success (1 if command RALLY_LAND_CMD successully executed, 0 otherwise)
 */
static inline uint8_t mavlink_msg_rally_land_cmd_ack_get_success(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a rally_land_cmd_ack message into a struct
 *
 * @param msg The message to decode
 * @param rally_land_cmd_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_rally_land_cmd_ack_decode(const mavlink_message_t* msg, mavlink_rally_land_cmd_ack_t* rally_land_cmd_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	rally_land_cmd_ack->target_system = mavlink_msg_rally_land_cmd_ack_get_target_system(msg);
	rally_land_cmd_ack->target_component = mavlink_msg_rally_land_cmd_ack_get_target_component(msg);
	rally_land_cmd_ack->success = mavlink_msg_rally_land_cmd_ack_get_success(msg);
#else
	memcpy(rally_land_cmd_ack, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RALLY_LAND_CMD_ACK_LEN);
#endif
}

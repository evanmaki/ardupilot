// MESSAGE EKF_STATE PACKING

#define MAVLINK_MSG_ID_EKF_STATE 19

typedef struct __mavlink_ekf_state_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t lat; ///< Latitude, expressed as * 1E7
 int32_t lon; ///< Longitude, expressed as * 1E7
 int32_t alt; ///< Altitude in meters above home altitude, expressed as * 1000 (millimeters), 
 int32_t relative_alt; ///< Fused altitude, expressed as * 1000 (millimeters)
 float roll; ///< Roll angle (rad, -pi..+pi)
 float pitch; ///< Pitch angle (rad, -pi..+pi)
 float yaw; ///< Yaw angle (rad, -pi..+pi)
 float gyro_x; ///< Roll angular speed (rad/s)
 float gyro_y; ///< Pitch angular speed (rad/s)
 float gyro_z; ///< Yaw angular speed (rad/s)
 int16_t vx; ///< Fused North velocity, expressed as m/s * 100
 int16_t vy; ///< Fused East velocity, expressed as m/s * 100
 int16_t vz; ///< Fused Down velocity, expressed as m/s * 100
} mavlink_ekf_state_t;

#define MAVLINK_MSG_ID_EKF_STATE_LEN 50
#define MAVLINK_MSG_ID_19_LEN 50

#define MAVLINK_MSG_ID_EKF_STATE_CRC 116
#define MAVLINK_MSG_ID_19_CRC 116



#define MAVLINK_MESSAGE_INFO_EKF_STATE { \
	"EKF_STATE", \
	14, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ekf_state_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_ekf_state_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ekf_state_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_ekf_state_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ekf_state_t, relative_alt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ekf_state_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ekf_state_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ekf_state_t, yaw) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ekf_state_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ekf_state_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ekf_state_t, gyro_z) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_ekf_state_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_ekf_state_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_ekf_state_t, vz) }, \
         } \
}


/**
 * @brief Pack a ekf_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters above home altitude, expressed as * 1000 (millimeters), 
 * @param relative_alt Fused altitude, expressed as * 1000 (millimeters)
 * @param vx Fused North velocity, expressed as m/s * 100
 * @param vy Fused East velocity, expressed as m/s * 100
 * @param vz Fused Down velocity, expressed as m/s * 100
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param gyro_x Roll angular speed (rad/s)
 * @param gyro_y Pitch angular speed (rad/s)
 * @param gyro_z Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, float roll, float pitch, float yaw, float gyro_x, float gyro_y, float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_STATE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);
	_mav_put_float(buf, 32, gyro_x);
	_mav_put_float(buf, 36, gyro_y);
	_mav_put_float(buf, 40, gyro_z);
	_mav_put_int16_t(buf, 44, vx);
	_mav_put_int16_t(buf, 46, vy);
	_mav_put_int16_t(buf, 48, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_STATE_LEN);
#else
	mavlink_ekf_state_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EKF_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_STATE_LEN, MAVLINK_MSG_ID_EKF_STATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
}

/**
 * @brief Pack a ekf_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters above home altitude, expressed as * 1000 (millimeters), 
 * @param relative_alt Fused altitude, expressed as * 1000 (millimeters)
 * @param vx Fused North velocity, expressed as m/s * 100
 * @param vy Fused East velocity, expressed as m/s * 100
 * @param vz Fused Down velocity, expressed as m/s * 100
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param gyro_x Roll angular speed (rad/s)
 * @param gyro_y Pitch angular speed (rad/s)
 * @param gyro_z Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,float roll,float pitch,float yaw,float gyro_x,float gyro_y,float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_STATE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);
	_mav_put_float(buf, 32, gyro_x);
	_mav_put_float(buf, 36, gyro_y);
	_mav_put_float(buf, 40, gyro_z);
	_mav_put_int16_t(buf, 44, vx);
	_mav_put_int16_t(buf, 46, vy);
	_mav_put_int16_t(buf, 48, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_STATE_LEN);
#else
	mavlink_ekf_state_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EKF_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_STATE_LEN, MAVLINK_MSG_ID_EKF_STATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
}

/**
 * @brief Encode a ekf_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ekf_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ekf_state_t* ekf_state)
{
	return mavlink_msg_ekf_state_pack(system_id, component_id, msg, ekf_state->time_boot_ms, ekf_state->lat, ekf_state->lon, ekf_state->alt, ekf_state->relative_alt, ekf_state->vx, ekf_state->vy, ekf_state->vz, ekf_state->roll, ekf_state->pitch, ekf_state->yaw, ekf_state->gyro_x, ekf_state->gyro_y, ekf_state->gyro_z);
}

/**
 * @brief Encode a ekf_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ekf_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ekf_state_t* ekf_state)
{
	return mavlink_msg_ekf_state_pack_chan(system_id, component_id, chan, msg, ekf_state->time_boot_ms, ekf_state->lat, ekf_state->lon, ekf_state->alt, ekf_state->relative_alt, ekf_state->vx, ekf_state->vy, ekf_state->vz, ekf_state->roll, ekf_state->pitch, ekf_state->yaw, ekf_state->gyro_x, ekf_state->gyro_y, ekf_state->gyro_z);
}

/**
 * @brief Send a ekf_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters above home altitude, expressed as * 1000 (millimeters), 
 * @param relative_alt Fused altitude, expressed as * 1000 (millimeters)
 * @param vx Fused North velocity, expressed as m/s * 100
 * @param vy Fused East velocity, expressed as m/s * 100
 * @param vz Fused Down velocity, expressed as m/s * 100
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param gyro_x Roll angular speed (rad/s)
 * @param gyro_y Pitch angular speed (rad/s)
 * @param gyro_z Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ekf_state_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, float roll, float pitch, float yaw, float gyro_x, float gyro_y, float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_STATE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);
	_mav_put_float(buf, 32, gyro_x);
	_mav_put_float(buf, 36, gyro_y);
	_mav_put_float(buf, 40, gyro_z);
	_mav_put_int16_t(buf, 44, vx);
	_mav_put_int16_t(buf, 46, vy);
	_mav_put_int16_t(buf, 48, vz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, buf, MAVLINK_MSG_ID_EKF_STATE_LEN, MAVLINK_MSG_ID_EKF_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, buf, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
#else
	mavlink_ekf_state_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, (const char *)&packet, MAVLINK_MSG_ID_EKF_STATE_LEN, MAVLINK_MSG_ID_EKF_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, (const char *)&packet, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_EKF_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ekf_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, float roll, float pitch, float yaw, float gyro_x, float gyro_y, float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);
	_mav_put_float(buf, 32, gyro_x);
	_mav_put_float(buf, 36, gyro_y);
	_mav_put_float(buf, 40, gyro_z);
	_mav_put_int16_t(buf, 44, vx);
	_mav_put_int16_t(buf, 46, vy);
	_mav_put_int16_t(buf, 48, vz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, buf, MAVLINK_MSG_ID_EKF_STATE_LEN, MAVLINK_MSG_ID_EKF_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, buf, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
#else
	mavlink_ekf_state_t *packet = (mavlink_ekf_state_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->relative_alt = relative_alt;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->gyro_x = gyro_x;
	packet->gyro_y = gyro_y;
	packet->gyro_z = gyro_z;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, (const char *)packet, MAVLINK_MSG_ID_EKF_STATE_LEN, MAVLINK_MSG_ID_EKF_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATE, (const char *)packet, MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE EKF_STATE UNPACKING


/**
 * @brief Get field time_boot_ms from ekf_state message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_ekf_state_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from ekf_state message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_ekf_state_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from ekf_state message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_ekf_state_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from ekf_state message
 *
 * @return Altitude in meters above home altitude, expressed as * 1000 (millimeters), 
 */
static inline int32_t mavlink_msg_ekf_state_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field relative_alt from ekf_state message
 *
 * @return Fused altitude, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_ekf_state_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vx from ekf_state message
 *
 * @return Fused North velocity, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_ekf_state_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field vy from ekf_state message
 *
 * @return Fused East velocity, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_ekf_state_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  46);
}

/**
 * @brief Get field vz from ekf_state message
 *
 * @return Fused Down velocity, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_ekf_state_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field roll from ekf_state message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_ekf_state_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from ekf_state message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_ekf_state_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from ekf_state message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_ekf_state_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field gyro_x from ekf_state message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_ekf_state_get_gyro_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field gyro_y from ekf_state message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_ekf_state_get_gyro_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field gyro_z from ekf_state message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_ekf_state_get_gyro_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a ekf_state message into a struct
 *
 * @param msg The message to decode
 * @param ekf_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_ekf_state_decode(const mavlink_message_t* msg, mavlink_ekf_state_t* ekf_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	ekf_state->time_boot_ms = mavlink_msg_ekf_state_get_time_boot_ms(msg);
	ekf_state->lat = mavlink_msg_ekf_state_get_lat(msg);
	ekf_state->lon = mavlink_msg_ekf_state_get_lon(msg);
	ekf_state->alt = mavlink_msg_ekf_state_get_alt(msg);
	ekf_state->relative_alt = mavlink_msg_ekf_state_get_relative_alt(msg);
	ekf_state->roll = mavlink_msg_ekf_state_get_roll(msg);
	ekf_state->pitch = mavlink_msg_ekf_state_get_pitch(msg);
	ekf_state->yaw = mavlink_msg_ekf_state_get_yaw(msg);
	ekf_state->gyro_x = mavlink_msg_ekf_state_get_gyro_x(msg);
	ekf_state->gyro_y = mavlink_msg_ekf_state_get_gyro_y(msg);
	ekf_state->gyro_z = mavlink_msg_ekf_state_get_gyro_z(msg);
	ekf_state->vx = mavlink_msg_ekf_state_get_vx(msg);
	ekf_state->vy = mavlink_msg_ekf_state_get_vy(msg);
	ekf_state->vz = mavlink_msg_ekf_state_get_vz(msg);
#else
	memcpy(ekf_state, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_EKF_STATE_LEN);
#endif
}

// MESSAGE NavEKF PACKING

#define MAVLINK_MSG_ID_NavEKF 19

typedef struct __mavlink_navekf_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float roll; ///< Roll angle (rad, -pi..+pi)
 float pitch; ///< Pitch angle (rad, -pi..+pi)
 float yaw; ///< Yaw angle (rad, -pi..+pi)
 float pos_n; ///< North Position in NED
 float pos_e; ///< East Position  in  NED
 float pos_d; ///< Down Position  in  NED
 float vel_n; ///< North Velocity in  NED
 float vel_e; ///< East Velocity  in  NED
 float vel_d; ///< Down Velocity  in  NED
 float gyro_x; ///< Roll angular speed (rad/s)
 float gyro_y; ///< Pitch angular speed (rad/s)
 float gyro_z; ///< Yaw angular speed (rad/s)
} mavlink_navekf_t;

#define MAVLINK_MSG_ID_NavEKF_LEN 52
#define MAVLINK_MSG_ID_19_LEN 52

#define MAVLINK_MSG_ID_NavEKF_CRC 26
#define MAVLINK_MSG_ID_19_CRC 26



#define MAVLINK_MESSAGE_INFO_NavEKF { \
	"NavEKF", \
	13, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_navekf_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_navekf_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_navekf_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_navekf_t, yaw) }, \
         { "pos_n", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_navekf_t, pos_n) }, \
         { "pos_e", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_navekf_t, pos_e) }, \
         { "pos_d", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_navekf_t, pos_d) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_navekf_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_navekf_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_navekf_t, vel_d) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_navekf_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_navekf_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_navekf_t, gyro_z) }, \
         } \
}


/**
 * @brief Pack a navekf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param pos_n North Position in NED
 * @param pos_e East Position  in  NED
 * @param pos_d Down Position  in  NED
 * @param vel_n North Velocity in  NED
 * @param vel_e East Velocity  in  NED
 * @param vel_d Down Velocity  in  NED
 * @param gyro_x Roll angular speed (rad/s)
 * @param gyro_y Pitch angular speed (rad/s)
 * @param gyro_z Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navekf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float roll, float pitch, float yaw, float pos_n, float pos_e, float pos_d, float vel_n, float vel_e, float vel_d, float gyro_x, float gyro_y, float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NavEKF_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, pos_n);
	_mav_put_float(buf, 20, pos_e);
	_mav_put_float(buf, 24, pos_d);
	_mav_put_float(buf, 28, vel_n);
	_mav_put_float(buf, 32, vel_e);
	_mav_put_float(buf, 36, vel_d);
	_mav_put_float(buf, 40, gyro_x);
	_mav_put_float(buf, 44, gyro_y);
	_mav_put_float(buf, 48, gyro_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NavEKF_LEN);
#else
	mavlink_navekf_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.pos_n = pos_n;
	packet.pos_e = pos_e;
	packet.pos_d = pos_d;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NavEKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NavEKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NavEKF_LEN, MAVLINK_MSG_ID_NavEKF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NavEKF_LEN);
#endif
}

/**
 * @brief Pack a navekf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param pos_n North Position in NED
 * @param pos_e East Position  in  NED
 * @param pos_d Down Position  in  NED
 * @param vel_n North Velocity in  NED
 * @param vel_e East Velocity  in  NED
 * @param vel_d Down Velocity  in  NED
 * @param gyro_x Roll angular speed (rad/s)
 * @param gyro_y Pitch angular speed (rad/s)
 * @param gyro_z Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navekf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float roll,float pitch,float yaw,float pos_n,float pos_e,float pos_d,float vel_n,float vel_e,float vel_d,float gyro_x,float gyro_y,float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NavEKF_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, pos_n);
	_mav_put_float(buf, 20, pos_e);
	_mav_put_float(buf, 24, pos_d);
	_mav_put_float(buf, 28, vel_n);
	_mav_put_float(buf, 32, vel_e);
	_mav_put_float(buf, 36, vel_d);
	_mav_put_float(buf, 40, gyro_x);
	_mav_put_float(buf, 44, gyro_y);
	_mav_put_float(buf, 48, gyro_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NavEKF_LEN);
#else
	mavlink_navekf_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.pos_n = pos_n;
	packet.pos_e = pos_e;
	packet.pos_d = pos_d;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NavEKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NavEKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NavEKF_LEN, MAVLINK_MSG_ID_NavEKF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NavEKF_LEN);
#endif
}

/**
 * @brief Encode a navekf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param navekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navekf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_navekf_t* navekf)
{
	return mavlink_msg_navekf_pack(system_id, component_id, msg, navekf->time_boot_ms, navekf->roll, navekf->pitch, navekf->yaw, navekf->pos_n, navekf->pos_e, navekf->pos_d, navekf->vel_n, navekf->vel_e, navekf->vel_d, navekf->gyro_x, navekf->gyro_y, navekf->gyro_z);
}

/**
 * @brief Encode a navekf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param navekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navekf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_navekf_t* navekf)
{
	return mavlink_msg_navekf_pack_chan(system_id, component_id, chan, msg, navekf->time_boot_ms, navekf->roll, navekf->pitch, navekf->yaw, navekf->pos_n, navekf->pos_e, navekf->pos_d, navekf->vel_n, navekf->vel_e, navekf->vel_d, navekf->gyro_x, navekf->gyro_y, navekf->gyro_z);
}

/**
 * @brief Send a navekf message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param pos_n North Position in NED
 * @param pos_e East Position  in  NED
 * @param pos_d Down Position  in  NED
 * @param vel_n North Velocity in  NED
 * @param vel_e East Velocity  in  NED
 * @param vel_d Down Velocity  in  NED
 * @param gyro_x Roll angular speed (rad/s)
 * @param gyro_y Pitch angular speed (rad/s)
 * @param gyro_z Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_navekf_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float pos_n, float pos_e, float pos_d, float vel_n, float vel_e, float vel_d, float gyro_x, float gyro_y, float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NavEKF_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, pos_n);
	_mav_put_float(buf, 20, pos_e);
	_mav_put_float(buf, 24, pos_d);
	_mav_put_float(buf, 28, vel_n);
	_mav_put_float(buf, 32, vel_e);
	_mav_put_float(buf, 36, vel_d);
	_mav_put_float(buf, 40, gyro_x);
	_mav_put_float(buf, 44, gyro_y);
	_mav_put_float(buf, 48, gyro_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, buf, MAVLINK_MSG_ID_NavEKF_LEN, MAVLINK_MSG_ID_NavEKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, buf, MAVLINK_MSG_ID_NavEKF_LEN);
#endif
#else
	mavlink_navekf_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.pos_n = pos_n;
	packet.pos_e = pos_e;
	packet.pos_d = pos_d;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, (const char *)&packet, MAVLINK_MSG_ID_NavEKF_LEN, MAVLINK_MSG_ID_NavEKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, (const char *)&packet, MAVLINK_MSG_ID_NavEKF_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NavEKF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_navekf_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float roll, float pitch, float yaw, float pos_n, float pos_e, float pos_d, float vel_n, float vel_e, float vel_d, float gyro_x, float gyro_y, float gyro_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, pos_n);
	_mav_put_float(buf, 20, pos_e);
	_mav_put_float(buf, 24, pos_d);
	_mav_put_float(buf, 28, vel_n);
	_mav_put_float(buf, 32, vel_e);
	_mav_put_float(buf, 36, vel_d);
	_mav_put_float(buf, 40, gyro_x);
	_mav_put_float(buf, 44, gyro_y);
	_mav_put_float(buf, 48, gyro_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, buf, MAVLINK_MSG_ID_NavEKF_LEN, MAVLINK_MSG_ID_NavEKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, buf, MAVLINK_MSG_ID_NavEKF_LEN);
#endif
#else
	mavlink_navekf_t *packet = (mavlink_navekf_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->pos_n = pos_n;
	packet->pos_e = pos_e;
	packet->pos_d = pos_d;
	packet->vel_n = vel_n;
	packet->vel_e = vel_e;
	packet->vel_d = vel_d;
	packet->gyro_x = gyro_x;
	packet->gyro_y = gyro_y;
	packet->gyro_z = gyro_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, (const char *)packet, MAVLINK_MSG_ID_NavEKF_LEN, MAVLINK_MSG_ID_NavEKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavEKF, (const char *)packet, MAVLINK_MSG_ID_NavEKF_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE NavEKF UNPACKING


/**
 * @brief Get field time_boot_ms from navekf message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_navekf_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from navekf message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_navekf_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from navekf message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_navekf_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from navekf message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_navekf_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pos_n from navekf message
 *
 * @return North Position in NED
 */
static inline float mavlink_msg_navekf_get_pos_n(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pos_e from navekf message
 *
 * @return East Position  in  NED
 */
static inline float mavlink_msg_navekf_get_pos_e(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pos_d from navekf message
 *
 * @return Down Position  in  NED
 */
static inline float mavlink_msg_navekf_get_pos_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vel_n from navekf message
 *
 * @return North Velocity in  NED
 */
static inline float mavlink_msg_navekf_get_vel_n(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vel_e from navekf message
 *
 * @return East Velocity  in  NED
 */
static inline float mavlink_msg_navekf_get_vel_e(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vel_d from navekf message
 *
 * @return Down Velocity  in  NED
 */
static inline float mavlink_msg_navekf_get_vel_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field gyro_x from navekf message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_navekf_get_gyro_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field gyro_y from navekf message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_navekf_get_gyro_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field gyro_z from navekf message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_navekf_get_gyro_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Decode a navekf message into a struct
 *
 * @param msg The message to decode
 * @param navekf C-struct to decode the message contents into
 */
static inline void mavlink_msg_navekf_decode(const mavlink_message_t* msg, mavlink_navekf_t* navekf)
{
#if MAVLINK_NEED_BYTE_SWAP
	navekf->time_boot_ms = mavlink_msg_navekf_get_time_boot_ms(msg);
	navekf->roll = mavlink_msg_navekf_get_roll(msg);
	navekf->pitch = mavlink_msg_navekf_get_pitch(msg);
	navekf->yaw = mavlink_msg_navekf_get_yaw(msg);
	navekf->pos_n = mavlink_msg_navekf_get_pos_n(msg);
	navekf->pos_e = mavlink_msg_navekf_get_pos_e(msg);
	navekf->pos_d = mavlink_msg_navekf_get_pos_d(msg);
	navekf->vel_n = mavlink_msg_navekf_get_vel_n(msg);
	navekf->vel_e = mavlink_msg_navekf_get_vel_e(msg);
	navekf->vel_d = mavlink_msg_navekf_get_vel_d(msg);
	navekf->gyro_x = mavlink_msg_navekf_get_gyro_x(msg);
	navekf->gyro_y = mavlink_msg_navekf_get_gyro_y(msg);
	navekf->gyro_z = mavlink_msg_navekf_get_gyro_z(msg);
#else
	memcpy(navekf, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NavEKF_LEN);
#endif
}

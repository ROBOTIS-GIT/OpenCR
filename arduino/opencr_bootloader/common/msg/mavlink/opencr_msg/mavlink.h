/** @file
 *	@brief MAVLink comm protocol built from opencr_msg.xml
 *	@see http://mavlink.org
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#ifndef MAVLINK_STX
#define MAVLINK_STX 254
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 0
#endif

#ifndef MAVLINK_PACKED
#define MAVLINK_PACKED __attribute__((__packed__))
#endif

#include "version.h"
#include "opencr_msg.h"

#endif // MAVLINK_H

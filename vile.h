/**
        libvile
        Copyright (C) 2022 Cat (Ivan Epifanov)

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __VILE_H__
#define __VILE_H__

#include <psp2/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum __attribute__ ((__packed__)) {
  NXT_OUT_A = 0x00,
  NXT_OUT_B = 0x01,
  NXT_OUT_C = 0x02,
  NXT_OUT_ALL = 0xFF
} vile_out_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_IN_1 = 0x00,
  NXT_IN_2 = 0x01,
  NXT_IN_3 = 0x02,
  NXT_IN_4 = 0x03
} vile_in_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_MOTOR_MODE_ON = 0x01,
  NXT_MOTOR_MODE_BRAKE = 0x02,
  NXT_MOTOR_MODE_REGULATED = 0x04
} vile_motor_mode_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_MOTOR_REGULATION_IDLE = 0x00,
  NXT_MOTOR_REGULATION_SPEED = 0x01,
  NXT_MOTOR_REGULATION_SYNC = 0x02
} vile_motor_regulation_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_MOTOR_RUNSTATE_IDLE = 0x00,
  NXT_MOTOR_RUNSTATE_RAMPUP = 0x10,
  NXT_MOTOR_RUNSTATE_RUNNING = 0x20,
  NXT_MOTOR_RUNSTATE_RAMPDOWN = 0x40
} vile_motor_runstate_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_SENSOR_NONE = 0x00,
  NXT_SENSOR_SWITCH,
  NXT_SENSOR_TEMPERATURE,
  NXT_SENSOR_REFLECTION,
  NXT_SENSOR_ANGLE,
  NXT_SENSOR_LIGHT_ACTIVE,
  NXT_SENSOR_LIGHT_INACTIVE,
  NXT_SENSOR_SOUND_DB,
  NXT_SENSOR_SOUND_DBA,
  NXT_SENSOR_CUSTOM,
  NXT_SENSOR_LOWSPEED,
  NXT_SENSOR_LOWSPEED_9V,
  NXT_SENSOR_NOST
} vile_sensor_type_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_SENSOR_MODE_RAW = 0x00,
  NXT_SENSOR_MODE_BOOLEAN = 0x20,
  NXT_SENSOR_MODE_TRANSITION_CNT = 0x40,
  NXT_SENSOR_MODE_PERIOD_CNT = 0x60,
  NXT_SENSOR_MODE_PCT_FULLSCALE = 0x80,
  NXT_SENSOR_MODE_CELSIUS = 0xA0,
  NXT_SENSOR_MODE_FAHRENHEIT = 0xC0,
  NXT_SENSOR_MODE_ANGLE_STEP = 0xE0,
  NXT_SENSOR_MASK_SLOPE = 0x1F,

  NXT_SENSOR_MASK_MODE = 0xE0
} vile_sensor_mode_t;

typedef enum __attribute__ ((__packed__)) {
  NXT_STATUS_OK = 0x00,
  NXT_STATUS_PENDING = 0x20,
  NXT_STATUS_QUEUE_EMPTY = 0x40,
  NXT_STATUS_SYS_NO_MORE_HANDLES = 0x81,
  NXT_STATUS_SYS_NO_SPACE = 0x82,
  NXT_STATUS_SYS_NO_MORE_FILES = 0x83,
  NXT_STATUS_SYS_EOF_EXPECTED = 0x84,
  NXT_STATUS_SYS_EOF = 0x85,
  NXT_STATUS_SYS_NOT_A_LINEAR_FILE = 0x86,
  NXT_STATUS_SYS_FILE_NOT_FOUND = 0x87,
  NXT_STATUS_SYS_HANDLE_ALREADY_CLOSED = 0x88,
  NXT_STATUS_SYS_NO_LINEAR_SPACE = 0x89,
  NXT_STATUS_SYS_UNDEFINED_ERROR = 0x8A,
  NXT_STATUS_SYS_FILE_BUSY = 0x8B,
  NXT_STATUS_SYS_NO_WRITE_BUFFERS = 0x8C,
  NXT_STATUS_SYS_APPEND_IMPOSSIBLE = 0x8D,
  NXT_STATUS_SYS_FILE_IS_FULL = 0x8E,
  NXT_STATUS_SYS_FILE_EXISTS = 0x8F,
  NXT_STATUS_SYS_MODULE_NOT_FOUND = 0x90,
  NXT_STATUS_SYS_OUT_OF_BOUNDARY = 0x91,
  NXT_STATUS_SYS_ILLEGAL_FILENAME = 0x92,
  NXT_STATUS_SYS_ILLEGAL_HANDLE = 0x93,
  NXT_STATUS_REQUEST_FAILED = 0xBD,
  NXT_STATUS_UNKNOWN_OPCODE = 0xBE,
  NXT_STATUS_INSANE_PACKET = 0xBF,
  NXT_STATUS_DATA_OUT_OF_RANGE = 0xC0,
  NXT_STATUS_COMMUNICATION_ERROR = 0xDD,
  NXT_STATUS_NO_BUFFER = 0xDE,
  NXT_STATUS_CHANNEL_INVALID = 0xDF,
  NXT_STATUS_CHANNEL_BUSY = 0xE0,
  NXT_STATUS_NO_ACTIVE_PROGRAM = 0xEC,
  NXT_STATUS_ILLEGAL_SIZE = 0xED,
  NXT_STATUS_ILLEGAL_MAILBOX = 0xEE,
  NXT_STATUS_ILLEGAL_FIELD = 0xEF,
  NXT_STATUS_BAD_IO = 0xF0,
  NXT_STATUS_NO_MEMORY = 0xFB,
  NXT_STATUS_BAD_ARGS = 0xFF
} vile_status_t;

#pragma pack(push,1)
typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint8_t port;
  int8_t power;
  uint8_t mode;
  uint8_t regulation;
  int8_t turn_ratio;
  uint8_t run_state;
  uint32_t tacho_limit;
  int32_t tacho_count;
  int32_t block_tacho_count;
  int32_t rotation_count;
} vile_outputstate_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint8_t port;
  uint8_t valid;
  uint8_t calibrated;
  uint8_t sensor_type;
  uint8_t sensor_mode;
  uint16_t raw_value;
  uint16_t normalized_value;
  int16_t scaled_value;
  int16_t calibrated_value;
} vile_inputstate_t __attribute__ ((aligned (64)));
#pragma pack(pop)


int vileStart();
int vileStop();

int vileHasNxt();

int vileStartProgram(const char *filename);
int vileStopProgram();
int vileGetCurrentProgramName(char* filename);
int vilePlaySoundfile(const char *filename, const unsigned short loop);
int vilePlayTone(const unsigned int freq, const unsigned int duration);
int vileStopSound();

typedef struct {
  vile_out_t port;
  int8_t power;
  vile_motor_mode_t mode;
  vile_motor_regulation_t regulation;
  int8_t turn_ratio;
  vile_motor_runstate_t run_state;
  uint32_t tacho_limit;
} vile_setoutputstate_t;

int vileSetOutputState(const vile_setoutputstate_t* outstate);
int vileGetOutputState(const vile_out_t port, vile_outputstate_t *out);

int vileSetInputMode(
  const vile_in_t port,
  const vile_sensor_type_t stype,
  const vile_sensor_mode_t smode
);

int vileGetInputValues(
  const vile_in_t port,
  vile_inputstate_t* out
);


int vileResetInputScaledValue(const vile_in_t port);
int vileResetMotorPosition(const vile_out_t port, const uint8_t relative);

int vileGetBatteryLevel();

#ifdef __cplusplus
}
#endif

#endif // __VILE_H__


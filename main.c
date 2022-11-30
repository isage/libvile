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

#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/cpu.h>
#include <psp2kern/kernel/debug.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/kernel/threadmgr/event_flags.h>
#include <psp2kern/kernel/sysmem/data_transfers.h>
#include <psp2kern/usbd.h>
#include <psp2kern/usbserv.h>
#include <string.h>
#include "vile.h"

const int NXT_USB_ID_VENDOR_LEGO = 0x0694;
const int NXT_USB_ID_PRODUCT_NXT = 0x0002;
const int NXT_USB_ENDPOINT_OUT = 0x01; //1
const int NXT_USB_ENDPOINT_IN = 0x82; //130
const int NXT_USB_TIMEOUT = 1000;
const int NXT_USB_READSIZE = 64;
const int NXT_USB_INTERFACE = 0;

enum {
  NXT_DIRECT_COMMAND_DOREPLY = 0x00,
  NXT_SYSTEM_COMMAND_DOREPLY = 0x01,
  NXT_COMMAND_REPLY = 0x02,
  NXT_DIRECT_COMMAND_NOREPLY = 0x80,
  NXT_SYSTEM_COMMAND_NOREPLY = 0x81,
};

enum {
  NXT_OPCODE_STARTPROGRAM = 0x00,
  NXT_OPCODE_STOPPROGRAM = 0x01,
  NXT_OPCODE_PLAYSOUND = 0x02,
  NXT_OPCODE_PLAYTONE = 0x03,
  NXT_OPCODE_SET_OUTPUTSTATE = 0x04,
  NXT_OPCODE_SET_INPUTMODE = 0x05,
  NXT_OPCODE_GET_OUTPUTSTATE = 0x06,
  NXT_OPCODE_GET_INPUTVALUES = 0x07,
  NXT_OPCODE_RESET_INPUT_SCALEDVALUES = 0x08,
  NXT_OPCODE_MESSAGE_WRITE = 0x09,
  NXT_OPCODE_MESSAGE_READ = 0x13,
  NXT_OPCODE_RESET_MOTOR_POSITION = 0x0A,
  NXT_OPCODE_BATTERYLEVEL = 0x0B,
  NXT_OPCODE_STOP_SOUND = 0x0C,
  NXT_OPCODE_KEEPALIVE = 0x0D,
  NXT_OPCODE_LS_GET_STATUS = 0x0E,
  NXT_OPCODE_LS_WRITE = 0x0F,
  NXT_OPCODE_LS_READ = 0x10,
  NXT_OPCODE_GET_CURRENTPROGRAM_NAME = 0x11,
  /** \todo system commands */
  NXT_OPCODE_SYS_OPENREAD = 0x80,
  NXT_OPCODE_SYS_OPENWRITE = 0x81,
  NXT_OPCODE_SYS_READ = 0x82,
  NXT_OPCODE_SYS_WRITE = 0x83,
  NXT_OPCODE_SYS_CLOSE = 0x84,
  NXT_OPCODE_SYS_DELETE = 0x85,
  NXT_OPCODE_SYS_FINDFIRST = 0x86,
  NXT_OPCODE_SYS_FINDNEXT = 0x87,
  NXT_OPCODE_SYS_GET_FIRMVAREVERSION = 0x88,
  NXT_OPCODE_SYS_OPENLINEARWRITE = 0x89,
  NXT_OPCODE_SYS_OPENLINEARREAD = 0x8A,
  NXT_OPCODE_SYS_OPENWRITEDATA = 0x8B,
  NXT_OPCODE_SYS_OPENAPPENDDATA = 0x8C,
  NXT_OPCODE_SYS_BOOT = 0x97,
  NXT_OPCODE_SYS_SETBRICKNAME = 0x98,
  NXT_OPCODE_SYS_GET_DEVICEINFO = 0x9B,
  NXT_OPCODE_SYS_DELETE_USERFLASH = 0xA0,
  NXT_OPCODE_SYS_POLLCOMMAND_LENGTH = 0xA1,
  NXT_OPCODE_SYS_POLLCOMMAND = 0xA2,
  NXT_OPCODE_SYS_RESET_BLUETOOTH = 0xA4
};

// packet return types
#pragma pack(push,1)

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
} ret_status_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint16_t mv;
} ret_battery_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint32_t msec;
} ret_keepalive_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  char filename[20];
} ret_currentprogram_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint8_t bytes_ready;
} ret_lsstatus_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint8_t bytes_read;
  char data[16];
} ret_lsread_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t status;
  uint8_t local_inbox;
  uint8_t msg_size;
  char data[58];
} ret_msgread_t __attribute__ ((aligned (64)));

// command packet types

typedef struct {
  uint8_t type;
  uint8_t opcode;
} cmd_simple_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t port;
} cmd_port_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t port;
  uint8_t relative;
} cmd_resetport_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  char filename[20];
} cmd_startprogram_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t loop;
  char filename[20];
} cmd_playsound_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint16_t freq;
  uint16_t duration;
} cmd_playtone_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t port;
  int8_t power;
  uint8_t mode;
  uint8_t regulation;
  int8_t turn_ratio;
  uint8_t run_state;
  uint32_t tacho_limit;
} cmd_setoutput_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t port;
  uint8_t stype;
  uint8_t smode;
} cmd_setinput_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t port;
  uint8_t tx_size;
  uint8_t rx_size;
  char data[20];
} cmd_lswrite_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t remote_inbox;
  uint8_t local_inbox;
  uint8_t remove;
} cmd_msgread_t __attribute__ ((aligned (64)));

typedef struct {
  uint8_t type;
  uint8_t opcode;
  uint8_t inbox;
  uint8_t message_size;
  char message[59];
} cmd_msgwrite_t __attribute__ ((aligned (64)));

#pragma pack(pop)

SceUID transfer_ev;
SceUID out_pipe_id = 0;
SceUID in_pipe_id = 0;

static uint8_t started = 0;
static uint8_t plugged = 0;

int vile_probe(int device_id);
int vile_attach(int device_id);
int vile_detach(int device_id);

static const SceUsbdDriver vileDriver = {
  .name = "vile",
  .probe = vile_probe,
  .attach = vile_attach,
  .detach = vile_detach,
};

static int vile_sysevent_handler(int resume, int eventid, void *args, void *opt)
{
  if (resume && started)
  {
    ksceUsbServMacSelect(2, 0); // re-set host mode
  }
  return 0;
}

static void set_config_done(int32_t result, int32_t count, void *arg)
{
  ksceDebugPrintf("config cb result: %08x, count: %d\n", result, count);
  ksceKernelSetEventFlag(transfer_ev, 4);
}

int vile_probe(int device_id)
{
  SceUsbdDeviceDescriptor *device;
  ksceDebugPrintf("probing device: %x\n", device_id);
  device = (SceUsbdDeviceDescriptor*)ksceUsbdScanStaticDescriptor(device_id, 0, SCE_USBD_DESCRIPTOR_DEVICE);
  if (device)
  {
    ksceDebugPrintf("vendor: %04x\n", device->idVendor);
    ksceDebugPrintf("product: %04x\n", device->idProduct);
    if (device->idVendor == NXT_USB_ID_VENDOR_LEGO && device->idProduct == NXT_USB_ID_PRODUCT_NXT)
    {
      ksceDebugPrintf("found NXT brick\n");
      return 0;
    }
  }

  return -1;
}

int vile_attach(int device_id)
{
  ksceDebugPrintf("attaching device: %x\n", device_id);
  SceUsbdDeviceDescriptor *device;
  device = (SceUsbdDeviceDescriptor*)ksceUsbdScanStaticDescriptor(device_id, 0, SCE_USBD_DESCRIPTOR_DEVICE);
  if (device && device->idVendor == NXT_USB_ID_VENDOR_LEGO && device->idProduct == NXT_USB_ID_PRODUCT_NXT)
  {

    SceUsbdConfigurationDescriptor *cdesc;
    if ((cdesc = (SceUsbdConfigurationDescriptor *)ksceUsbdScanStaticDescriptor(device_id, NULL, SCE_USBD_DESCRIPTOR_CONFIGURATION)) == NULL)
      return SCE_USBD_ATTACH_FAILED;

    if (cdesc->bNumInterfaces != 1)
      return SCE_USBD_ATTACH_FAILED;


    SceUsbdEndpointDescriptor *endpoint;
    ksceDebugPrintf("scanning endpoints\n");
    endpoint = (SceUsbdEndpointDescriptor*)ksceUsbdScanStaticDescriptor(device_id, device, SCE_USBD_DESCRIPTOR_ENDPOINT);
    while (endpoint)
    {
      ksceDebugPrintf("got EP: %02x\n", endpoint->bEndpointAddress);
      if (endpoint->bEndpointAddress == NXT_USB_ENDPOINT_IN)
      {
        ksceDebugPrintf("opening in pipe\n");
        in_pipe_id = ksceUsbdOpenPipe(device_id, endpoint);
        ksceDebugPrintf("= 0x%08x\n", in_pipe_id);
      }
      else if (endpoint->bEndpointAddress == NXT_USB_ENDPOINT_OUT)
      {
        ksceDebugPrintf("opening out pipe\n");
        out_pipe_id = ksceUsbdOpenPipe(device_id, endpoint);
        ksceDebugPrintf("= 0x%08x\n", out_pipe_id);
      }
      endpoint = (SceUsbdEndpointDescriptor*)ksceUsbdScanStaticDescriptor(device_id, endpoint, SCE_USBD_DESCRIPTOR_ENDPOINT);
    }

    SceUID control_pipe_id = ksceUsbdOpenPipe(device_id, NULL);
    ksceUsbdSetConfiguration(control_pipe_id, cdesc->bConfigurationValue, set_config_done, NULL);
    unsigned int matched;
    ksceDebugPrintf("waiting ef (cfg)\n");
    ksceKernelWaitEventFlag(transfer_ev, 4, SCE_EVENT_WAITCLEAR_PAT | SCE_EVENT_WAITAND, &matched, 0);

    if (out_pipe_id > 0 && in_pipe_id > 0)
    {
      plugged = 1;
      return 0;
    }
  }
  return -1;
}

int vile_detach(int device_id)
{
  in_pipe_id = 0;
  out_pipe_id = 0;
  plugged = 0;
  return -1;
}

int vileStart()
{
  uint32_t state;
  ENTER_SYSCALL(state);

  ksceDebugPrintf("starting ViLE\n");
  started = 1;
  int ret = ksceUsbServMacSelect(2, 0);
  ksceDebugPrintf("MAC select = 0x%08x\n", ret);
  ret = ksceUsbdRegisterDriver(&vileDriver);
  ksceDebugPrintf("ksceUsbdRegisterDriver = 0x%08x\n", ret);
  EXIT_SYSCALL(state);
  return 1;
}

int vileStop()
{
  uint32_t state;
  ENTER_SYSCALL(state);

  started = 0;
  plugged = 0;
  if (in_pipe_id) ksceUsbdClosePipe(in_pipe_id);
  if (out_pipe_id) ksceUsbdClosePipe(out_pipe_id);
  ksceUsbdUnregisterDriver(&vileDriver);
  ksceUsbServMacSelect(2, 1);

  EXIT_SYSCALL(state);

  return 1;
  // TODO: restore udcd?
}

int vileHasNxt()
{
//  ksceDebugPrintf("started: %d, plugged: %d\n", started, plugged);
  return (started && plugged);
}

void nxt_callback_send(int32_t result, int32_t count, void* arg)
{
  ksceDebugPrintf("send cb result: %08x, count: %d\n", result, count);
  *(int*)arg = count;
  ksceKernelSetEventFlag(transfer_ev, 1);
}

void nxt_callback_recv(int32_t result, int32_t count, void* arg)
{
//  ksceDebugPrintf("recv cb result: %08x, count: %d\n", result, count);
  *(int*)arg = count;
  ksceKernelSetEventFlag(transfer_ev, 2);
}

int nxt_send(unsigned char *request, unsigned int length)
{
  int transferred = 0;
  // transfer
  ksceDebugPrintf("sending 0x%08x\n", request);
  for (int i = 0; i < length; i++)
  {
    ksceDebugPrintf("%02x ", request[i]);
  }
  ksceDebugPrintf("\n");
  int ret = ksceUsbdBulkTransfer(out_pipe_id, request, length,  nxt_callback_send, &transferred);
  ksceDebugPrintf("send 0x%08x\n", ret);
  // wait for eventflag
  unsigned int matched;
  ksceDebugPrintf("waiting ef\n");
  ksceKernelWaitEventFlag(transfer_ev, 1, SCE_EVENT_WAITCLEAR_PAT | SCE_EVENT_WAITAND, &matched, 0);
  return transferred;
}

int nxt_recv(unsigned char *result)
{
  int transferred = 0;
  // transfer
//  ksceDebugPrintf("sending (recv) 0x%08x, len 64\n", result);
  int ret = ksceUsbdBulkTransfer(in_pipe_id, result, 64,  nxt_callback_recv, &transferred);
//  ksceDebugPrintf("send 0x%08x\n", ret);
  // wait for eventflag
  unsigned int matched;
  ksceDebugPrintf("waiting ef (recv)\n");
  ksceKernelWaitEventFlag(transfer_ev, 2, SCE_EVENT_WAITCLEAR_PAT | SCE_EVENT_WAITAND, &matched, 0);
  return transferred;
}

/*
 *  PUBLIC COMMANDS
 */

int vileStartProgram(const char *filename)
{

  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_startprogram_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_STARTPROGRAM, ""};
  strncat(cmd.filename, filename, 19);

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);
  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_STARTPROGRAM)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}

int vileStopProgram()
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_simple_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_STOPPROGRAM};

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_STOPPROGRAM)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}

int vileGetCurrentProgramName(char* filename)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_simple_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_GET_CURRENTPROGRAM_NAME};

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    return -1;
  }

  ret_currentprogram_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_currentprogram_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_GET_CURRENTPROGRAM_NAME)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  strncpy(filename, st.filename, 20);
  EXIT_SYSCALL(state);
  return 0;
}

int vilePlaySoundfile(const char *filename, const unsigned short loop)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_playsound_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_PLAYSOUND, loop, ""};
  strncat(cmd.filename, filename, 19);

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_PLAYSOUND)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return loop;
}


int vilePlayTone(const unsigned int freq, const unsigned int duration)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_playtone_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_PLAYTONE, freq, duration};

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_PLAYTONE)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}

int vileStopSound()
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_simple_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_STOP_SOUND};

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_STOP_SOUND)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}

int vileSetOutputState(
  const vile_setoutputstate_t* outstate
)
{
  uint32_t state;
  ENTER_SYSCALL(state);


  vile_setoutputstate_t koutstate;
  ksceKernelMemcpyUserToKernel(&koutstate, outstate, sizeof(vile_setoutputstate_t));

  cmd_setoutput_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_SET_OUTPUTSTATE, (uint8_t)koutstate.port, (int8_t)koutstate.power,
    (uint8_t)koutstate.mode, (uint8_t)koutstate.regulation, (int8_t)koutstate.turn_ratio, (uint8_t)koutstate.run_state, (uint32_t)koutstate.tacho_limit
  };

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_SET_OUTPUTSTATE)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);

  return 0;
}

int vileSetInputMode(
  const vile_in_t port,
  const vile_sensor_type_t stype,
  const vile_sensor_mode_t smode
)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_setinput_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_SET_INPUTMODE, port, stype, smode
  };

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_SET_INPUTMODE)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}


int vileGetOutputState(const vile_out_t port, vile_outputstate_t *out)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_port_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_GET_OUTPUTSTATE, port
  };

  vile_outputstate_t kout;

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  int ret = nxt_recv((unsigned char*) &kout);

  if (ret != sizeof (vile_outputstate_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (out->type != NXT_COMMAND_REPLY || out->opcode != NXT_OPCODE_GET_OUTPUTSTATE)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (out->status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ksceKernelMemcpyKernelToUser(out, &kout, sizeof(vile_outputstate_t));

  EXIT_SYSCALL(state);
  return 0;
}


int vileGetInputValues(const vile_in_t port, vile_inputstate_t *out)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_port_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_GET_INPUTVALUES, port
  };

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  vile_inputstate_t kout;

  int ret = nxt_recv((unsigned char*)&kout);

  if (ret != sizeof (vile_inputstate_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (kout.type != NXT_COMMAND_REPLY || kout.opcode != NXT_OPCODE_GET_INPUTVALUES)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (kout.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ksceKernelMemcpyKernelToUser(out, &kout, sizeof(vile_inputstate_t));

  EXIT_SYSCALL(state);
  return 0;
}


int vileResetInputScaledValue(const vile_in_t port)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_port_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_RESET_INPUT_SCALEDVALUES, port
  };

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_RESET_INPUT_SCALEDVALUES)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}

int vileResetMotorPosition(const vile_out_t port, const uint8_t relative)
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_resetport_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_RESET_MOTOR_POSITION, port, (relative > 0) ? 1 : 0
  };

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof (cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_status_t st;
  int ret = nxt_recv((unsigned char*) &st);

  if (ret != sizeof (ret_status_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_RESET_MOTOR_POSITION)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (st.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return 0;
}


int vileGetBatteryLevel()
{
  uint32_t state;
  ENTER_SYSCALL(state);

  cmd_simple_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_BATTERYLEVEL};

  int sent = nxt_send((unsigned char*) &cmd, sizeof (cmd));

  if (sent != sizeof(cmd))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  ret_battery_t bt;
  int ret = nxt_recv((unsigned char*) &bt);

  if (ret != sizeof (ret_battery_t))
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (bt.type != NXT_COMMAND_REPLY || bt.opcode != NXT_OPCODE_BATTERYLEVEL)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  if (bt.status != NXT_STATUS_OK)
  {
    EXIT_SYSCALL(state);
    return -1;
  }

  EXIT_SYSCALL(state);
  return bt.mv;
}

/*
int nxt_ls_get_status(
                      const libnxtusb_device_handle *handle, const libnxtusb_in_t port,
                      int* bytes_ready
                      ) {

  cmd_port_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_LS_GET_STATUS, port
  };

  int sent = nxt_send(handle, (const unsigned char*) &cmd, sizeof (cmd));

  ret_lsstatus_t st;
  int ret = nxt_recv(handle, (unsigned char*) &st);
  if (ret != sizeof (ret_lsstatus_t)) {
    return -1;
  }
  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_LS_GET_STATUS) {
    return -1;
  }
  if (st.status != NXT_STATUS_OK) {
    libnxtusb_error = st.status;
    return -1;
  }
  *bytes_ready = st.bytes_ready;

  return 0;
}

int nxt_ls_write(
                 const libnxtusb_device_handle *handle, const libnxtusb_in_t port,
                 const char* data, const uint8_t data_size, const uint8_t expected_data_size
                 ) {

  cmd_lswrite_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_LS_WRITE, port, data_size, expected_data_size
  };
  strncat(cmd.data, data, 19);

  int sent = nxt_send(handle, (const unsigned char*) &cmd, sizeof (cmd));

  ret_status_t st;
  int ret = nxt_recv(handle, (unsigned char*) &st);
  if (ret != sizeof (ret_status_t)) {
    return -1;
  }
  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_LS_WRITE) {
    return -1;
  }
  if (st.status != NXT_STATUS_OK) {
    libnxtusb_error = st.status;

    return -1;
  }
  return 0;
}

int nxt_ls_read(
                const libnxtusb_device_handle *handle, const libnxtusb_in_t port,
                char* data
                ) {

  cmd_port_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_LS_READ, port
  };

  int sent = nxt_send(handle, (const unsigned char*) &cmd, sizeof (cmd));

  ret_lsread_t st;
  int ret = nxt_recv(handle, (unsigned char*) &st);
  if (ret != sizeof (ret_lsread_t)) {
    return -1;
  }
  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_LS_READ) {
    return -1;
  }
  if (st.status != NXT_STATUS_OK) {
    libnxtusb_error = st.status;
    return -1;
  }
  strncpy(data, st.data, 16);

  return 0;
}

int nxt_message_write(
                      const libnxtusb_device_handle *handle, const uint8_t inbox,
                      char* message
                      ) {

  cmd_msgwrite_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_MESSAGE_WRITE, strlen(message) + 1
  };
  strncat(cmd.message, message, strlen(message) + 3);

  int sent = nxt_send(handle, (const unsigned char*) &cmd, sizeof (cmd));

  ret_status_t st;
  int ret = nxt_recv(handle, (unsigned char*) &st);
  if (ret != sizeof (ret_status_t)) {
    return -1;
  }
  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_MESSAGE_WRITE) {
    return -1;
  }
  if (st.status != NXT_STATUS_OK) {
    libnxtusb_error = st.status;

    return -1;
  }
  return 0;
}

int nxt_message_read(
                     const libnxtusb_device_handle *handle, const uint8_t remote_inbox,
                     uint8_t local_inbox, char* message, uint8_t remove
                     ) {

  cmd_msgread_t cmd = {
    NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_MESSAGE_READ,
    remote_inbox, local_inbox, remove
  };

  int sent = nxt_send(handle, (const unsigned char*) &cmd, sizeof (cmd));

  ret_msgread_t st;
  int ret = nxt_recv(handle, (unsigned char*) &st);
  if (ret != sizeof (ret_msgread_t)) {
    return -1;
  }
  if (st.type != NXT_COMMAND_REPLY || st.opcode != NXT_OPCODE_MESSAGE_READ) {
    return -1;
  }
  if (st.status != NXT_STATUS_OK) {
    libnxtusb_error = st.status;
    return -1;
  }
  strncpy(message, st.data, st.msg_size);

  return 0;
}

int nxt_keepalive(const libnxtusb_device_handle *handle, unsigned int* msec) {

  cmd_simple_t cmd = {NXT_DIRECT_COMMAND_DOREPLY, NXT_OPCODE_KEEPALIVE};

  int sent = nxt_send(handle, (const unsigned char*) &cmd, sizeof (cmd));
  if (sent != sizeof (cmd_simple_t)) {
    return -1;
  }

  ret_keepalive_t kt;
  int ret = nxt_recv(handle, (unsigned char*) &kt);
  if (ret != sizeof (ret_keepalive_t)) {
    return -1;
  }
  if (kt.type != NXT_COMMAND_REPLY || kt.opcode != NXT_OPCODE_KEEPALIVE) {
    return -1;
  }
  if (kt.status != NXT_STATUS_OK) {
    libnxtusb_error = kt.status;
    return -1;
  }
  *msec = kt.msec;

  return 0;
}

*/

void _start() __attribute__ ((weak, alias("module_start")));

int module_start(SceSize args, void *argp)
{
  ksceDebugPrintf("libViLE starting\n");
  ksceKernelRegisterSysEventHandler("zvile_sysevent", vile_sysevent_handler, NULL);
  transfer_ev = ksceKernelCreateEventFlag("vile_transfer", 0, 0, NULL);
  ksceDebugPrintf("ef: 0x%08x\n", transfer_ev);
  return SCE_KERNEL_START_SUCCESS;
}

int module_stop(SceSize args, void *argp)
{
  vileStop();
  return SCE_KERNEL_STOP_SUCCESS;
}



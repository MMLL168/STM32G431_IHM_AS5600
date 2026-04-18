#include "follow_tuning_mcp.h"

#include <string.h>

#include "follow_control.h"
#include "follow_tuning_runtime.h"
#include "mcp.h"
#include "user_sensored.h"

static uint8_t FollowTuningMcp_CopyResponse(const void *source,
                                            uint16_t response_size,
                                            int16_t tx_sync_free_space,
                                            uint16_t *tx_length,
                                            uint8_t *tx_buffer)
{
  if ((tx_length == (uint16_t *)0) || (tx_buffer == (uint8_t *)0))
  {
    return MCP_CMD_NOK;
  }

  if ((int16_t)response_size > tx_sync_free_space)
  {
    *tx_length = 0U;
    return MCP_ERROR_NO_TXSYNC_SPACE;
  }

  if ((source != (const void *)0) && (response_size > 0U))
  {
    (void)memcpy(tx_buffer, source, response_size);
  }

  *tx_length = response_size;
  return MCP_CMD_OK;
}

static uint8_t FollowTuningMcp_Callback(uint16_t rx_length,
                                        uint8_t *rx_buffer,
                                        int16_t tx_sync_free_space,
                                        uint16_t *tx_length,
                                        uint8_t *tx_buffer)
{
  FollowTuningProfile_t profile;
  FollowTuningSnapshot_t snapshot;
  FollowTuningHello_t hello;

  if ((rx_buffer == (uint8_t *)0) || (tx_length == (uint16_t *)0) || (tx_buffer == (uint8_t *)0))
  {
    return MCP_CMD_NOK;
  }

  if (rx_length < 1U)
  {
    *tx_length = 0U;
    return MCP_ERROR_BAD_RAW_FORMAT;
  }

  switch (rx_buffer[0])
  {
    case FOLLOW_TUNING_MCP_OPCODE_PING:
      FollowTuning_GetHello(&hello);
      return FollowTuningMcp_CopyResponse(&hello,
                                          (uint16_t)sizeof(hello),
                                          tx_sync_free_space,
                                          tx_length,
                                          tx_buffer);

    case FOLLOW_TUNING_MCP_OPCODE_GET_PROFILE:
      FollowTuning_GetProfileCopy(&profile);
      return FollowTuningMcp_CopyResponse(&profile,
                                          (uint16_t)sizeof(profile),
                                          tx_sync_free_space,
                                          tx_length,
                                          tx_buffer);

    case FOLLOW_TUNING_MCP_OPCODE_SET_PROFILE:
      if (rx_length != (uint16_t)(1U + sizeof(FollowTuningProfile_t)))
      {
        *tx_length = 0U;
        return MCP_ERROR_BAD_RAW_FORMAT;
      }

      (void)memcpy(&profile, &rx_buffer[1], sizeof(profile));
      FollowTuning_SetProfile(&profile);
      UserSensored_OnTuningProfileChanged();
      FollowControl_OnTuningProfileChanged();
      FollowTuning_GetProfileCopy(&profile);
      return FollowTuningMcp_CopyResponse(&profile,
                                          (uint16_t)sizeof(profile),
                                          tx_sync_free_space,
                                          tx_length,
                                          tx_buffer);

    case FOLLOW_TUNING_MCP_OPCODE_GET_SNAPSHOT:
      FollowTuning_GetSnapshot(&snapshot);
      return FollowTuningMcp_CopyResponse(&snapshot,
                                          (uint16_t)sizeof(snapshot),
                                          tx_sync_free_space,
                                          tx_length,
                                          tx_buffer);

    case FOLLOW_TUNING_MCP_OPCODE_RESET_PROFILE:
      FollowTuning_ResetProfile();
      UserSensored_OnTuningProfileChanged();
      FollowControl_OnTuningProfileChanged();
      FollowTuning_GetProfileCopy(&profile);
      return FollowTuningMcp_CopyResponse(&profile,
                                          (uint16_t)sizeof(profile),
                                          tx_sync_free_space,
                                          tx_length,
                                          tx_buffer);

    default:
      *tx_length = 0U;
      return MCP_CMD_UNKNOWN;
  }
}

void FollowTuningMcp_Init(void)
{
  FollowTuning_Init();
  (void)MCP_RegisterCallBack(FOLLOW_TUNING_MCP_CALLBACK_ID, &FollowTuningMcp_Callback);
}
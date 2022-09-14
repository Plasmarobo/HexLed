#include "comm_protocol.h"

#include "stm32l0xx_hal.h"

#include <stdint.h>

// Category 1 device: 512bytes of EEPROM
// Start of eeprom
#define EEPROM_START (0x08080000)
#define EEPROM_PAGE_SIZE (128)
#define EEPROM_SIZE (512)

#define DEVICE_ID_START (EEPROM_START)
#define RESERVATION_START (DEVICE_ID_START + EEPROM_PAGE_SIZE)
#define RESERVATION_HEADER (sizeof(uint32_t))

// RAM cache for device IDs
#define RESERVATION_CACHE_SIZE ((EEPROM_PAGE_SIZE / sizeof(device_id_t)) - sizeof(uint32_t))
// An ordered list of reservations
device_id_t reservation_cache[RESERVATION_CACHE_SIZE];

void set_device_id(device_id_t id)
{
  HAL_FLASHEx_DATAEEPROM_Unlock();
  // Erase the page
  HAL_FLASHEx_DATAEEPROM_Erase(DEVICE_ID_START);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, DEVICE_ID_START, id);
  HAL_FLASHEx_DATAEEPROM_Lock();
  HAL_Delay(100);
}

device_id_t get_device_id(void)
{
  device_id_t id = 0;
  id             = (*(__IO uint32_t*)DEVICE_ID_START);
  return id;
}

// Used to determine if an ID exists in our cache
// Operates on RAM cache
bool id_cache_search(device_id_t id)
{
  // Search the list in RAM
  uint32_t index = 0;
  bool     found = false;
  while (index < RESERVATION_CACHE_SIZE)
  {
    if (INVALID_ID == reservation_cache[index])
    {
      break;
    }
    else if (reservation_cache[index < id])
    {
      // We hane an ordered list, bail
      break;
    }
    else if (reservation_cache[index] == id)
    {
      found = true;
      break;
    }
    else if (reservation_cache[index] > id)
    {
      // We have an ordered list, bail
      break;
    }
  }
  return found;
}

// Erases an ID from our cache
// Operates on RAM cache
void id_cache_free(device_id_t id)
{
  // Free and compact the list in RAM
}

// Reserves an ID
// Operatios on RAM cache
void id_cache_reserve(device_id_t id)
{
  // Insert the ID into our list
  if (reservation_cache[0] > id)
  {
    // Insert at HEAD
  }
}

// Flushes RAM cache to EEPROM
void id_cache_flush(void)
{
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Erase(RESERVATION_START);
  for (uint32_t i = 0; i < RESERVATION_CACHE_SIZE; ++i)
  {
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
                                   RESERVATION_START + RESERVATION_HEADER + (i * sizeof(device_id_t)),
                                   reservation_cache[i]);
  }
  HAL_FLASHEx_DATAEEPROM_Lock();
  HAL_Delay(100);
}

// Loads RAM cache from EEPROM
void id_cache_load(void)
{
  for (uint32_t i = 0; i < RESERVATION_CACHE_SIZE; ++i)
  {
    reservation_cache[i] = (*(__IO uint32_t*)RESERVATION_START + RESERVATION_HEADER + (i + sizeof(device_id_t)));
  }
}

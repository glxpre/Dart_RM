#include "bsp_flash.h"
#include "main.h"
#include "string.h"

static uint32_t ger_sector(uint32_t address);

/**
  * @brief          erase flash
  * @param[in]      address: flash address
  * @param[in]      len: page num
  * @retval         none
  */
/**
  * @brief          擦除flash
  * @param[in]      address: flash 地址
  * @param[in]      len: 页数量
  * @retval         none
  */
void flash_erase_address(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = ger_sector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = len;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/**
  * @brief          write data to one page of flash
  * @param[in]      start_address: flash address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
/**
  * @brief          往一页flash写数据
  * @param[in]      start_address: flash 地址
  * @param[in]      buf: 数据指针 需注意字节数必须为8的倍数
  * @param[in]      len: 数据长度
  * @retval         success 0, fail -1编程失败 -2地址未对齐 -3数据长度不符合8字节粒度 -4非数据都已写入
  */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len)
{

    uint32_t uw_address = start_address;
    uint32_t end_address = get_next_flash_address(start_address);
    uint32_t data_idx = 0;  // 用索引而非指针偏移，更清晰

    // 检查地址是否8字节对齐（H723要求）
    if ((uw_address % 8) != 0) {
        return -2;  // 地址未对齐
    }

    // 检查总数据量是否为8字节的整数倍（不足需补全，此处简化处理为直接校验）
    if ((len * 4) % 8 != 0) {
        return -3;  // 数据长度不符合8字节粒度
    }

    HAL_FLASH_Unlock();

    while (uw_address <= end_address && data_idx < len)
    {
        // 每次取2个uint32_t（共8字节），通过指针强制转换为64位数据
        uint64_t data_64 = *(uint64_t*)(&buf[data_idx]);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, uw_address, data_64) == HAL_OK)
        {
            uw_address += 8;       // 地址递增8字节（匹配单次写入量）
            data_idx += 2;         // 处理了2个uint32_t元素
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;  // 编程失败
        }
    }

    HAL_FLASH_Lock();

    // 检查是否所有数据都已写入
    return (data_idx == len) ? 0 : -4;

    // static uint32_t uw_address;
    // static uint32_t end_address;
    // static uint32_t *data_buf;
    // static uint32_t data_len;
    //
    // HAL_FLASH_Unlock();
    //
    // uw_address = start_address;
    // end_address = get_next_flash_address(start_address);
    // data_buf = buf;
    // data_len = 0;
    //
    // while (uw_address <= end_address)
    // {
    //
    //     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,uw_address, *data_buf) == HAL_OK)//h723仅支持单次写入一个flash字积即8字节
    //     {
    //         uw_address += 4;
    //         data_buf++;
    //         data_len++;
    //         if (data_len == len)
    //         {
    //             break;
    //         }
    //     }
    //     else
    //     {
    //         HAL_FLASH_Lock();
    //         return -1;
    //     }
    // }
    //
    // HAL_FLASH_Lock();
    // return 0;
}


/**
 * @brief 多地址范围 Flash 写入（按 8 字节粒度）
 * @param start_address 起始地址（必须 8 字节对齐）
 * @param end_address   结束地址（包含）
 * @param buf           待写入数据（uint32_t 数组，总字节数需为 8 的倍数）
 * @param len           buf 中 uint32_t 元素的总个数（需满足 (len * 4) % 8 == 0）
 * @return 0: 成功; 非0: 错误码（-1:编程失败; -2:地址未对齐; -3:数据长度非法; -4:地址越界未写完）
 */
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len)
{
    uint32_t uw_address = start_address;
    uint32_t data_idx = 0;  // 跟踪已处理的 uint32_t 元素索引
    uint64_t *data_64;      // 指向 8 字节（双字）数据的指针

    /* 1. 地址对齐校验 */
    if ((uw_address % 8) != 0) {
        return -2;  // 起始地址必须 8 字节对齐
    }

    /* 2. 数据长度校验：总字节数（len*4）需为 8 的倍数 */
    if ((len * 4) % 8 != 0) {
        return -3;  // 数据长度不满足 8 字节编程粒度要求
    }

    HAL_FLASH_Unlock();  // 解锁 Flash

    while (uw_address <= end_address && data_idx < len)
    {
        /* 3. 合并两个 uint32_t 为 1 个 uint64_t（8 字节） */
        data_64 = (uint64_t*)(&buf[data_idx]);

        /* 4. 执行 8 字节编程 */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, uw_address, *data_64) == HAL_OK)
        {
            uw_address += 8;       // 地址递增 8 字节（匹配单次写入量）
            data_idx += 2;         // 已处理 2 个 uint32_t 元素（共 8 字节）
        }
        else
        {
            HAL_FLASH_Lock();      // 编程失败时锁 Flash
            return -1;             // 编程失败
        }
    }

    HAL_FLASH_Lock();  // 写入完成后锁 Flash

    /* 5. 校验是否所有数据都已写入 */
    return (data_idx == len) ? 0 : -4;  // -4: 地址越界导致数据未写完
}


// /**
//   * @brief          write data to some pages of flash
//   * @param[in]      start_address: flash start address
//   * @param[in]      end_address: flash end address
//   * @param[in]      buf: data point
//   * @param[in]      len: data num
//   * @retval         success 0, fail -1
//   */
// /**
//   * @brief          往几页flash写数据
//   * @param[in]      start_address: flash 开始地址
//   * @param[in]      end_address: flash 结束地址
//   * @param[in]      buf: 数据指针
//   * @param[in]      len: 数据长度
//   * @retval         success 0, fail -1
//   */
// int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len)
// {
//
//      uint32_t uw_address = 0;
//      uint32_t *data_buf;
//      uint32_t data_len;
//
//      HAL_FLASH_Unlock();
//
//      uw_address = start_address;
//      data_buf = buf;
//      data_len = 0;
//      while (uw_address <= end_address)
//      {
//          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,uw_address, *data_buf) == HAL_OK)
//          {
//              uw_address += 4;
//              data_buf++;
//              data_len++;
//              if (data_len == len)
//              {
//                  break;
//              }
//          }
//          else
//          {
//              HAL_FLASH_Lock();
//              return -1;
//          }
//      }
//
//      HAL_FLASH_Lock();
//      return 0;
// }


/**
  * @brief          read data for flash
  * @param[in]      address: flash address
  * @param[out]     buf: data point
  * @param[in]      len: data num
  * @retval         none
  */
/**
  * @brief          从flash读数据
  * @param[in]      start_address: flash 地址
  * @param[out]     buf: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void*)address, len *4);
}


/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
/**
  * @brief          获取flash的sector号
  * @param[in]      address: flash 地址
  * @retval         sector号
  */
static uint32_t ger_sector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    // else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    // {
    //     sector = FLASH_SECTOR_8;
    // }
    // else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    // {
    //     sector = FLASH_SECTOR_9;
    // }
    // else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    // {
    //     sector = FLASH_SECTOR_10;
    // }
    // else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
    // {
    //     sector = FLASH_SECTOR_11;
    // }
    // else
    // {
    //     sector = FLASH_SECTOR_11;
    // }

    return sector;
}

/**
  * @brief          get the next page flash address
  * @param[in]      address: flash address
  * @retval         next page flash address
  */
/**
  * @brief          获取下一页flash地址
  * @param[in]      address: flash 地址
  * @retval         下一页flash地址
  */
uint32_t get_next_flash_address(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = ADDR_FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = ADDR_FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = ADDR_FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = ADDR_FLASH_SECTOR_11;
    }
    else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}

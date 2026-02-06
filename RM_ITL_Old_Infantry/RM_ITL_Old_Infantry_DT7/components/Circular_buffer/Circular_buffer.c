#include "Circular_buffer.h"
#include "CRC.h"

//环形缓存区算法


// 指令的最小长度
#define COMMAND_MIN_LENGTH 9

// 循环缓冲区大小
#define BUFFER_SIZE 512
// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
uint16_t readIndex = 0;
// 循环缓冲区写索引
uint16_t writeIndex = 0;

/**
* @brief 增加读索引
* @param length 要增加的长度
*/
void Command_AddReadIndex(uint16_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param i 要读取的数据索引
*/

uint8_t Command_Read(uint16_t i) {
    uint16_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/**
* @brief 计算未处理的数据长度
* @return 未处理的数据长度
* @retval 0 缓冲区为空
* @retval 1~BUFFER_SIZE-1 未处理的数据长度
* @retval BUFFER_SIZE 缓冲区已满
*/
uint16_t Command_GetLength() {
 // 读索引等于写索引时，缓冲区为空
 if (readIndex == writeIndex) {
   return 0;
 }
 // 如果缓冲区已满,返回BUFFER_SIZE
 if (writeIndex + 1 == readIndex || (writeIndex == BUFFER_SIZE - 1 && readIndex == 0)) {
   return BUFFER_SIZE;
 }
 // 如果缓冲区未满,返回未处理的数据长度
 if (readIndex <= writeIndex) {
   return writeIndex - readIndex;
 } else {
   return BUFFER_SIZE - readIndex + writeIndex;
 }
}

// uint8_t Command_GetLength() {
//     return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
// }


/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余空间
* @retval BUFFER_SIZE 缓冲区为空
*/
uint16_t Command_GetRemain() {
    return BUFFER_SIZE - Command_GetLength();
}

/**
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @return 写入的数据长度
*/
uint8_t Command_Write(uint8_t *data, uint16_t length) {
    // 如果缓冲区不足 则不写入数据 返回0
    if (Command_GetRemain() < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (writeIndex + length < BUFFER_SIZE) {
        memcpy(buffer + writeIndex, data, length);
        writeIndex += length;
    } else {
        uint16_t firstLength = BUFFER_SIZE - writeIndex;
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = length - firstLength;
    }
    return length;
}

/**
 * @brief 从环形缓冲区获取一帧完整的裁判系统数据
 * @param frame 数据帧存放指针
 * @return 获取的数据帧长度
 * @retval 0 没有获取到完整帧
 */
uint16_t Referee_GetFrame(uint8_t *frame) 
{
    uint16_t data_length;
    uint16_t total_frame_length;
    uint16_t i;
    uint8_t temp_buffer[256];  // 临时缓冲区，用于CRC校验
    
    // 寻找完整帧
    while (1) {
        // 如果缓冲区长度小于最小帧长（5字节帧头+2字节cmd_id+2字节帧尾=9字节），则不可能有完整帧
        if (Command_GetLength() < COMMAND_MIN_LENGTH) {
            return 0;
        }
        
        // 如果不是帧头SOF(0xA5)，则跳过重新寻找
        if (Command_Read(readIndex) != 0xA5) {
            Command_AddReadIndex(1);
            continue;
        }
        
        // 读取data_length（小端字节序）
        data_length = Command_Read(readIndex + 1) | (Command_Read(readIndex + 2) << 8);
        
        // 计算完整帧长度：帧头(5) + cmd_id(2) + data + 帧尾(2)
        total_frame_length = 5 + 2 + data_length + 2;
        
        // 检查帧长度是否合理（防止数据错误导致计算值过大）
        if (total_frame_length > 256) {
            Command_AddReadIndex(1);
            continue;
        }
        
        // 如果缓冲区长度小于完整帧长度，则不可能有完整帧
        if (Command_GetLength() < total_frame_length) {
            return 0;
        }
        
        // 复制帧头前5字节到临时缓冲区用于CRC8校验
        for (i = 0; i < 5; i++) {
            temp_buffer[i] = Command_Read(readIndex + i);
        }
        
        // 验证帧头CRC8（校验前4字节：SOF + data_length + seq）
        if (!verify_CRC8_check_sum(temp_buffer, 5)) {
            Command_AddReadIndex(1);
            continue;
        }
        
        // 复制完整帧到临时缓冲区用于CRC16校验
        for (i = 0; i < total_frame_length; i++) {
            temp_buffer[i] = Command_Read(readIndex + i);
        }
        
        // 验证整包CRC16（从SOF到data的所有字节，不包含帧尾CRC16本身）
        if (!verify_CRC16_check_sum(temp_buffer, total_frame_length)) {
            Command_AddReadIndex(1);
            continue;
        }
        
        // 如果找到完整帧，则将数据复制到输出缓冲区
        for (i = 0; i < total_frame_length; i++) {
            frame[i] = Command_Read(readIndex + i);
        }
        
        // 更新读索引
        Command_AddReadIndex(total_frame_length);
        
        return total_frame_length;
    }
}

/**
* @brief 尝试获取一条指令
* @param command 指令存放指针
* @return 获取的指令长度
* @retval 0 没有获取到指令
*/
uint8_t Angle_GetCommand(uint8_t *command) 
{
    // 寻找完整指令
    while (1) {
        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (Command_GetLength() < 14) {
        return 0;
        }
        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(readIndex) != 0xFF) {
        Command_AddReadIndex(1);
        continue;
        }
        // 检查包尾是否是0xFE
        if(Command_Read(readIndex + 13) != 0xFE)
        {
            Command_AddReadIndex(1);
            continue;
        }
        // 如果找到完整指令 则将指令写入command 返回指令长度
        for (uint8_t i = 0; i < 14; i++) {
        command[i] = Command_Read(readIndex + i);
        }
        Command_AddReadIndex(14);
        return 14;
    }
}

#include "my_robot_driver/protocol.h"
 uint8_t ProtocolHelper::calculateChecksum(uint8_t *buf, int len)
 {
    uint8_t sum = 0; 
    for (int i = 0; i < len; i++)
    {
        sum += buf[i]; 
    
    } 
    
return sum; 
}
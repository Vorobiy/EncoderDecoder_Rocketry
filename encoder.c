    #ifndef __RRC_ENCODER_H__
    #define __RRC_ENCODER_H__

    ////    Includes   ////
    #include <stdint.h>


    ////    Defines    ////
    // Version
    #define RRC_ENCODER_VERSION 3

    // Headers
    #define RRC_HEAD_GPS_LONG   0x00        //  gps longitude   (000)
    #define RRC_HEAD_GPS_LAT    0x01        //  gps latitude    (001)
    #define RRC_HEAD_ACC_X      0x02        //  accelerometer x (010)
    #define RRC_HEAD_ACC_Y      0x03        //  accelerometer y (011)
    #define RRC_HEAD_ACC_Z      0x04        //  accelerometer z (100)
    #define RRC_HEAD_PRESS      0x05        //  baromete        (101)
    #define RRC_HEAD_TEMP       0x06        //  temperature     (110)
    //#define RRC_HEAD_BATT_V   0x07
    #define RRC_HEAD_END        0x07        //  end             (111)

    // Bit Shift
    #define RRC_SHIFT_CHECKS    0           //  checksum shift  (000x xxxx) << 0  => (000x xxxx)
    #define RRC_SHIFT_HEADER    5           //  header shift    (0000 0xxx) << 3  => (xxx0 0000) 
    #define RRC_SHIFT_DATA      19          //  header shift    (000x xxxx) << 19 => (xxxx x000 0000 0000 0000 0000) 

    // Bit Masks
    #define RRC_MASK_CHECKS     0x1f        //  0001 1111
    #define RRC_MASK_DATA       0x1f        //  0001 1111
    #define RRC_MASK_TIME       0x1f        //  0001 1111
    #define RRC_MASK_HEADER     0x07        //  0000 0111

    // Package size
    #define RRC_DATAPACK_SIZE   10          //  each package is 10 bytes

    ////    Functions Defenitions    ////

    /**
     * @brief Generate the checksum for a 3 byte integer
     * 
     * @param data is the 3 byte integer
     * @return An 8 bit integer with the calulated checksum, use only the lowest 5 bits
     */
    uint8_t _checksum(uint32_t data)
    {
        uint8_t result = ( 0x01 &   (data & 0x01   )         )   //  LSB of the lowest byte (0x01    = 0000 0000 0000 0000 0000 0001)
                    | ( 0x02 & ( (data & 0x80   ) >> 6  ) )   //  MSB of the lowest byte (0x80    = 0000 0000 0000 0000 1000 00x0)
                    | ( 0x04 & ( (data & 0x100  ) >> 6  ) )   //  LSB of the middle byte (0x100   = 0000 0000 0000 0001 0000 0x00)
                    | ( 0x08 & ( (data & 0x8000 ) >> 12 ) )   //  MSB of the middle byte (0x8000  = 0000 0000 1000 0000 0000 x000)
                    | ( 0x10 & ( (data & 0x10000) >> 12 ) );  //  LSB of the higest byte (0x10000 = 0000 0001 0000 0000 000x 0000)

        return result;
    }


    /**
     * @brief Convert double precision float data to a 3 byte int with highest bit as sign
     * 
     * @param data the float data to be converted
     * @param range the number of decimal points to be kept in the resulting integer
     * @return A 32 bit integer with only top 24 bits used. Bit 24 is flipped to 1 if data is negative
     */
    uint32_t _float2int(double data, uint8_t header)
    {
        uint32_t int_data, neg   = 0;

        if(data < 0)
        {
            data *= -1;                //  We don't need 2's complement
            neg   = 1;
        }

        switch(header)
        {
            case RRC_HEAD_GPS_LONG:
            case RRC_HEAD_GPS_LAT:
                int_data = data * 10000 + 0.5;
                break;
            default:
                int_data = data * 100 + 0.5;
                break;
        }

        int_data |= neg << 23;         //  flip higest bit if negative

        return int_data;
    }


    /**
     * @brief Encodes a double precision float data into 8 bytes with headers, checksum and timestamp
     * 
     * @param data is the float data to be encoded
     * @param header is the header (type) of the data being encoded
     * @param time is the time stamp of the time the data was encoded
     * @param out is a pointer to the list for the resulting list
     * @return Always zero
     */
    uint8_t encode(double data, uint8_t header, uint32_t time, uint8_t *out)
    {
        uint32_t int_data = _float2int(data, header);
        uint8_t  checks   = _checksum(int_data);

        uint8_t header_value = (header & RRC_MASK_HEADER) << RRC_SHIFT_HEADER; //*Implemented header_value equation outside the for loop
        header_value &= RRC_MASK_HEADER << RRC_SHIFT_HEADER;

        //  set the headers for all the packets
        for(uint8_t i = 0; i < RRC_DATAPACK_SIZE; i++)
            out[i] = header_value;

        //  store the data in packets  to 5
        out[1] |= (int_data & 0xf80000) >> 19;  // (0xf80000 = 1111 1000 0000 0000 0000 0000)
        out[2] |= (int_data & 0x07c000) >> 14;  // (0x07c000 = 0000 0111 1100 0000 0000 0000)
        out[3] |= (int_data & 0x3e00  ) >> 9 ;  // (0x3e00   = 0000 0000 0011 1110 0000 0000)
        out[4] |= (int_data & 0x01f0  ) >> 4 ;  // (0x01f0   = 0000 0000 0000 0001 1111 0000)
        out[5] |= (int_data & 0x0f    )      ;  // (0x1f     = 0000 0000 0000 0000 0000 1111)

        // store timestamp in packets 6 to 9
        out[6] |= (time & 0x0f8000) >> 15;      // (0x0f8000 = 1111 1000 0000 0000 0000)
        out[7] |= (time & 0x7c00  ) >> 10;      // (0x7c00   = 0000 0111 1100 0000 0000)
        out[8] |= (time & 0x03e0  ) >> 5 ;      // (0x03e0   = 0000 0000 0011 1110 0000)
        out[9] |= (time & 0x1f    )      ;      // (0x1f     = 0000 0000 0000 0001 1111)

        return 0;
    }

    #endif

// I changed:
// 1. Removed redundant function declerations
// 2. Implemented header_value equation outside the for loop (line 105)
// 3. Removed redundant equation on line 16 and used header_value variable to recreate it
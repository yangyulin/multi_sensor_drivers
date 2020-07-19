#ifndef __microstrain_comm_util_
#define __microstrain_comm_util_

typedef unsigned char Byte;

bool systemLittleEndianCheck()
{
    short int word = 0x0001;
    char* bytes = (char*)&word;
    if (bytes[0] == 0)
        return false;
    else
        return true;
}

unsigned int make32UnsignedInt(Byte* pBytes, bool little_endian)
{
    unsigned int i;

    if (little_endian) {
        ((Byte*)(&i))[0] = pBytes[3];
        ((Byte*)(&i))[1] = pBytes[2];
        ((Byte*)(&i))[2] = pBytes[1];
        ((Byte*)(&i))[3] = pBytes[0];
    } else {
        ((Byte*)(&i))[0] = pBytes[0];
        ((Byte*)(&i))[1] = pBytes[1];
        ((Byte*)(&i))[2] = pBytes[2];
        ((Byte*)(&i))[3] = pBytes[3];
    }

    return i;
}

unsigned int make16UnsignedInt(const Byte* pBytes, bool little_endian)
{
    unsigned int i;

    if (little_endian) {
        ((Byte*)(&i))[0] = pBytes[1];
        ((Byte*)(&i))[1] = pBytes[0];

    } else {
        ((Byte*)(&i))[0] = pBytes[0];
        ((Byte*)(&i))[1] = pBytes[1];
    }

    return i;
}

void makeUnsignedInt16(unsigned int val, Byte* high, Byte* low)
{
    *low = static_cast<Byte>(val);
    *high = static_cast<Byte>(val >> 8);
}

void makeUnsignedInt32(unsigned int val, Byte* byte3, Byte* byte2, Byte* byte1, Byte* byte0)
{
    *byte0 = static_cast<Byte>(val);
    *byte1 = static_cast<Byte>(val >> 8);
    *byte2 = static_cast<Byte>(val >> 16);
    *byte3 = static_cast<Byte>(val >> 24);
}

float make32bitFloat(const Byte* pBytes, bool little_endian)
{
    float f = 0;

    if (little_endian) {
        ((Byte*)(&f))[0] = pBytes[3];
        ((Byte*)(&f))[1] = pBytes[2];
        ((Byte*)(&f))[2] = pBytes[1];
        ((Byte*)(&f))[3] = pBytes[0];
    } else {
        ((Byte*)(&f))[0] = pBytes[0];
        ((Byte*)(&f))[1] = pBytes[1];
        ((Byte*)(&f))[2] = pBytes[2];
        ((Byte*)(&f))[3] = pBytes[3];
    }

    return f;
}

void unpack32BitFloats(float* dest, const Byte* source_bytes, int length, bool little_endian)
{
    int ii;
    int byte_ind = 0;
    for (ii = 0; ii < length; ii++) {
        dest[ii] = make32bitFloat(&source_bytes[byte_ind], little_endian);
        byte_ind += 4;
    }
}

void convertFloatToDouble(double* dest, const float* source, int length)
{
    for (int ii = 0; ii < length; ii++) {
        dest[ii] = (double)source[ii];
    }
}

/**
 * Converts a raw, pitch, and yaw messurements into a quaturian
 * Orginally part of the libbot library, but converted to work with floats
 */
void roll_pitch_yaw_to_quat(const float rpy[3], float q[4])
{
    float roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

    float halfroll = roll / 2;
    float halfpitch = pitch / 2;
    float halfyaw = yaw / 2;

    float sin_r2 = sin(halfroll);
    float sin_p2 = sin(halfpitch);
    float sin_y2 = sin(halfyaw);

    float cos_r2 = cos(halfroll);
    float cos_p2 = cos(halfpitch);
    float cos_y2 = cos(halfyaw);

    q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
    q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
    q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
    q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
}

/*
 * Prints byte arrays in hex
 */
void print_array_char_hex(const unsigned char* array, int length)
{
    int ii;
    for (ii = 0; ii < length; ii++) {
        fprintf(stderr, "%02X ", (unsigned char)array[ii]);
    }
    fprintf(stderr, "\n");
}

#endif
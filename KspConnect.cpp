#include <KspConnect.h>
#include "Arduino.h"

HandShakePacket HPacket;
VesselData VData;
ControlPacket CPacket;

uint8_t rx_len;
uint16_t *address;
byte buffer[256]; //address for temporary storage and parsing buffer
uint8_t structSize;
uint8_t rx_array_inx; //index for RX parsing buffer
uint8_t calc_CS;      //calculated checksum
byte id;

unsigned long deadtime, deadtimeOld, controlTime, controlTimeOld;
unsigned long now;

boolean Connected = false;

boolean GetConnect()
{
    return Connected;
}

void KSPDataHandler(void (*function)())
{
    int returnValue = -1;
    now = millis();

    if (KSPBoardReceiveData())
    {
        deadtimeOld = now;
        returnValue = id;
        switch (id)
        {
        case 0: //Handshake packet
            Handshake();
            break;
        case 1:
            (*function)();
            break;
        }

        //We got some data, turn the green led on
        Connected = true;
    }
    else
    { //if no message received for a while, go idle
        deadtime = now - deadtimeOld;
        if (deadtime > IDLETIMER)
        {
            deadtimeOld = now;
            Connected = false;
        }
    }

    return returnValue;
}

void SetControlPacketValues(int throttle, int pitch, int yaw)
{
    CPacket.Pitch = pitch;
    CPacket.Throttle = throttle;
    CPacket.Yaw = yaw;
}

void KSPSendControlPacket()
{
    KSPBoardSendData(details(CPacket));
}

void KspConnectInitialize()
{
    HPacket.id = 0;
    CPacket.id = 101;
}

byte getSASMode()
{
    return VData.NavballSASMode & B00001111; // leaves alone the lower 4 bits of; all higher bits set to 0.
}

void setSASMode(byte m)
{
    CPacket.NavballSASMode &= B11110000;
    CPacket.NavballSASMode += m;
}

byte getNavballMode()
{
    return VData.NavballSASMode >> 4; // leaves alone the higher 4 bits of; all lower bits set to 0.
}

void setNavballMode(byte m)
{
    CPacket.NavballSASMode &= B00001111;
    CPacket.NavballSASMode += m << 4;
}

void MainControls(byte n, boolean s)
{
    if (s)
        CPacket.MainControls |= (1 << n); // forces nth bit of x to be 1.  all other bits left alone.
    else
        CPacket.MainControls &= ~(1 << n); // forces nth bit of x to be 0.  all other bits left alone.
}

void ControlGroups(byte n, boolean s)
{
    if (s)
        CPacket.ControlGroup |= (1 << n); // forces nth bit of x to be 1.  all other bits left alone.
    else
        CPacket.ControlGroup &= ~(1 << n); // forces nth bit of x to be 0.  all other bits left alone.
}

byte ControlStatus(byte n)
{
    return ((VData.ActionGroups >> n) & 1) == 1;
}

boolean KSPBoardReceiveData()
{
    if ((rx_len == 0) && (Serial.available() > 3))
    {
        //Read the first header.
        while (Serial.read() != 0xBE)
        {
            if (Serial.available() == 0)
                return false;
        }

        //Read the second header.
        if (Serial.read() == 0xEF)
        {
            rx_len = Serial.read();
            id = Serial.read();
            rx_array_inx = 1;

            // If id == 0 then it Handshake()
            // if id == 1 then define structure size and create pointer.
            switch (id)
            {
            case 0:
                structSize = sizeof(HPacket);
                address = (uint16_t *)&HPacket;
                break;
            case 1:
                structSize = sizeof(VData);
                address = (uint16_t *)&VData;
                break;
            }
        }

        //Make sure the binary structs on both sides are the same size.
        if (rx_len != structSize)
        {
            rx_len = 0;
            return false;
        }
    }

    if (rx_len != 0)
    {
        while (Serial.available() && rx_array_inx <= rx_len)
        {
            buffer[rx_array_inx++] = Serial.read();
        }
        buffer[0] = id;

        if (rx_len == (rx_array_inx - 1))
        {
            //Seem to have got whole message
            //Last uint8_t is CS
            calc_CS = rx_len;
            for (int i = 0; i < rx_len; i++)
            {
                calc_CS ^= buffer[i];
            }

            if (calc_CS == buffer[rx_array_inx - 1])
            { 
                //Checksum passed. Copy data to structure.
                memcpy(address, buffer, structSize);
                rx_len = 0;
                rx_array_inx = 1;
                return true;
            }
            else
            {
                //Failed checksum, need to clear this out anyway.
                rx_len = 0;
                rx_array_inx = 1;
                return false;
            }
        }
    }

    return false;
}

void KSPBoardSendData(uint8_t *address, uint8_t len)
{
    uint8_t CS = len;
    Serial.write(0xBE);
    Serial.write(0xEF);
    Serial.write(len);

    for (int i = 0; i < len; i++)
    {
        CS ^= *(address + i);
        Serial.write(*(address + i));
    }

    Serial.write(CS);
}

void Handshake()
{
    HPacket.id = 0;
    HPacket.M1 = 3;
    HPacket.M2 = 1;
    HPacket.M3 = 4;

    KSPBoardSendData(details(HPacket));
}
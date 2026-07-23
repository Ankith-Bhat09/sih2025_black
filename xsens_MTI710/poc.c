#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

//Big Endian Conversion Helpers

float be_float(unsigned char *b)
{
    union
    {
        unsigned int i;
        float f;
    } value;

    value.i =
          ((unsigned int)b[0] << 24)
        | ((unsigned int)b[1] << 16)
        | ((unsigned int)b[2] << 8)
        |  (unsigned int)b[3];

    return value.f;
}

unsigned short be_u16(unsigned char *b)
{
    return ((unsigned short)b[0] << 8) | b[1];
}



//    Verify Checksum

int verify_checksum(unsigned char *packet, int length)
{
    unsigned char sum = 0;
    int i;

    for(i = 1; i < length; i++)
        sum += packet[i];

    return (sum == 0);
}


//    Parse MTData2 Payload

void parse_mtdata2(unsigned char *payload, int length)
{
    int index = 0;

    while(index + 3 <= length)
    {
        unsigned short data_id;
        unsigned char data_len;
        unsigned char *data;

        data_id = be_u16(payload + index);
        data_len = payload[index + 2];

        if(index + 3 + data_len > length)
            break;

        data = payload + index + 3;

        switch(data_id)
        {
            case 0x2010:      /* Quaternion */
            {
                float q0 = be_float(data);
                float q1 = be_float(data + 4);
                float q2 = be_float(data + 8);
                float q3 = be_float(data + 12);

                printf("Quaternion : %.3f %.3f %.3f %.3f\n",
                        q0,q1,q2,q3);
                break;
            }

            case 0x4020:      /* Accelerometer */
            {
                float ax = be_float(data);
                float ay = be_float(data + 4);
                float az = be_float(data + 8);

                printf("Acceleration : %.3f %.3f %.3f\n",
                        ax,ay,az);
                break;
            }

            case 0x8020:      /* Gyroscope */
            {
                float gx = be_float(data);
                float gy = be_float(data + 4);
                float gz = be_float(data + 8);

                printf("Gyroscope : %.3f %.3f %.3f\n",
                        gx,gy,gz);
                break;
            }

            case 0x1020:      /* Packet Counter */
            {
                unsigned short count = be_u16(data);

                printf("Packet Counter : %u\n",count);
                break;
            }

            default:
                break;
        }

        index += 3 + data_len;
    }

    printf("-------------------------------------\n");
}


//    Parse Complete Packet

void parse_packet(unsigned char *packet, int packet_size)
{
    unsigned char mid;
    unsigned char len;
    unsigned char *payload;

    if(packet[0] != 0xFA)
        return;

    if(packet[1] != 0xFF)
        return;

    if(!verify_checksum(packet, packet_size))
        return;

    mid = packet[2];
    len = packet[3];

    if(mid != 0x36)          /* MTData2 */
        return;

    payload = packet + 4;

    parse_mtdata2(payload, len);
}

// Send Xsens Message

int send_x(libusb_device_handle *dev,
           unsigned char mid,
           unsigned char *payload,
           unsigned char len)
{
    unsigned char packet[256];
    unsigned char checksum = 0;
    int transferred;
    int i;

    packet[0] = 0xFA;
    packet[1] = 0xFF;
    packet[2] = mid;
    packet[3] = len;

    for(i = 0; i < len; i++)
        packet[4+i] = payload[i];

    for(i = 1; i < 4 + len; i++)
        checksum += packet[i];

    packet[4+len] = (unsigned char)(256 - checksum);

    return libusb_bulk_transfer(
            dev,
            0x02,
            packet,
            5 + len,
            &transferred,
            500);
}


//    Configure Sensor

void xsens_configure(libusb_device_handle *dev)
{
    unsigned char config[16];
    int index = 0;

    /* Packet Counter */
    config[index++] = 0x10;
    config[index++] = 0x20;
    config[index++] = 0xFF;
    config[index++] = 0xFF;

    /* Quaternion @100Hz */
    config[index++] = 0x20;
    config[index++] = 0x10;
    config[index++] = 0x00;
    config[index++] = 100;

    /* Acceleration @100Hz */
    config[index++] = 0x40;
    config[index++] = 0x20;
    config[index++] = 0x00;
    config[index++] = 100;

    /* Gyroscope @100Hz */
    config[index++] = 0x80;
    config[index++] = 0x20;
    config[index++] = 0x00;
    config[index++] = 100;

    send_x(dev,0x30,NULL,0);          /* Go to Config */

    send_x(dev,0xC0,config,index);    /* Set Output */

    send_x(dev,0x10,NULL,0);          /* Go to Measurement */
}



int main(void)
{
    libusb_context *ctx;
    libusb_device_handle *dev;

    unsigned char buffer[4096];

    int transferred;
    int status;

    libusb_init(&ctx);

    dev = libusb_open_device_with_vid_pid(
                ctx,
                0x2639,
                0x0017);

    if(dev == NULL)
    {
        printf("Unable to open device\n");
        return 1;
    }

    libusb_claim_interface(dev,0);
    libusb_claim_interface(dev,1);

    xsens_configure(dev);

    printf("Streaming...\n");

    while(1)
    {
        status = libusb_bulk_transfer(
                    dev,
                    0x83,
                    buffer,
                    sizeof(buffer),
                    &transferred,
                    100);

        if(status == 0)
        {
            parse_packet(buffer,transferred);
        }
    }

    libusb_release_interface(dev,0);
    libusb_release_interface(dev,1);

    libusb_close(dev);
    libusb_exit(ctx);

    return 0;
}

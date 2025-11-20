#include "packets.h"

#include <stdio.h>
#include <string.h>

    void print_packet_details(const uint8_t* frame, uint32_t frame_len);
    void print_binary_uint8(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        putchar((value & (1 << i)) ? '1' : '0');
    }
}

    const uint8_t version = 1;



    int main(int argc, char *argv[]) {
        uint8_t data[MAX_DATA_SIZE];
        memcpy(&data[0], (uint8_t[]){'A', 'B', 'C'}, 3);
        uint32_t data_size = 3;
        
        pkt_data_type_t type = PKT_DATA_IMAGE;
        uint8_t flags = 0;
        uint16_t sequence_num = 1;
        uint8_t packet[MAX_SIZE];
        uint32_t packet_size = pkt_build(packet, MAX_SIZE, type, flags, sequence_num, data, data_size);
            printf("Packet size: %d", packet_size);
        print_packet_details(packet, MAX_SIZE);
        return 0;
    }

    void print_packet_details(const uint8_t* frame, uint32_t frame_len){
            pkt_header_t header;
            const uint8_t *outdata;
            uint32_t packet_parse = pkt_parse(frame, frame_len, &header, &outdata);
            printf("Packet version: %d \t", header.version);
            print_binary_uint8(header.version);
            printf("\n");
            printf("Packet type: %d \t", header.data_type);
            print_binary_uint8(header.data_type);
            printf("\n");
            printf("Packet flags: %d \t", header.flags);
            print_binary_uint8(header.flags);
            printf("\n");

            printf("Packet sequence number: %d \t", header.seq);
            print_binary_uint8((uint8_t)(header.seq >> 8));
            printf(" ");
            print_binary_uint8(header.seq);
            printf("\n");

            printf("Packet length: %d \t", header.len);
            print_binary_uint8((uint8_t)(header.len >> 8));
            printf(" ");
            print_binary_uint8(header.len);
            printf("\n");
            printf("Data stream (binary): \n");
            for (uint32_t i = 0; i < header.len; ++i){
                print_binary_uint8(outdata[i]);
                printf(" ");
            }
            printf("\nData stream (hex): \n");
            for (uint32_t i = 0; i < header.len; ++i){
                printf("%X ", outdata[i]);

            }
            printf("\nFull packet stream (hex): \n");
            for (uint32_t i = 0; i < header.len + OVERHEAD; ++i){
                printf("%X ", frame[i]);
            }
            printf("\nFull outdata stream (hex): \n");
            for (uint32_t i = 0; i < header.len; ++i){
                printf("%X ", outdata[i]);
            }
        }
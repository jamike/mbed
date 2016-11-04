// Playback example with the USBAUDIO library

#include "mbed.h"
#include "USBAudio.h"

// frequency: 48 kHz
#define FREQ_SPK 48000
#define FREQ_MIC 48000

// 2channels: stereo
#define NB_CHA_SPK 2
#define NB_CHA_MIC 2

// length computed: each ms, we receive 48 * 16bits ->48 * 2 bytes. as there are two channels, the length will be 48 * 2 * 2
#define LENGTH_AUDIO_PACKET_SPK (FREQ_SPK / 500) * NB_CHA_SPK
#define LENGTH_AUDIO_PACKET_MIC (FREQ_MIC / 500) * NB_CHA_MIC

// USBAudio object
USBAudio audio(FREQ_SPK, NB_CHA_SPK, FREQ_MIC, NB_CHA_MIC, 0xab45, 0x0378);
int filled;
int buf[2][LENGTH_AUDIO_PACKET_SPK/sizeof(int)];
void tx_audio(void)
{
    audio.writeSync((uint8_t *)buf[filled]);
}
void rx_audio(void)
{
    if (filled) {
        audio.readSync((uint8_t *)buf[0]);
        filled =0;
    }
    else { audio.readSync((uint8_t *)buf[1]);
        filled =1;
    }
}

int main() {
filled = 0;
memset(&buf[0][0], 0, sizeof (buf));
audio.attachTx(tx_audio);
audio.attachRx(rx_audio);
/*  start the tx with a silent packet */
audio.write((uint8_t *)buf[0]);
while(1);
}



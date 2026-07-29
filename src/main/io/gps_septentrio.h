/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Default COM port used
extern const char septentrioDefaultPort[];

// Septentrio Binary Format (SBF) parser constants
#define	SBF_SYNC1                    '$' // 0x24
#define	SBF_SYNC2                    '@' // 0x40
#define	SBF_HEADER_SIZE              14

// The largest SBF block we parse is ChannelStatus (4013).
// It consists of a 20-byte base overhead (14-byte global frame header + 6-byte block header),
// followed by N satellite channels. Each satellite channel includes a 12-byte ChannelSatInfo sub-block.
// Inside each satellite channel sub-block, there are N2 tracking states (one per antenna). 
// We assume a worst-case of 2 active antennas per satellite (N2 = 2), 
// meaning each antenna adds an 8-byte ChannelStateInfo block.
#define SBF_MAX_ANTENNAS_PER_CHANNEL 2 

// As Septentrio receivers can track more than the 32-satellite limit of the UBX protocol,
// the maximum number of supported satellites is set to 50. 
// The frame buffer is therefore sized for a 50-satellite ChannelStatus block with two antennas per channel.
// The GPS_svinfo array has been updated accordingly, 
// and the Configurator now displays up to 50 satellites in the GPS tab.
// In many cases, more than 50 satellites are tracked. 
// To stay within this limit, only satellites contributing to the current GPS solution are retained, 
// while channels with an idle or not applicable tracking status are ignored.
#define SBF_MAX_FRAME_SIZE           (SBF_HEADER_SIZE + 6 + (12 + (8 * SBF_MAX_ANTENNAS_PER_CHANNEL)) * GPS_SV_MAXSATS) 

// We also use a sanity check to reject any frame larger than a 100-satellite ChannelStatus block
// with 2 antennas per channel, as this can be considered unreasonable and likely indicates a corrupted frame.
#define SBF_MAX_FRAME_SANITY_SIZE    (SBF_HEADER_SIZE + 6 + (12 + (8 * SBF_MAX_ANTENNAS_PER_CHANNEL)) * 2 * GPS_SV_MAXSATS) 

// SBF block IDs
#define SBF_BLOCK_DOP            4001
#define SBF_BLOCK_PVTGEODETIC    4007
#define SBF_BLOCK_VELCOVGEODETIC 5908
#define SBF_BLOCK_ENDOFPVT       5921
#define SBF_BLOCK_CHANNELSTATUS  4013

// GNSS Constellation IDs (IMES not provided in SBF)
#define SEPTENTRIO_GNSS_GPS     0
#define SEPTENTRIO_GNSS_SBAS    1
#define SEPTENTRIO_GNSS_GALILEO 2
#define SEPTENTRIO_GNSS_BEIDOU  3
#define SEPTENTRIO_GNSS_QZSS    5
#define SEPTENTRIO_GNSS_GLONASS 6
#define SEPTENTRIO_GNSS_NAVIC   7
#define SEPTENTRIO_GNSS_UNKNOWN 255

typedef struct __attribute__((packed)) {
	uint8_t sync1;
	uint8_t sync2;
	uint16_t crc;
	uint16_t id_word; // 13 bits of block ID, 3 bits of version
	uint16_t length;
	// Receiver time stamp
	uint32_t tow; // Time of Week (ms)
	uint16_t wnc; // Week Number Count (mod 1024)
} sbfHeader_t;

typedef struct __attribute__((packed)) {
	uint8_t nr_sv;
	uint8_t reserved;
	uint16_t p_dop;
	uint16_t t_dop;
	uint16_t h_dop;
	uint16_t v_dop;
	float hpl;
	float vpl;
} sbfDop_t;

typedef struct __attribute__((packed)) {
	uint8_t mode; 
	uint8_t error;
	double latitude; 
	double longitude;
	double height;
	float undulation;
	float vn;
	float ve;
	float vu;
	float cog;
	double rx_clk_bias;
	float rx_clk_drift;
	uint8_t time_system;
	uint8_t datum;
	uint8_t nr_sv;
	uint8_t wa_corr_info;
	uint16_t reference_id;
	uint16_t mean_corr_age;
	uint32_t signal_info;
	uint8_t alert_flag;
	uint8_t nr_bases;
	uint16_t ppp_info;
	uint16_t latency;
	uint16_t h_accuracy;
	uint16_t v_accuracy;
} sbfPvtGeodetic_t;

typedef struct __attribute__((packed)) {
	uint8_t mode;
	uint8_t error;
	float cov_vn_vn;
	float cov_ve_ve;
	float cov_vu_vu;
	float cov_dt_dt;
	float cov_vn_ve;
	float cov_vn_vu;
	float cov_vn_dt;
	float cov_ve_vu;
	float cov_ve_dt;
	float cov_vu_dt;
} sbfVelCovGeodetic_t;

// Space Vehicle (Satellite) Information
typedef struct __attribute__((packed)) {
    uint8_t  n;               // number of ChannelSatInfo sub-blocks
    uint8_t  sb1_length;      // ChannelSatInfo size (excluding nested StateInfo)
    uint8_t  sb2_length;      // ChannelStateInfo size
    uint8_t  reserved[3];
} sbfChannelStatusHeader_t;

typedef struct __attribute__((packed)) {
    uint8_t  svid;            // 0 = use svidFull instead
    uint8_t  freq_nr;         // GLONASS only
    uint16_t svid_full;       // used when svid == 0
    uint16_t azimuth_riseset; // bits 0-8: azimuth, bits 14-15: rise/set
    uint16_t health_status;   // 2-bit health status per signal: 0=health unknown or not applicable, 1=healthy, 3=unhealthy
    int8_t   elevation;       // degrees, -90 to 90
    uint8_t  n2;              // number of ChannelStateInfo sub-blocks following
    uint8_t  rx_channel;
    uint8_t  reserved2;
} sbfChannelSatInfo_t;

typedef struct __attribute__((packed)) {
    uint8_t  antenna;         // 0 = main antenna
    uint8_t  reserved;
    uint16_t tracking_status; // 2-bit fields per signal: 0=idle or not applicable, 1=search, 2=sync, 3=tracking
    uint16_t pvt_status;      // 2-bit fields per signal: 0=not used, 1=wait eph, 2=used, 3=rejected
    uint16_t pvt_info;        // internal, ignore
} sbfChannelStateInfo_t;

typedef struct {
	uint8_t frame[SBF_MAX_FRAME_SIZE];
	uint16_t index;          // current index into frame buffer
	uint16_t expectedLength;
	uint16_t calculatedCrc;  // accumulated CRC of the received frame  
	uint32_t currentTow;
	uint16_t currentWnc;
	uint64_t lastNavEpochMs; // timestamp of the last committed epoch in milliseconds since GPS epoch
	bool synced; 			 // true when the sync sequence has been detected and we are accumulating bytes into the frame buffer
	bool havePvt;
	bool haveDop;
	bool haveVelCov;
	bool haveChannelStatus;
	sbfHeader_t header;
	sbfPvtGeodetic_t pvt;
	sbfDop_t dop;
	sbfVelCovGeodetic_t velCov;
	uint8_t channelStatusPayload[SBF_MAX_FRAME_SIZE - SBF_HEADER_SIZE];
	uint16_t channelStatusPayloadLength; 
} sbfParserState_t; // SBF parser state 

typedef enum {
    SEPTENTRIO_CFG_FORCE_INPUT = 0,
    SEPTENTRIO_CFG_SET_DATAIO,
    SEPTENTRIO_CFG_SET_SBF_OUTPUT,
    SEPTENTRIO_CFG_SET_DYNAMICS,
    SEPTENTRIO_CFG_COMPLETE,
} septentrioConfigStep_e;

bool gpsNewFrameSeptentrio(uint8_t byte);
void gpsSeptentrioReset(void);

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

#include "platform.h"

#ifdef USE_GPS_SEPTENTRIO

#include <math.h>
#include <string.h>
#include <stdio.h> // debugging only

#include "common/maths.h"
#include "io/gps.h"
#include "io/gps_septentrio.h"

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

enum {
	SBF_SYNC1 = '$', // 0x24
	SBF_SYNC2 = '@', // 0x40
	SBF_HEADER_SIZE = 14,
	SBF_MAX_FRAME_SIZE = 300, // based on PX4 autopilot's SBF parser (k_max_message_size), maybe to be increased 
}; // Septentrio Binary Format (SBF) parser constants

enum {
	SBF_BLOCK_DOP = 4001,
	SBF_BLOCK_PVTGEODETIC = 4007,
	SBF_BLOCK_VELCOVGEODETIC = 5908,
	SBF_BLOCK_ENDOFPVT = 5921, // end of transmission of all PVT related blocks belonging to the same epoch
}; // SBF block IDs

typedef struct {
	uint8_t frame[SBF_MAX_FRAME_SIZE];
	uint16_t index; // current index into frame buffer
	uint16_t expectedLength;
	uint32_t currentTow;
	uint16_t currentWnc;
	uint64_t lastNavEpochMs; // timestamp of the last committed epoch in milliseconds since GPS epoch
	bool synced; 			 // true when the sync sequence has been detected and we are accumulating bytes into the frame buffer
	bool havePvt;
	bool haveDop;
	bool haveVelCov;
	sbfHeader_t header;
	sbfPvtGeodetic_t pvt;
	sbfDop_t dop;
	sbfVelCovGeodetic_t velCov;
} sbfParserState_t; // SBF parser state 

static sbfParserState_t sbfState;

static uint16_t sbfBlockId(const sbfHeader_t *header)
{
	return (uint16_t)(header->id_word & 0x1FFF);
}

static uint16_t sbfCrc16(const uint8_t *data, uint16_t length)
{
	uint8_t x;
	uint16_t crc = 0;
	// Calculate CRC16 over the data, starting after the sync bytes 
	while (length--) {
		x = (uint8_t)((crc >> 8) ^ *data++);
		x ^= x >> 4;
		crc = (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ x);
	}

	return crc;
}

static void sbfResetFrame(void)
{
	sbfState.index = 0;
	sbfState.expectedLength = 0;
	sbfState.synced = false;
	memset(sbfState.frame, 0, sizeof(sbfState.frame));
}

static void sbfResetEpoch(void)
{
	sbfState.currentTow = 0;
	sbfState.currentWnc = 0;
	sbfState.havePvt = false;
	sbfState.haveDop = false;
	sbfState.haveVelCov = false;
	memset(&sbfState.pvt, 0, sizeof(sbfState.pvt));
	memset(&sbfState.dop, 0, sizeof(sbfState.dop));
	memset(&sbfState.velCov, 0, sizeof(sbfState.velCov));
}

void gpsSeptentrioReset(void)
{
	memset(&sbfState, 0, sizeof(sbfState));
	sbfResetFrame();
	sbfResetEpoch();
}

static void sbfStartEpochIfNeeded(uint32_t tow)
{
	// If we have a new TOW, reset the epoch state to start accumulating new data for this epoch
	if (sbfState.currentTow != 0 && tow != sbfState.currentTow) {
		fprintf(stderr, "[GPS] TOW Mismatch! Old: %u, New: %u\n", sbfState.currentTow, tow); // debugging only
		sbfState.havePvt = false;
		sbfState.haveDop = false;
		sbfState.haveVelCov = false;
	}
	sbfState.currentTow = tow;
}

static bool sbfCommitEpoch(void)
{
	// Commit the current epoch data to gpsSol if we have a complete PVT epoch
	fprintf(stderr, "[GPS] Commit attempt (flags: PVT=%d, DOP=%d, VelCov=%d)\n", sbfState.havePvt, sbfState.haveDop, sbfState.haveVelCov); // debugging only

	if (!sbfState.havePvt) {
		fprintf(stderr, "[GPS] Commit failed: Missing PVT block data!\n"); // debugging only
		return false;
	}

	const sbfPvtGeodetic_t *pvt = &sbfState.pvt;
	const uint8_t modeType = (uint8_t)(pvt->mode & 0x0F); // 0: no GNSS PVT available, 1: stand-alone PVT, 2: differential PVT, 3: fixed solution...

	gpsSol.time = sbfState.currentTow;
	gpsSol.llh.lat = (int32_t)lround(RADIANS_TO_DEGREES((float)pvt->latitude) * GPS_DEGREES_DIVIDER); // multiply by GPS_DEGREES_DIVIDER to convert from degrees to the internal representation
	gpsSol.llh.lon = (int32_t)lround(RADIANS_TO_DEGREES((float)pvt->longitude) * GPS_DEGREES_DIVIDER);
	gpsSol.llh.altCm = (int32_t)lround((pvt->height - (double)pvt->undulation) * 100.0); // subtract the geoid undulation to get height above mean sea level
	gpsSol.numSat = (pvt->nr_sv == 255U) ? 0U : pvt->nr_sv;

	if (sbfState.haveDop) {
		gpsSol.dop.pdop = sbfState.dop.p_dop;
		gpsSol.dop.hdop = sbfState.dop.h_dop;
		gpsSol.dop.vdop = sbfState.dop.v_dop;
	}

	gpsSol.groundSpeed = (uint16_t)lround(sqrt(sq(pvt->vn) + sq(pvt->ve)) * 100.0); 
	gpsSol.speed3d = (uint16_t)lround(sqrt(sq(pvt->vn) + sq(pvt->ve) + sq(pvt->vu)) * 100.0); 

	// Normalize course over ground to be within [0, 360) degrees
	if (isnan(pvt->cog) || isinf(pvt->cog) || pvt->cog < -1e9f) { // invalid course value, set to 0
		gpsSol.groundCourse = 0;
	} else {
		float courseDeg = fmodf(pvt->cog, 360.0f); 
		if (courseDeg < 0.0f) { // ensure course is non-negative
			courseDeg += 360.0f;
		}
		gpsSol.groundCourse = (uint16_t)lroundf(courseDeg * 10.0f); // convert to degrees * 10 for internal representation
	}

	gpsSol.velned.velN = (int16_t)lroundf(pvt->vn * 100.0f); 
	gpsSol.velned.velE = (int16_t)lroundf(pvt->ve * 100.0f);
	gpsSol.velned.velD = (int16_t)lroundf(-pvt->vu * 100.0f);

	gpsSol.acc.hAcc = (uint32_t)pvt->h_accuracy * 5U;  
	gpsSol.acc.vAcc = (uint32_t)pvt->v_accuracy * 5U;
	gpsSol.acc.sAcc = 0;
	if (sbfState.haveVelCov) {
		// SBF does not provide a direct speed accuracy value, but it can be estimated from the velocity covariance matrix
		// The diagonal elements of the covariance matrix represent the variance of the respective velocity components (vn, ve, vu)
		// The speed accuracy can be approximated as the square root of the maximum variance among these components
		const float maxVariance = fmaxf(fmaxf(sbfState.velCov.cov_vn_vn, sbfState.velCov.cov_ve_ve), sbfState.velCov.cov_vu_vu);
		if (maxVariance > 0.0f) {
			gpsSol.acc.sAcc = (uint32_t)lroundf(sqrtf(maxVariance) * 1000.0f); // the square root of the variance gives the standard deviation (accuracy)
		}
	}
	gpsSol.acc.headAcc = 0; // not provided by SBF with this set of blocks (to be continued)
	
	// Calculate the navigation interval based on the current and last epoch timestamps
	const uint64_t weekDurationMs = 7ULL * 24ULL * 3600ULL * 1000ULL;
	const uint64_t currentEpochMs = ((uint64_t)sbfState.currentWnc * weekDurationMs) + sbfState.currentTow;
	if (sbfState.lastNavEpochMs == 0U) { 
		gpsSol.navIntervalMs = 100; // default to 100 ms for the first epoch
	} else {
		const uint64_t navDeltaMs = currentEpochMs - sbfState.lastNavEpochMs; 
		gpsSol.navIntervalMs = (uint32_t)constrain((uint32_t)navDeltaMs, 50, 2500); // see calculateNavInterval() function 
	}
	sbfState.lastNavEpochMs = currentEpochMs;

	bool hasFix = (modeType != 0U && pvt->error == 0U); // true if a valid GNSS fix is available

	gpsSetFixState(hasFix); 

	// Verify commited data (debugging only)
	fprintf(stderr, "[GPS] Commit successful!\n");
	fprintf(stderr, "[GPS] Lat: %d, Lon: %d, Alt: %d cm, NumSat: %d, GroundSpeed: %d cm/s, GroundCourse: %d deg*10\n", gpsSol.llh.lat, gpsSol.llh.lon, gpsSol.llh.altCm, gpsSol.numSat, gpsSol.groundSpeed, gpsSol.groundCourse); // debugging only
	fprintf(stderr, "[GPS] VelN: %d cm/s, VelE: %d cm/s, VelD: %d cm/s\n", gpsSol.velned.velN, gpsSol.velned.velE, gpsSol.velned.velD); // debugging only
	fprintf(stderr, "[GPS] AccH: %d mm, AccV: %d mm, AccS: %d mm\n", gpsSol.acc.hAcc, gpsSol.acc.vAcc, gpsSol.acc.sAcc); // debugging only
	fprintf(stderr, "[GPS] Nav Interval: %d ms, Fix State: %s\n", gpsSol.navIntervalMs, hasFix ? "FIX" : "NO FIX"); // debugging only
	fprintf(stderr, "[GPS] DOP: PDOP=%d, HDOP=%d, VDOP=%d\n", gpsSol.dop.pdop, gpsSol.dop.hdop, gpsSol.dop.vdop); // debugging only

	return true;
}

static void sbfProcessBlock(void)
{
	const sbfHeader_t *header = &sbfState.header;
	const uint16_t blockId = sbfBlockId(header);
	const uint8_t *payload = &sbfState.frame[SBF_HEADER_SIZE];
	const uint16_t payloadLength = (uint16_t)(sbfState.expectedLength - SBF_HEADER_SIZE);

	if (blockId != SBF_BLOCK_ENDOFPVT) {
        sbfStartEpochIfNeeded(header->tow); 
        sbfState.currentWnc = header->wnc;
    }

	switch (blockId) {
	case SBF_BLOCK_PVTGEODETIC:
		if (payloadLength >= sizeof(sbfPvtGeodetic_t)) {
			fprintf(stderr, "[GPS] Processing PVT Geodetic block\n"); // debugging only
			memcpy(&sbfState.pvt, payload, sizeof(sbfPvtGeodetic_t));
			sbfState.havePvt = true;
		}
		break;

	case SBF_BLOCK_DOP:
		if (payloadLength >= sizeof(sbfDop_t)) {
			fprintf(stderr, "[GPS] Processing DOP block\n"); // debugging only
			memcpy(&sbfState.dop, payload, sizeof(sbfDop_t));
			sbfState.haveDop = true;
		}
		break;

	case SBF_BLOCK_VELCOVGEODETIC:
		if (payloadLength >= sizeof(sbfVelCovGeodetic_t)) {
			fprintf(stderr, "[GPS] Processing Velocity Covariance Geodetic block\n"); // debugging only
			memcpy(&sbfState.velCov, payload, sizeof(sbfVelCovGeodetic_t));
			sbfState.haveVelCov = true;
		}
		break;

	case SBF_BLOCK_ENDOFPVT:
		fprintf(stderr, "[GPS] Processing End of PVT block\n"); // debugging only
		break;

	default:
		fprintf(stderr, "[GPS] Unknown block ID: %d\n", blockId); // debugging only
		break;
	}
}

bool gpsNewFrameSeptentrio(uint8_t data)
{	
	if (!sbfState.synced) {
		if (sbfState.index == 0) {
			if (data != SBF_SYNC1) {
				fprintf(stderr, "[GPS] Not Synced 1\n"); // debugging only
				return false;
			}
			fprintf(stderr, "[GPS] Synced 1\n"); // debugging only
			sbfState.frame[sbfState.index++] = data; // copy the first sync byte into the frame buffer (and increment the index) 
			// However, as the sync is always the same, do we really need to store it in the frame buffer? 
			// (to be discussed)
			return false; // wait for the second sync byte
		}

		if (sbfState.index == 1) { // we have received the first sync byte, now check for the second
			if (data != SBF_SYNC2) {
				fprintf(stderr, "[GPS] Not Synced 2\n"); // debugging only
				sbfResetFrame();
				return false;
			}
			fprintf(stderr, "[GPS] Synced 2\n"); // debugging only
			sbfState.frame[sbfState.index++] = data; 
			sbfState.synced = true; // valid sync sequence received, we are now synced
			return false; // wait for the rest of the frame
		}
	}

	if (sbfState.index >= SBF_MAX_FRAME_SIZE) {
		fprintf(stderr, "[GPS] Frame size exceeded maximum allowed size\n"); // debugging only
		sbfResetFrame();
		return false;
	}

	sbfState.frame[sbfState.index++] = data; // once synced, bytes are accumulated into the frame buffer until the expected length is reached

	if (sbfState.index == SBF_HEADER_SIZE) { // full header needed to determine the expected length of the frame
		memcpy(&sbfState.header, sbfState.frame, sizeof(sbfHeader_t));
		sbfState.expectedLength = sbfState.header.length;

		if (sbfState.expectedLength < SBF_HEADER_SIZE || sbfState.expectedLength > SBF_MAX_FRAME_SIZE) {
			fprintf(stderr, "[GPS] Invalid frame length: %d\n", sbfState.expectedLength); // debugging only
			sbfResetFrame();
			return false;
		}
	}

	if (sbfState.expectedLength != 0 && sbfState.index >= sbfState.expectedLength) {
		memcpy(&sbfState.header, sbfState.frame, sizeof(sbfHeader_t));
		// Validate the CRC of the received frame
		if (sbfCrc16(&sbfState.frame[4], (uint16_t)(sbfState.expectedLength - 4)) == sbfState.header.crc) {
			const uint16_t blockId = sbfBlockId(&sbfState.header);
			sbfProcessBlock();
			if (blockId == SBF_BLOCK_ENDOFPVT) { // the end of a PVT epoch has been reached, commit the epoch and reset for the next one
				fprintf(stderr, "[GPS] End of PVT block received, committing epoch\n"); // debugging only
				const bool updated = sbfCommitEpoch();
				sbfResetEpoch();
				sbfResetFrame();
				return updated;
			} else {
				fprintf(stderr, "[GPS] Not end of PVT block\n"); // debugging only
			}
		} else {
			fprintf(stderr, "[GPS] CRC mismatch: expected %04X, calculated %04X\n", sbfState.header.crc, sbfCrc16(&sbfState.frame[4], (uint16_t)(sbfState.expectedLength - 4))); // debugging only
		}
		sbfResetFrame(); 
	}

	return false;
}

#endif // USE_GPS_SEPTENTRIO

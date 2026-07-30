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
// #include <stdio.h> // debugging only

#include "common/maths.h"
#include "io/gps.h"
#include "io/gps_septentrio.h"

static sbfParserState_t sbfState;
septentrioPortDetector_t portDetector;

static uint16_t sbfBlockId(const sbfHeader_t *header)
{
	return (uint16_t)(header->id_word & 0x1FFF);
}

// Before: CRC calculation was performed over the entire frame once the expected length was reached.
// Now: CRC calculation is performed incrementally as each byte is received,
// allowing early detection of corrupted frames and avoiding the need to store the entire frame before validation.

// This also allows handling frames exceeding SBF_MAX_FRAME_SIZE. Until this limit is reached
// (or for smaller frames), the payload is stored in the frame buffer while the CRC is computed progressively. 
// If the frame exceeds SBF_MAX_FRAME_SIZE, additional payload bytes are no longer stored, 
// but the CRC calculation continues until the expected frame length is reached.

// Without this change, handling frames larger than SBF_MAX_FRAME_SIZE would require skipping
// CRC validation to store the truncated payload, which could lead to accepting corrupted frames.

// static uint16_t sbfCrc16(const uint8_t *data, uint16_t length)
// {
// 	uint8_t x;
// 	uint16_t crc = 0;
// 	// Calculate CRC16 over the data, starting after the sync bytes 
// 	while (length--) {
// 		x = (uint8_t)((crc >> 8) ^ *data++);
// 		x ^= x >> 4;
// 		crc = (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ x);
// 	}
// 	return crc;
// }

static uint16_t sbfAccumulateCrc16(uint16_t crc, uint8_t data)
{
    uint8_t x = (uint8_t)((crc >> 8) ^ data);
    x ^= x >> 4;
    return (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ x);
}

static uint8_t sbfSvidToGnssId(uint16_t svid) {  
    if (svid >= 1   && svid <= 37)  return SEPTENTRIO_GNSS_GPS; 
    if (svid >= 38  && svid <= 61)  return SEPTENTRIO_GNSS_GLONASS;  
    if (svid == 62)                 return SEPTENTRIO_GNSS_GLONASS; // GLONASS unknown slot
    if (svid >= 63  && svid <= 68)  return SEPTENTRIO_GNSS_GLONASS;  
    if (svid >= 71  && svid <= 106) return SEPTENTRIO_GNSS_GALILEO; 
    // 107-119: L-Band MSS, no standard GNSS ID, skip
    if (svid >= 120 && svid <= 140) return SEPTENTRIO_GNSS_SBAS; 
    if (svid >= 141 && svid <= 180) return SEPTENTRIO_GNSS_BEIDOU;
    if (svid >= 181 && svid <= 190) return SEPTENTRIO_GNSS_QZSS; 
    if (svid >= 191 && svid <= 197) return SEPTENTRIO_GNSS_NAVIC; 
    if (svid >= 198 && svid <= 215) return SEPTENTRIO_GNSS_SBAS;  
    if (svid >= 216 && svid <= 222) return SEPTENTRIO_GNSS_NAVIC;
    if (svid >= 223 && svid <= 245) return SEPTENTRIO_GNSS_BEIDOU;
    if (svid >= 250 && svid <= 251) return SEPTENTRIO_GNSS_GPS;  
    return SEPTENTRIO_GNSS_UNKNOWN; // unknown, same sentinel as unused slot 
}

static uint8_t sbfSvidToSatId(uint16_t svid) {
    if (svid >= 1   && svid <= 37)  return svid;       // GPS G01-G37
    if (svid >= 38  && svid <= 61)  return svid - 37;  // GLONASS R01-R24
    if (svid == 62)                 return 0;          // GLONASS unknown slot
    if (svid >= 63  && svid <= 68)  return svid - 38;  // GLONASS R25-R30
    if (svid >= 71  && svid <= 106) return svid - 70;  // Galileo E01-E36
    if (svid >= 120 && svid <= 140) return svid - 100; // SBAS S20-S40
    if (svid >= 141 && svid <= 180) return svid - 140; // BeiDou C01-C40
    if (svid >= 181 && svid <= 190) return svid - 180; // QZSS J01-J10
    if (svid >= 191 && svid <= 197) return svid - 190; // NavIC I01-I07
    if (svid >= 198 && svid <= 215) return svid - 157; // SBAS S41-S58
    if (svid >= 216 && svid <= 222) return svid - 208; // NavIC I08-I14
    if (svid >= 223 && svid <= 245) return svid - 182; // BeiDou C41-C63
    if (svid >= 250 && svid <= 251) return svid - 212; // GPS G38-G39
    return (uint8_t)svid;
}

static void sbfResetFrame(void)
{
	sbfState.index = 0;
	sbfState.expectedLength = 0;
	sbfState.synced = false;
	sbfState.calculatedCrc = 0;
	memset(sbfState.frame, 0, sizeof(sbfState.frame));
}

static void sbfResetEpoch(void)
{
	sbfState.currentTow = 0;
	sbfState.currentWnc = 0;
	sbfState.havePvt = false;
	sbfState.haveDop = false;
	sbfState.haveVelCov = false;
	sbfState.haveChannelStatus = false;
	memset(&sbfState.pvt, 0, sizeof(sbfState.pvt));
	memset(&sbfState.dop, 0, sizeof(sbfState.dop));
	memset(&sbfState.velCov, 0, sizeof(sbfState.velCov));
	memset(sbfState.channelStatusPayload, 0, sizeof(sbfState.channelStatusPayload));
}

void gpsSeptentrioReset(void)
{
	memset(&sbfState, 0, sizeof(sbfState));
	sbfResetFrame();
	sbfResetEpoch();
}

void gpsSeptentrioPortDetectorReset(void)
{
    memset(portDetector.rxBuf, 0, sizeof(portDetector.rxBuf));
    portDetector.rxIdx = 0;
    portDetector.isDetected = false;
    portDetector.portName[0] = '\0'; // strictly empty port name (no fallback) 
}

static void sbfStartEpochIfNeeded(uint32_t tow)
{
	// If we have a new TOW, reset the epoch state to start accumulating new data for this epoch
	if (sbfState.currentTow != 0 && tow != sbfState.currentTow) {
		// fprintf(stderr, "[GPS] TOW Mismatch! Old: %u, New: %u\n", sbfState.currentTow, tow); // debugging only
		sbfState.havePvt = false;
		sbfState.haveDop = false;
		sbfState.haveVelCov = false;
	}
	sbfState.currentTow = tow;
}

static bool sbfCommitEpoch(void)
{
	// Commit the current epoch data to gpsSol if we have a complete PVT epoch
	// fprintf(stderr, "[GPS] Commit attempt (flags: PVT=%d, DOP=%d, VelCov=%d)\n", sbfState.havePvt, sbfState.haveDop, sbfState.haveVelCov); // debugging only

	if (!sbfState.havePvt) {
		// fprintf(stderr, "[GPS] Commit failed: Missing PVT block data!\n"); // debugging only
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
	// fprintf(stderr, "[GPS] Commit successful\n");
	// fprintf(stderr, "[GPS] Lat: %d, Lon: %d, Alt: %d cm, NumSat: %d, GroundSpeed: %d cm/s, GroundCourse: %d deg*10\n", gpsSol.llh.lat, gpsSol.llh.lon, gpsSol.llh.altCm, gpsSol.numSat, gpsSol.groundSpeed, gpsSol.groundCourse); // debugging only
	// fprintf(stderr, "[GPS] VelN: %d cm/s, VelE: %d cm/s, VelD: %d cm/s\n", gpsSol.velned.velN, gpsSol.velned.velE, gpsSol.velned.velD); // debugging only
	// fprintf(stderr, "[GPS] AccH: %d mm, AccV: %d mm, AccS: %d mm\n", gpsSol.acc.hAcc, gpsSol.acc.vAcc, gpsSol.acc.sAcc); // debugging only
	// fprintf(stderr, "[GPS] Nav Interval: %d ms, Fix State: %s\n", gpsSol.navIntervalMs, hasFix ? "FIX" : "NO FIX"); // debugging only
	// fprintf(stderr, "[GPS] DOP: PDOP=%d, HDOP=%d, VDOP=%d\n", gpsSol.dop.pdop, gpsSol.dop.hdop, gpsSol.dop.vdop); // debugging only

	return true;
}

static void sbfProcessChannelStatus(void) {
    const sbfChannelStatusHeader_t *header = (const sbfChannelStatusHeader_t *)sbfState.channelStatusPayload;

    uint8_t *sat = sbfState.channelStatusPayload + sizeof(*header); // pointer to the first ChannelSatInfo sub-block

    GPS_numCh = header->n;
	// fprintf(stderr, "[GPS] Header: n=%d (MAX=%d), sb1_length=%d, sb2_length=%d\n\n", header->n, GPS_SV_MAXSATS, header->sb1_length, header->sb2_length); // debugging only

	unsigned svCount = 0; // count of valid satellites processed

    for (unsigned i = 0; i < GPS_numCh; i++) { // loop over the number of satellites reported by the receiver 
		sbfChannelSatInfo_t s1;
		memcpy(&s1, sat, sizeof(s1));

		// Resolve SVID
		const uint16_t svid = (s1.svid != 0) ? s1.svid : s1.svid_full;
		const uint8_t gnssId = sbfSvidToGnssId(svid);

		// Walk N2 ChannelStateInfo sub-blocks to find main antenna tracking status
        uint8_t track = 0; 
        uint8_t quality = 0;
        const uint8_t *state = sat + header->sb1_length;

		for (uint8_t j = 0; j < s1.n2; j++) {
            sbfChannelStateInfo_t s2;
            memcpy(&s2, state, sizeof(s2));
            if (s2.antenna == 0) { // main antenna only for the quality assessment 
				// Extract the lower 2 bits for the tracking status
                track = s2.tracking_status & 0x3; 
                if      (track == 1) quality |= 1; // search 
                else if (track == 2) quality |= 2; // sync
                else if (track == 3) quality |= 5; // tracking = code+carrier locked

                if ((s2.pvt_status & 0x3) == 2) quality |= (1 << 3); // used in PVT
                break;
            }
            state += header->sb2_length; // advance to the next ChannelStateInfo sub-block
        }

		if (gnssId == 255 || track == 0) { // unknown constellation or idle/not applicable antenna tracking status
            sat += header->sb1_length + s1.n2 * header->sb2_length; // skip to the next ChannelSatInfo sub-block
            continue; 
        }

		if (svCount < GPS_SV_MAXSATS) { // only process up to the maximum number of satellites we can store
            GPS_svinfo[svCount].chn = gnssId;
            GPS_svinfo[svCount].svid = sbfSvidToSatId(svid);
            GPS_svinfo[svCount].cno  = 0; // not provided in ChannelStatus

            // Extract the health status from the lower 2 bits of the health_status field
            uint8_t health = s1.health_status & 0x3;
            if      (health == 1) quality |= (1 << 4); // healthy
            else if (health == 3) quality |= (2 << 4); // unhealthy 
            GPS_svinfo[svCount].quality = quality;

            svCount++;
        }

        sat += header->sb1_length + s1.n2 * header->sb2_length; // advance to the next ChannelSatInfo sub-block
	}

	GPS_numCh = svCount; // assign final active satellite count 

	// Fill the rest of the array with the standard sentinel values (as UBLOX does)
	for (unsigned i = svCount; i < GPS_SV_MAXSATS; i++) {
        GPS_svinfo[i] = (GPS_svinfo_t){ .chn = 255 };
    }

	// fprintf(stderr, "[GPS] Registered %d satellites\n", GPS_numCh); // debugging only
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
			// fprintf(stderr, "[GPS] Processing PVT Geodetic block\n"); // debugging only
			memcpy(&sbfState.pvt, payload, sizeof(sbfPvtGeodetic_t));
			sbfState.havePvt = true;
		}
		break;

	case SBF_BLOCK_DOP:
		if (payloadLength >= sizeof(sbfDop_t)) {
			// fprintf(stderr, "[GPS] Processing DOP block\n"); // debugging only
			memcpy(&sbfState.dop, payload, sizeof(sbfDop_t));
			sbfState.haveDop = true;
		}
		break;

	case SBF_BLOCK_VELCOVGEODETIC:
		if (payloadLength >= sizeof(sbfVelCovGeodetic_t)) {
			// fprintf(stderr, "[GPS] Processing Velocity Covariance Geodetic block\n"); // debugging only
			memcpy(&sbfState.velCov, payload, sizeof(sbfVelCovGeodetic_t));
			sbfState.haveVelCov = true;
		}
		break;

	case SBF_BLOCK_ENDOFPVT:
		// fprintf(stderr, "[GPS] Processing End of PVT block\n"); // debugging only
		// Epoch commit handled in gpsNewFrameSeptentrio(uint8_t) 
		break;

	case SBF_BLOCK_CHANNELSTATUS:
		if (payloadLength >= sizeof(sbfChannelStatusHeader_t)) {
			// fprintf(stderr, "[GPS] Processing Channel Status block\n"); // debugging only
			memcpy(sbfState.channelStatusPayload, payload, MIN(payloadLength, SBF_MAX_FRAME_SIZE - SBF_HEADER_SIZE));
    	    sbfState.channelStatusPayloadLength = payloadLength; // store the actual payload length for processing
			sbfState.haveChannelStatus = true;
			sbfProcessChannelStatus();
		}
		break;

	default:
		// fprintf(stderr, "[GPS] Unknown block ID: %d\n", blockId); // debugging only
		break;
	}
}

static void gpsSeptentrioProcessAck(uint8_t data)
{
    static uint8_t ackBuf[4]; // buffer to hold the last 4 bytes of the ACK/NACK response
    static uint8_t ackIdx = 0;

    ackBuf[ackIdx & 0x3] = data; // & 0x3 equivalent to modulo 4, keeps the index within the bounds of the buffer
    ackIdx++;

	// The reply to a valid command is the command itself, preceded by "$R: "
	// Otherwise, the reply is "$?: " for an invalid command
    if (data == '\n' && ackIdx >= 3) { // check for the end of the response line and ensure we have at least 3 bytes to check
        const uint8_t prev2 = ackBuf[(ackIdx - 3) & 0x3]; 
        const uint8_t prev1 = ackBuf[(ackIdx - 2) & 0x3];
        if (prev2 == '$' && prev1 == 'R') {
            gpsData.ackState = GPS_ACK_GOT_ACK; 
            ackIdx = 0;
        } else if (prev2 == '$' && prev1 == '?') { 
            gpsData.ackState = GPS_ACK_GOT_NACK;
            ackIdx = 0;
        }
    }
}

bool gpsSeptentrioProcessPort(uint8_t data)
{
	if (portDetector.isDetected) { // port already detected, no further processing needed
		return true; 
	}
	if (data == 0 || data == '\r') { 
        return false;
    }

	// Append byte to sliding buffer
	if (portDetector.rxIdx < SEPTENTRIO_RX_BUF_SIZE - 1) { // ensure space for null terminator
        portDetector.rxBuf[portDetector.rxIdx++] = (char)data;
        portDetector.rxBuf[portDetector.rxIdx] = '\0'; // null-terminate the string 
    } else { // sliding buffer is full, shift left and append new byte
        memmove(portDetector.rxBuf, portDetector.rxBuf + 1, SEPTENTRIO_RX_BUF_SIZE - 2); 
        portDetector.rxBuf[SEPTENTRIO_RX_BUF_SIZE - 2] = (char)data;
        portDetector.rxBuf[SEPTENTRIO_RX_BUF_SIZE - 1] = '\0';
    }

	// Match serial and USB port names directly preceding the '>' prompt character
	char *promptPtr = strchr(portDetector.rxBuf, '>');
	if (promptPtr != NULL) {
		// Reverse-search from '>' back to the start of rxBuf to find "COM" or "USB"
        // Future-proofs against the 1-digit port limit of 4-character matching (promptPtr - 4)
        // (e.g., "COM10" or "USB10" will be detected correctly)
		char *searchPtr = promptPtr - 1;
		while (searchPtr >= portDetector.rxBuf) {
            // Check if searchPtr currently points to the start of "COM" or "USB"
            if (strncmp(searchPtr, "COM", 3) == 0 || strncmp(searchPtr, "USB", 3) == 0) {
                size_t nameLen = promptPtr - searchPtr; // length of the port string 

                // Ensure the parsed name fits inside the destination buffer
                if (nameLen < SEPTENTRIO_PORT_NAME_LENGTH) {
                    strncpy(portDetector.portName, searchPtr, nameLen);
                    portDetector.portName[nameLen] = '\0'; // properly null-terminate
                    portDetector.isDetected = true;
                    return true;
                }
            }
            searchPtr--;
        }
	}
	return false; // continue accumulating bytes until a valid port name is detected 
}

bool gpsNewFrameSeptentrio(uint8_t data)
{	
	// Non-SBF data processing (port detection and ACK handling)
	if (gpsData.state == GPS_STATE_CONFIGURE && gpsData.state_position == SEPTENTRIO_CFG_DETECT_PORT) { 
		if (gpsSeptentrioProcessPort(data)) { 
			portDetector.isDetected = true;
			gpsData.ackState = GPS_ACK_GOT_ACK; // port detected, move to next configuration step
		}
		return false; // continue processing until the port is detected and configuration can proceed
	}
	if (gpsData.state == GPS_STATE_CONFIGURE && gpsData.ackState == GPS_ACK_WAITING) {
        gpsSeptentrioProcessAck(data); // process ACK/NACK responses for configuration commands
        return false;
    }

	// SBF frame processing
	if (!sbfState.synced) { // we are not yet synced, check for the sync sequence
		if (sbfState.index == 0) {
			if (data != SBF_SYNC1) return false;
			
			sbfState.frame[sbfState.index++] = data; // copy the first sync byte into the frame buffer 
			return false; // wait for the second sync byte
		}

		if (sbfState.index == 1) { // we have received the first sync byte, now check for the second
			if (data != SBF_SYNC2) {
				sbfResetFrame();
				return false;
			}
			// fprintf(stderr, "[GPS] Synced\n"); // debugging only
			sbfState.frame[sbfState.index++] = data; 
			sbfState.synced = true;     // valid sync sequence received, we are now synced
			sbfState.calculatedCrc = 0; // initialize the accumulated CRC for the frame (excluding the sync bytes)
			return false; // wait for the rest of the frame
		}
	}

	if (sbfState.index >= 4) { // CRC accumulation starts after sync and CRC fields (first 4 bytes of the frame)
		sbfState.calculatedCrc = sbfAccumulateCrc16(sbfState.calculatedCrc, data);
	}

	if (sbfState.index < SBF_MAX_FRAME_SIZE) { // only store bytes in the frame buffer if we haven't exceeded the defined maximum size
		sbfState.frame[sbfState.index] = data;
	} 
	sbfState.index++; // keep updating the index to count up to the true frame length 

	if (sbfState.index == SBF_HEADER_SIZE) { // get packet length from the header 
		memcpy(&sbfState.header, sbfState.frame, sizeof(sbfHeader_t));
		sbfState.expectedLength = sbfState.header.length;
		// fprintf(stderr, "[GPS] Expected frame length: %d (max: %d)\n", sbfState.expectedLength, SBF_MAX_FRAME_SIZE); // debugging only

		if (sbfState.expectedLength < SBF_HEADER_SIZE || sbfState.expectedLength > SBF_MAX_FRAME_SANITY_SIZE) {
			// fprintf(stderr, "[GPS] Invalid frame length: %d\n", sbfState.expectedLength); // debugging only
			sbfResetFrame();
			return false;
		}
	}

	// Once we have received the expected length of the frame, we can process it
	if (sbfState.expectedLength != 0 && sbfState.index >= sbfState.expectedLength) {
		memcpy(&sbfState.header, sbfState.frame, sizeof(sbfHeader_t));
		
		if (sbfState.calculatedCrc == sbfState.header.crc) { // accumulated CRC matches the expected CRC in the header
			const uint16_t blockId = sbfBlockId(&sbfState.header);
			sbfProcessBlock(); // only the bytes up to MAX_FRAME_SIZE are processed, any excess bytes are not included in the frame buffer and are ignored
			
			// Detect boundary block to commit the epoch
			if (blockId == SBF_BLOCK_ENDOFPVT) { 
				const bool updated = sbfCommitEpoch();
				sbfResetEpoch();
				sbfResetFrame();
				return updated;
			} 
		} else {
			// fprintf(stderr, "[GPS] CRC mismatch! Expected: %04X, progressively calculated: %04X\n", sbfState.header.crc, sbfState.calculatedCrc);
			// Skip until the next sync sequence is detected, reset the frame state
		}
		sbfResetFrame(); 
	}
	return false;
}

#endif // USE_GPS_SEPTENTRIO

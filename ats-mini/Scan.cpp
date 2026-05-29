#include "Common.h"
#include "Utils.h"
#include "Menu.h"

// Tuning delays after rx.setFrequency()
#define TUNE_DELAY_DEFAULT 30
#define TUNE_DELAY_FM      60
#define TUNE_DELAY_AM_SSB  80

#define SCAN_POLL_TIME    10 // Tuning status polling interval (msecs)
#define SCAN_POINTS      200 // Number of frequencies to scan
#define SCAN_CANDIDATES   32 // Max detected station candidates

#define SCAN_OFF    0   // Scanner off, no data
#define SCAN_RUN    1   // Scanner running
#define SCAN_DONE   2   // Scanner done, valid data in scanData[]

static struct
{
  uint8_t rssi;
  uint8_t snr;
} scanData[SCAN_POINTS];

static uint32_t scanTime = millis();
static uint8_t  scanStatus = SCAN_OFF;

static uint16_t scanStartFreq;
static uint16_t scanStep;
static uint16_t scanCount;
static uint8_t  scanMinRSSI;
static uint8_t  scanMaxRSSI;
static uint8_t  scanMinSNR;
static uint8_t  scanMaxSNR;

static inline uint8_t min(uint8_t a, uint8_t b) { return(a<b? a:b); }
static inline uint8_t max(uint8_t a, uint8_t b) { return(a>b? a:b); }

static bool namesMatch(const char *a, const char *b)
{
  if(!a || !b || !a[0] || !b[0]) return(false);

  for(; *a && *b ; a++, b++)
  {
    char ca = *a >= 'A' && *a <= 'Z' ? *a - 'A' + 'a' : *a;
    char cb = *b >= 'A' && *b <= 'Z' ? *b - 'A' + 'a' : *b;
    if(ca != cb) return(false);
  }

  return(*a == '\0' && *b == '\0');
}

float scanGetRSSI(uint16_t freq)
{
  // Input frequency must be in range of existing data
  if((scanStatus!=SCAN_DONE) || (freq<scanStartFreq) || (freq>=scanStartFreq+scanStep*scanCount))
    return(0.0);

  uint8_t result = scanData[(freq - scanStartFreq) / scanStep].rssi;
  return((result - scanMinRSSI) / (float)(scanMaxRSSI - scanMinRSSI + 1));
}

float scanGetSNR(uint16_t freq)
{
  // Input frequency must be in range of existing data
  if((scanStatus!=SCAN_DONE) || (freq<scanStartFreq) || (freq>=scanStartFreq+scanStep*scanCount))
    return(0.0);

  uint8_t result = scanData[(freq - scanStartFreq) / scanStep].snr;
  return((result - scanMinSNR) / (float)(scanMaxSNR - scanMinSNR + 1));
}

static void scanInit(uint16_t centerFreq, uint16_t step)
{
  scanStep    = step;
  scanCount   = 0;
  scanMinRSSI = 255;
  scanMaxRSSI = 0;
  scanMinSNR  = 255;
  scanMaxSNR  = 0;
  scanStatus  = SCAN_RUN;
  scanTime    = millis();

  const Band *band = getCurrentBand();
  int freq = scanStep * (centerFreq / scanStep - SCAN_POINTS / 2);

  // Adjust to band boundaries
  if(freq + scanStep * (SCAN_POINTS - 1) > band->maximumFreq)
    freq = band->maximumFreq - scanStep * (SCAN_POINTS - 1);
  if(freq < band->minimumFreq)
    freq = band->minimumFreq;
  scanStartFreq = freq;

  // Clear scan data
  memset(scanData, 0, sizeof(scanData));
}

static bool scanTickTime()
{
  // Scan must be on
  if((scanStatus!=SCAN_RUN) || (scanCount>=SCAN_POINTS)) return(false);

  // Wait for the right time
  if(millis() - scanTime < SCAN_POLL_TIME) return(true);

  // This is our current frequency to scan
  uint16_t freq = scanStartFreq + scanStep * scanCount;

  // Poll for the tuning status
  rx.getStatus(0, 0);
  if(!rx.getTuneCompleteTriggered())
  {
    scanTime = millis();
    return(true);
  }

  // If frequency not yet set, set it and wait until next call to measure
  if(rx.getCurrentFrequency() != freq)
  {
    rx.setFrequency(freq); // Implies tuning delay
    scanTime = millis() - SCAN_POLL_TIME;
    return(true);
  }

  // Measure RSSI/SNR values
  rx.getCurrentReceivedSignalQuality();
  scanData[scanCount].rssi = rx.getCurrentRSSI();
  scanData[scanCount].snr  = rx.getCurrentSNR();

  // Measure range of values
  scanMinRSSI = min(scanData[scanCount].rssi, scanMinRSSI);
  scanMaxRSSI = max(scanData[scanCount].rssi, scanMaxRSSI);
  scanMinSNR  = min(scanData[scanCount].snr, scanMinSNR);
  scanMaxSNR  = max(scanData[scanCount].snr, scanMaxSNR);

  // Next frequency to scan
  freq += scanStep;

  // Set next frequency to scan or expire scan
  if((++scanCount >= SCAN_POINTS) || !isFreqInBand(getCurrentBand(), freq) || consumeAbortPending())
    scanStatus = SCAN_DONE;
  else
    rx.setFrequency(freq); // Implies tuning delay

  // Save last scan time
  scanTime = millis() - SCAN_POLL_TIME;

  // Return current scan status
  return(scanStatus==SCAN_RUN);
}

//
// Run entire scan once
//
void scanRun(uint16_t centerFreq, uint16_t step)
{
  // Set tuning delay
  rx.setMaxDelaySetFrequency(currentMode == FM ? TUNE_DELAY_FM : TUNE_DELAY_AM_SSB);
  // Mute the audio
  muteOn(MUTE_TEMP, true);
  // Flag is set by rotary encoder and cleared on seek/scan entry
  seekStop = false;
  // Save current frequency
  uint16_t curFreq = rx.getFrequency();
  // Scan the whole range
  for(scanInit(centerFreq, step) ; scanTickTime(););
  // Restore current frequency
  rx.setFrequency(curFreq);
  // Unmute the audio
  muteOn(MUTE_TEMP, false);
  // Restore tuning delay
  rx.setMaxDelaySetFrequency(TUNE_DELAY_DEFAULT);
}

//
// Save scanned station candidates to memory slots.
// Returns number of slots written.
//
uint8_t scanStoreToMemory()
{
  if(scanStatus != SCAN_DONE || scanCount < 3) return(0);

  struct Candidate
  {
    uint16_t freq;
    uint16_t score;
  } candidates[SCAN_CANDIDATES];

  uint8_t candidateCount = 0;
  uint8_t minSnr = currentMode == FM ? 8 : 4;
  uint8_t minRssi = scanMinRSSI + (currentMode == FM ? 6 : 4);

  for(uint16_t i=1 ; i+1<scanCount ; i++)
  {
    uint8_t rssi = scanData[i].rssi;
    uint8_t snr = scanData[i].snr;

    // Keep only local maxima above minimal quality thresholds.
    if(rssi < minRssi || snr < minSnr) continue;
    if(rssi < scanData[i-1].rssi || rssi <= scanData[i+1].rssi) continue;

    uint16_t score = (uint16_t)rssi + (uint16_t)snr * 3;
    int insertAt = candidateCount;

    while(insertAt>0 && candidates[insertAt-1].score < score)
    {
      if(insertAt < SCAN_CANDIDATES) candidates[insertAt] = candidates[insertAt-1];
      insertAt--;
    }

    if(insertAt >= SCAN_CANDIDATES) continue;

    candidates[insertAt].freq = scanStartFreq + scanStep * i;
    candidates[insertAt].score = score;
    if(candidateCount < SCAN_CANDIDATES) candidateCount++;
  }

  uint8_t saved = 0;
  for(uint8_t i=0 ; i<candidateCount ; i++)
  {
    uint32_t freqHz = freqToHz(candidates[i].freq, currentMode);
    char candidateName[sizeof(memories[0].name)] = {0};
    bool exists = false;
    int8_t slot = -1;

    // Populate predictable station names (EiBi / known frequencies).
    identifyFrequency(candidates[i].freq);
    Memory tmp = {0};
    setMemoryName(&tmp, getStationName(), true);
    if(tmp.name[0]) memcpy(candidateName, tmp.name, sizeof(tmp.name));

    for(int j=0 ; j<getTotalMemories() ; j++)
    {
      if(memories[j].freq == freqHz && memories[j].band == bandIdx && memories[j].mode == currentMode)
      {
        exists = true;
        break;
      }
      if(candidateName[0] && memories[j].mode == currentMode && namesMatch(memories[j].name, candidateName))
      {
        exists = true;
        break;
      }
      if(slot<0 && !memories[j].freq) slot = j;
    }

    if(exists || slot<0) continue;

    memories[slot].freq = freqHz;
    memories[slot].band = bandIdx;
    memories[slot].mode = currentMode;
    memset(memories[slot].name, 0, sizeof(memories[slot].name));
    if(candidateName[0]) setMemoryName(&memories[slot], candidateName, true);
    saved++;
  }

  // Restore current station info after temporary name lookups.
  identifyFrequency(currentFrequency + currentBFO / 1000);
  if(saved) sortMemoriesByFrequency();

  return(saved);
}

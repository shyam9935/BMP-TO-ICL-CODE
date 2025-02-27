# BMP-TO-ICL-CODE
BMP TO ICL CODE
#include <Arduino.h>
#include <FS.h>
#include <SD.h>

// JPEG Huffman tables and quantization tables (standard tables for quality ~90%)
static const uint8_t LUM_QTABLE[64] PROGMEM = {  
  // Luminance Quantization Table (quality ~90%)
  3,  2,  2,  3,  2,  2,  3,  3,
  3,  4,  3,  3,  4,  5,  8,  5,
  5,  4,  4,  5, 10,  7,  7,  6,
  8, 12, 10, 12, 12, 11, 10, 11,
  11, 13, 14, 18, 16, 13, 14, 17,
  14, 11, 11, 16, 22, 16, 17, 19,
  20, 21, 21, 21, 12, 15, 23, 24,
  22, 20, 24, 18, 20, 21, 20,  // 64 values
};
static const uint8_t CHROMA_QTABLE[64] PROGMEM = {
  // Chrominance Quantization Table (quality ~90%)
  3,  4,  4,  5,  4,  5,  9,  5,
  5,  9, 20, 13, 11, 13, 20, 20,
  20, 20, 20, 20, 20, 20, 20, 20,
  20, 20, 20, 20, 20, 20, 20, 20,
  20, 20, 20, 20, 20, 20, 20, 20,
  20, 20, 20, 20, 20, 20, 20, 20,
  20, 20, 20, 20, 20, 20, 20, 20,
  20, 20, 20, 20, 20, 20, 20, 20,
};

// Standard Huffman Tables (JPEG Baseline) for DC and AC, Luminance and Chrominance
// The tables are given as lengths and symbols as per the JPEG standard
static const uint8_t STD_DC_LUM_LEN[16] PROGMEM  = { 0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0 };
static const uint8_t STD_DC_LUM_VAL[12] PROGMEM  = { 0,1,2,3,4,5,6,7,8,9,10,11 };
static const uint8_t STD_DC_CHROM_LEN[16] PROGMEM= { 0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0 };
static const uint8_t STD_DC_CHROM_VAL[12] PROGMEM= { 0,1,2,3,4,5,6,7,8,9,10,11 };

static const uint8_t STD_AC_LUM_LEN[16] PROGMEM  = { 0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7D };
static const uint8_t STD_AC_LUM_VAL[162] PROGMEM = {
  0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
  0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,
  0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
  0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
  0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
  0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
  0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
  0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,
  0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
  0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
  0xF9,0xFA
};
static const uint8_t STD_AC_CHROM_LEN[16] PROGMEM= { 0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77 };
static const uint8_t STD_AC_CHROM_VAL[162] PROGMEM = {
  0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
  0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,
  0x15,0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,
  0x27,0x28,0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,
  0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,
  0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,
  0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,
  0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,
  0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,
  0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
  0xF9,0xFA
};

// Buffers for bit output during Huffman encoding
static uint32_t bitBuffer = 0;
static int bitCount = 0;

// Forward declarations for helper functions
void write16(File &file, uint16_t value); 
void writeJPEGHeader(File &outFile, uint16_t width, uint16_t height);
void writeHuffmanBits(File &outFile, uint16_t code, uint8_t length);
void flushBits(File &outFile);
void encodeMCU(File &outFile, const uint8_t *blockY, const uint8_t *blockCb, const uint8_t *blockCr,
               int &prevDC_Y, int &prevDC_Cb, int &prevDC_Cr);
void fdct(float *block);
void quantizeAndZigZag(const float *block, const uint8_t *qtable, int16_t *zigzagOut);

// Writes a 16-bit value in **big-endian** to the file.
void write16(File &file, uint16_t value) {
  file.write(uint8_t(value >> 8));
  file.write(uint8_t(value & 0xFF));
}

// Writes the fixed JPEG header (SOI, JFIF, DQT, DHT, SOF0) to start a JPEG file.
void writeJPEGHeader(File &outFile, uint16_t width, uint16_t height) {
  // SOI (Start of Image)
  outFile.write(0xFF); outFile.write(0xD8);
  // APP0 (JFIF header)
  outFile.write(0xFF); outFile.write(0xE0);
  write16(outFile, 16); // APP0 length
  outFile.print("JFIF"); outFile.write(uint8_t(0)); // "JFIF" null-terminated
  outFile.write(uint8_t(1)); outFile.write(uint8_t(1)); // JFIF version 1.01
  outFile.write(uint8_t(0)); // No units
  write16(outFile, 1); write16(outFile, 1); // Xdensity=1, Ydensity=1
  outFile.write(uint8_t(0)); outFile.write(uint8_t(0)); // No thumbnail
  // DQT (Define Quantization Tables) - Luminance and Chrominance
  outFile.write(0xFF); outFile.write(0xDB);
  write16(outFile, 67); // DQT length (2 + 65)
  outFile.write(uint8_t(0)); // QTable info: precision=0 (8-bit), table ID=0 (lum)
  // Write luminance table
  for (uint8_t i = 0; i < 64; ++i) outFile.write(pgm_read_byte(&LUM_QTABLE[i]));
  // Second DQT for chroma
  outFile.write(0xFF); outFile.write(0xDB);
  write16(outFile, 67);
  outFile.write(uint8_t(1)); // Table ID=1 (chrom)
  for (uint8_t i = 0; i < 64; ++i) outFile.write(pgm_read_byte(&CHROMA_QTABLE[i]));
  // DHT (Define Huffman Tables) for DC and AC, luminance and chrominance
  // Luminance DC
  outFile.write(0xFF); outFile.write(0xC4);
  uint16_t dcLumLenSum = 0; for (int i = 0; i < 16; ++i) dcLumLenSum += pgm_read_byte(&STD_DC_LUM_LEN[i]);
  write16(outFile, 3 + 16 + dcLumLenSum); // DHT length
  outFile.write(uint8_t(0x00)); // DC Luminance table (class=0, id=0)
  for (uint8_t i = 0; i < 16; ++i) outFile.write(pgm_read_byte(&STD_DC_LUM_LEN[i]));
  for (uint8_t i = 0; i < dcLumLenSum; ++i) outFile.write(pgm_read_byte(&STD_DC_LUM_VAL[i]));
  // Luminance AC
  outFile.write(0xFF); outFile.write(0xC4);
  uint16_t acLumLenSum = 0; for (int i = 0; i < 16; ++i) acLumLenSum += pgm_read_byte(&STD_AC_LUM_LEN[i]);
  write16(outFile, 3 + 16 + acLumLenSum);
  outFile.write(uint8_t(0x10)); // AC Luminance table (class=1, id=0)
  for (uint8_t i = 0; i < 16; ++i) outFile.write(pgm_read_byte(&STD_AC_LUM_LEN[i]));
  for (uint8_t i = 0; i < acLumLenSum; ++i) outFile.write(pgm_read_byte(&STD_AC_LUM_VAL[i]));
  // Chrominance DC
  outFile.write(0xFF); outFile.write(0xC4);
  uint16_t dcChromLenSum = 0; for (int i = 0; i < 16; ++i) dcChromLenSum += pgm_read_byte(&STD_DC_CHROM_LEN[i]);
  write16(outFile, 3 + 16 + dcChromLenSum);
  outFile.write(uint8_t(0x01)); // DC Chrominance table (class=0, id=1)
  for (uint8_t i = 0; i < 16; ++i) outFile.write(pgm_read_byte(&STD_DC_CHROM_LEN[i]));
  for (uint8_t i = 0; i < dcChromLenSum; ++i) outFile.write(pgm_read_byte(&STD_DC_CHROM_VAL[i]));
  // Chrominance AC
  outFile.write(0xFF); outFile.write(0xC4);
  uint16_t acChromLenSum = 0; for (int i = 0; i < 16; ++i) acChromLenSum += pgm_read_byte(&STD_AC_CHROM_LEN[i]);
  write16(outFile, 3 + 16 + acChromLenSum);
  outFile.write(uint8_t(0x11)); // AC Chrominance table (class=1, id=1)
  for (uint8_t i = 0; i < 16; ++i) outFile.write(pgm_read_byte(&STD_AC_CHROM_LEN[i]));
  for (uint8_t i = 0; i < acChromLenSum; ++i) outFile.write(pgm_read_byte(&STD_AC_CHROM_VAL[i]));
  // SOF0 (Start of Frame: Baseline DCT)
  outFile.write(0xFF); outFile.write(0xC0);
  write16(outFile, 17);             // SOF length (17 bytes)
  outFile.write(uint8_t(8));        // precision: 8-bit
  write16(outFile, height);         // Image height
  write16(outFile, width);          // Image width
  outFile.write(uint8_t(3));        // number of components (Y, Cb, Cr)
  // Component 1: Y (Luminance), sampling factor 4:2:0 => (H=2, V=2 = 0x22), Q-table 0
  outFile.write(uint8_t(1)); outFile.write(uint8_t(0x22)); outFile.write(uint8_t(0));
  // Component 2: Cb (Chrominance), sampling factor 1:1 (0x11), Q-table 1
  outFile.write(uint8_t(2)); outFile.write(uint8_t(0x11)); outFile.write(uint8_t(1));
  // Component 3: Cr (Chrominance), sampling factor 1:1 (0x11), Q-table 1
  outFile.write(uint8_t(3)); outFile.write(uint8_t(0x11)); outFile.write(uint8_t(1));
  // SOS (Start of Scan) header (will be completed when writing scan data)
  outFile.write(0xFF); outFile.write(0xDA);
  write16(outFile, 12);             // SOS length (12 bytes for 3 components)
  outFile.write(uint8_t(3));        // number of components in scan
  // Y component: ID=1, Huffman tables: DC=0, AC=0
  outFile.write(uint8_t(1)); outFile.write(uint8_t(0x00));
  // Cb component: ID=2, Huffman tables: DC=1, AC=1
  outFile.write(uint8_t(2)); outFile.write(uint8_t(0x11));
  // Cr component: ID=3, Huffman tables: DC=1, AC=1
  outFile.write(uint8_t(3)); outFile.write(uint8_t(0x11));
  outFile.write(uint8_t(0x00));     // Ss (start of spectral or predictor selection)
  outFile.write(uint8_t(0x3F));     // Se (end of spectral selection, 0x3F for baseline)
  outFile.write(uint8_t(0x00));     // Ah/Al (successive approximation)
}

// Write Huffman bits to the output buffer, and flush to file when full (32 bits at a time).
void writeHuffmanBits(File &outFile, uint16_t code, uint8_t length) {
  // Append bits to the buffer (MSB first)
  bitBuffer |= ((uint32_t)code << (32 - bitCount - length));
  bitCount += length;
  while (bitCount >= 8) {
    uint8_t byte = (bitBuffer >> 24) & 0xFF;
    outFile.write(byte);
    // JPEG bitstream rule: insert 0x00 after any 0xFF byte in compressed data
    if (byte == 0xFF) outFile.write(uint8_t(0x00));
    bitBuffer <<= 8;
    bitCount -= 8;
  }
}

// Flush remaining bits by adding padding 1s and writing one more byte if needed.
void flushBits(File &outFile) {
  if (bitCount > 0) {
    // Pad with 1s (as per JPEG spec for end-of-stream)
    uint8_t byte = (bitBuffer >> 24) | ((0xFF << (8 - bitCount)) & 0xFF);
    outFile.write(byte);
    if (byte == 0xFF) outFile.write(uint8_t(0x00));
  }
  bitBuffer = 0;
  bitCount = 0;
}

// Perform forward DCT (Discrete Cosine Transform) on an 8x8 block of samples.
void fdct(float *block) {
  const float S = 0.35355339059f; // 1/(2*sqrt(2))
  float tmp[64];
  // 1-D DCT on rows
  for (int y = 0; y < 8; ++y) {
    float *row = block + 8*y;
    float a0 = row[0] + row[7];
    float a1 = row[1] + row[6];
    float a2 = row[2] + row[5];
    float a3 = row[3] + row[4];
    float a4 = row[3] - row[4];
    float a5 = row[2] - row[5];
    float a6 = row[1] - row[6];
    float a7 = row[0] - row[7];
    float b0 = a0 + a3;
    float b1 = a1 + a2;
    float b2 = a1 - a2;
    float b3 = a0 - a3;
    tmp[8*y + 0] =  (b0 + b1);
    tmp[8*y + 4] =  (b0 - b1);
    float c0 = (a6 + a5) * 0.70710678f; // sqrt(2)/2
    tmp[8*y + 2] =  (a7 + c0);
    tmp[8*y + 6] =  (a7 - c0);
    float c1 = a4 + a5;
    float c2 = a6 + a7;
    float c3 = a4 + a6;
    float c4 = a5 + a7;
    float c5 = (c3 + c4) * 0.38268343f; // cos(3*pi/8)
    float c6 = 0.5411961f * c3 + c5;    // cos(pi/8)
    float c7 = 1.30656296f * c4 + c5;   // cos(pi/16)
    tmp[8*y + 1] =  c1 * 0.70710678f + c6;
    tmp[8*y + 3] =  c2 * 0.70710678f + c7;
    tmp[8*y + 5] =  c2 * 0.70710678f - c7;
    tmp[8*y + 7] =  c1 * 0.70710678f - c6;
  }
  // 1-D DCT on columns of tmp, store back in block
  for (int x = 0; x < 8; ++x) {
    float a0 = tmp[x + 8*0] + tmp[x + 8*7];
    float a1 = tmp[x + 8*1] + tmp[x + 8*6];
    float a2 = tmp[x + 8*2] + tmp[x + 8*5];
    float a3 = tmp[x + 8*3] + tmp[x + 8*4];
    float a4 = tmp[x + 8*3] - tmp[x + 8*4];
    float a5 = tmp[x + 8*2] - tmp[x + 8*5];
    float a6 = tmp[x + 8*1] - tmp[x + 8*6];
    float a7 = tmp[x + 8*0] - tmp[x + 8*7];
    float b0 = a0 + a3;
    float b1 = a1 + a2;
    float b2 = a1 - a2;
    float b3 = a0 - a3;
    block[x + 8*0] =  (b0 + b1) * S;
    block[x + 8*4] =  (b0 - b1) * S;
    float c0 = (a6 + a5) * 0.70710678f;
    block[x + 8*2] =  (a7 + c0) * S;
    block[x + 8*6] =  (a7 - c0) * S;
    float c1 = a4 + a5;
    float c2 = a6 + a7;
    float c3 = a4 + a6;
    float c4 = a5 + a7;
    float c5 = (c3 + c4) * 0.38268343f;
    float c6 = 0.5411961f * c3 + c5;
    float c7 = 1.30656296f * c4 + c5;
    block[x + 8*1] =  (c1 * 0.70710678f + c6) * S;
    block[x + 8*3] =  (c2 * 0.70710678f + c7) * S;
    block[x + 8*5] =  (c2 * 0.70710678f - c7) * S;
    block[x + 8*7] =  (c1 * 0.70710678f - c6) * S;
  }
}

// Apply quantization and zigzag reorder to the 8x8 DCT block.
// Output is 64 coefficients in zigzag order, quantized to integers.
void quantizeAndZigZag(const float *block, const uint8_t *qtable, int16_t *zigzagOut) {
  // Zigzag index pattern for an 8x8 block
  static const uint8_t ZZ[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
  };
  for (uint8_t i = 0; i < 64; ++i) {
    // Quantize: divide by Q-table and round to nearest integer
    float val = block[i] / float(pgm_read_byte(&qtable[i]));
    // Round to nearest integer (towards zero)
    int16_t quantVal = (val > 0.0f) ? int16_t(val + 0.5f) : int16_t(val - 0.5f);
    // Place in zigzag order
    zigzagOut[ZZ[i]] = quantVal;
  }
}

// Encode one MCU (macroblock) consisting of 8x8 Y, 8x8 Cb, 8x8 Cr blocks.
void encodeMCU(File &outFile, const uint8_t *blockY, const uint8_t *blockCb, const uint8_t *blockCr,
               int &prevDC_Y, int &prevDC_Cb, int &prevDC_Cr) {
  // Buffers for DCT input (after level shift), and output for quantization
  float dctY[64], dctCb[64], dctCr[64];
  int16_t coeffY[64], coeffCb[64], coeffCr[64];
  // Level shift (BMP is 0-255, JPEG uses -128..127)
  for (int i = 0; i < 64; ++i) {
    dctY[i]  = blockY[i]  - 128.0f;
    dctCb[i] = blockCb[i] - 128.0f;
    dctCr[i] = blockCr[i] - 128.0f;
  }
  // Perform FDCT on each block
  fdct(dctY);  fdct(dctCb);  fdct(dctCr);
  // Quantize and Zigzag reorder
  quantizeAndZigZag(dctY, LUM_QTABLE, coeffY);
  quantizeAndZigZag(dctCb, CHROMA_QTABLE, coeffCb);
  quantizeAndZigZag(dctCr, CHROMA_QTABLE, coeffCr);
  // Calculate DC differences (DPCM)
  int16_t diffY  = coeffY[0]  - prevDC_Y;
  int16_t diffCb = coeffCb[0] - prevDC_Cb;
  int16_t diffCr = coeffCr[0] - prevDC_Cr;
  prevDC_Y  = coeffY[0];
  prevDC_Cb = coeffCb[0];
  prevDC_Cr = coeffCr[0];
  // Encode DC for Y (luminance) with Huffman
  // Determine category (number of bits needed) for diff
  auto calcCategory = [](int16_t v) {
    uint16_t absV = (v < 0) ? -v : v;
    uint8_t category = 0;
    while (absV) { absV >>= 1; category++; }
    return category;
  };
  auto writeValueBits = [&](int16_t value, uint8_t bits) {
    // For negative values in JPEG, complement the bits
    if (bits > 0) {
      if (value < 0) value = value - 1;
      uint16_t mask = (1 << bits) - 1;
      writeHuffmanBits(outFile, value & mask, bits);
    }
  };
  uint8_t catY  = calcCategory(diffY);
  uint8_t catCb = calcCategory(diffCb);
  uint8_t catCr = calcCategory(diffCr);
  // Huffman code lookup: (We know the standard DC tables are sorted by category)
  // For DC luminance, categories 0-11 have symbols 0-11 in order.
  uint8_t symY = catY;
  uint8_t symCb = catCb;
  uint8_t symCr = catCr;
  // Retrieve Huffman code (and length) for DC lum symbol symY (from STD_DC_LUM tables)
  // Build code tables for quick lookup (optional optimization in small-scale).
  // Here, we'll derive codes by reading lengths (since standard tables have known codes).
  // Hardcoded standard DC luminance Huffman codes for 0-11 (per JPEG spec):
  static const uint16_t DC_LUM_CODES[12] = {0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 
                                           0x0E, 0x1E, 0x3E, 0x7E, 0xFE, 0x1FE};
  static const uint8_t  DC_LUM_BITS[12]  = {2, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9};
  // Standard DC chrominance codes for 0-11 (same lengths except two codes differ):
  static const uint16_t DC_CHROM_CODES[12] = {0x00, 0x01, 0x02, 0x06, 0x0E, 0x1E,
                                             0x3E, 0x7E, 0xFE, 0x1FE, 0x3FE, 0x7FE};
  static const uint8_t  DC_CHROM_BITS[12]  = {2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  // Write DC coefficients
  writeHuffmanBits(outFile, DC_LUM_CODES[symY], DC_LUM_BITS[symY]); 
  writeValueBits(diffY, symY);
  writeHuffmanBits(outFile, DC_CHROM_CODES[symCb], DC_CHROM_BITS[symCb]);
  writeValueBits(diffCb, symCb);
  writeHuffmanBits(outFile, DC_CHROM_CODES[symCr], DC_CHROM_BITS[symCr]);
  writeValueBits(diffCr, symCr);
  // Encode AC coefficients for each block (Y has 4 blocks in 4:2:0, but we call encodeMCU per 8x8 block, adjusting input accordingly).
  auto encodeAC = [&](const int16_t *coeff, bool isLuminance) {
    // Huffman tables: use standard AC Luminance or Chrominance
    // We need a mapping from run-length to (code, length). For simplicity, we'll precompute a small table:
    static bool initialized = false;
    static uint16_t AC_LUM_CODES[256];
    static uint8_t  AC_LUM_BITS[256];
    static uint16_t AC_CHROM_CODES[256];
    static uint8_t  AC_CHROM_BITS[256];
    if (!initialized) {
      // Build AC Huffman code tables from length and value tables
      auto buildTable = [&](const uint8_t *lenTable, const uint8_t *valTable,
                             uint16_t *codeTable, uint8_t *bitsTable) {
        uint16_t code = 0;
        for (uint8_t n=1, idx=0; n<=16; ++n) { // for each code length
          for (uint8_t i=0; i < pgm_read_byte(&lenTable[n-1]); ++i) {
            uint8_t sym = pgm_read_byte(&valTable[idx++]);
            codeTable[sym] = code;
            bitsTable[sym] = n;
            code++;
          }
          code <<= 1;
        }
      };
      buildTable(STD_AC_LUM_LEN, STD_AC_LUM_VAL, AC_LUM_CODES, AC_LUM_BITS);
      buildTable(STD_AC_CHROM_LEN, STD_AC_CHROM_VAL, AC_CHROM_CODES, AC_CHROM_BITS);
      initialized = true;
    }
    // Iterate over AC coefficients in zigzag (1 to 63)
    uint8_t zeroRun = 0;
    for (uint8_t k = 1; k < 64; ++k) {
      int16_t acVal = coeff[k];
      if (acVal == 0) {
        ++zeroRun;
        continue;
      }
      // Emit any required ZRL codes for runs >=16
      while (zeroRun > 15) {
        // ZRL (zero run length 16): symbol 0xF0
        uint8_t sym = 0xF0;
        if (isLuminance) {
          writeHuffmanBits(outFile, AC_LUM_CODES[sym], AC_LUM_BITS[sym]);
        } else {
          writeHuffmanBits(outFile, AC_CHROM_CODES[sym], AC_CHROM_BITS[sym]);
        }
        zeroRun -= 16;
      }
      // Calculate size (category) of acVal
      uint8_t cat = 0; uint16_t absVal = (acVal < 0) ? -acVal : acVal;
      while (absVal) { absVal >>= 1; cat++; }
      // Symbol is (run << 4) | size
      uint8_t symbol = (zeroRun << 4) | cat;
      // Write Huffman code for this AC symbol
      if (isLuminance) {
        writeHuffmanBits(outFile, AC_LUM_CODES[symbol], AC_LUM_BITS[symbol]);
      } else {
        writeHuffmanBits(outFile, AC_CHROM_CODES[symbol], AC_CHROM_BITS[symbol]);
      }
      // Write the coefficient bits
      writeValueBits(acVal, cat);
      zeroRun = 0;
    }
    // If there are trailing zeros, write End-of-Block (EOB) if not already written
    if (zeroRun > 0) {
      // EOB symbol = 0x00 (0 run, 0 size)
      uint8_t sym = 0x00;
      if (isLuminance) {
        writeHuffmanBits(outFile, AC_LUM_CODES[sym], AC_LUM_BITS[sym]);
      } else {
        writeHuffmanBits(outFile, AC_CHROM_CODES[sym], AC_CHROM_BITS[sym]);
      }
    }
  };
  // Encode AC coefficients for Y, Cb, Cr blocks
  encodeAC(coeffY, true);
  encodeAC(coeffCb, false);
  encodeAC(coeffCr, false);
}

// Main function to convert a BMP file to an ICL (with one JPEG image).
bool convertBmpToIcl(const char *bmpFilename, const char *iclFilename, uint16_t fileID) {
  File bmpFile = SD.open(bmpFilename);
  if (!bmpFile) {
    Serial.printf("Failed to open BMP file: %s\n", bmpFilename);
    return false;
  }
  // Read BMP header
  uint8_t header[54];
  if (bmpFile.read(header, 54) != 54 || header[0] != 'B' || header[1] != 'M') {
    Serial.println("Not a valid BMP file");
    bmpFile.close();
    return false;
  }
  // Extract image dimensions and bit depth from BMP header
  uint32_t bmpWidth  = *(uint32_t*)&header[18];
  uint32_t bmpHeight = *(uint32_t*)&header[22];
  uint16_t bmpBpp    = *(uint16_t*)&header[28];
  if (bmpBpp != 8 && bmpBpp != 24) {
    // For simplicity, we handle 8-bit (grayscale/palette) or 24-bit (RGB) BMP.
    Serial.println("Unsupported BMP format (only 8-bit or 24-bit supported)");
    bmpFile.close();
    return false;
  }
  // BMP data offset
  uint32_t dataOffset = *(uint32_t*)&header[10];
  // Determine image stride (padded row size to multiple of 4)
  uint32_t rowSize = ((bmpBpp * bmpWidth + 31) / 32) * 4;
  // Allocate buffer for one row of BMP pixels (plus palette for 8-bit if needed)
  uint8_t *rowBuf = (uint8_t*)malloc(rowSize);
  uint8_t palette[1024];
  if (bmpBpp == 8) {
    // Read the color palette (256 entries * 4 bytes each = 1024 bytes)
    bmpFile.read(palette, 1024);
  }
  // Open output ICL file for writing
  File iclFile = SD.open(iclFilename, FILE_WRITE);
  if (!iclFile) {
    Serial.printf("Failed to create ICL file: %s\n", iclFilename);
    bmpFile.close();
    free(rowBuf);
    return false;
  }
  // Reserve space for the ICL header (we'll fill it later after JPEG is written)
  // Header is 32 bytes (DGUS_3 + CRC + length + ID + zeros + pointer table entries)
  // We also have a 10-byte info block before the JPEG data.
  uint8_t iclHeader[46];
  memset(iclHeader, 0, sizeof(iclHeader));
  // Fill DGUS_3 signature
  memcpy(iclHeader, "DGUS_3", 6);
  // Leave CRC (bytes [6-7]) and length (bytes [8-11]) as 0 for now.
  // Set ID (bytes [12-13] as little-endian)
  iclHeader[12] = fileID & 0xFF;
  iclHeader[13] = (fileID >> 8) & 0xFF;
  // The next two bytes [14-15] should mirror the ID for icon library (older files varied, new ones use ID exactly).
  iclHeader[14] = iclHeader[12];
  iclHeader[15] = iclHeader[13];
  // Pointer table: entry for this image ID at offset 32 (0-based indexing)
  // Each ID offset = 32 + 4*(ID - 4). But an easier approach: we write pointer for this ID and zeros for others < ID.
  // For simplicity, assume fileID is minimal (like 4 or 5) since each .icl usually contains one image. 
  // If needed to support multiple images, one would adjust pointer offsets accordingly.
  uint32_t imgInfoOffset = 0x00000020 + 4; // pointer to info block (which starts at offset 36 or 40, etc.)
  if (fileID == 4) {
    // If ID=4, info block starts right after pointer table (offset 36 decimal = 0x24)
    imgInfoOffset = 0x00000024;
  } else {
    // If ID > 4, space for previous pointer entries that are zero.
    // For ID=5, pointer starts at 40 (0x28). For ID=6, pointer at 44 (0x2C), etc.
    imgInfoOffset = 0x20 + (fileID - 3) * 4;
  }
  // Place pointer (big-endian)
  iclHeader[32] = (imgInfoOffset >> 24) & 0xFF;
  iclHeader[33] = (imgInfoOffset >> 16) & 0xFF;
  iclHeader[34] = (imgInfoOffset >> 8) & 0xFF;
  iclHeader[35] = imgInfoOffset & 0xFF;
  // Pointers for lower IDs remain 0 (which they already are in iclHeader initialized to 0)
  // Prepare image info block at bytes 36-45:
  // Width (big-endian):
  iclHeader[36] = (bmpWidth >> 8) & 0xFF;
  iclHeader[37] = bmpWidth & 0xFF;
  // Type/Category: use 0x0100 (consistent with new ICL files)
  iclHeader[38] = 0x01;
  iclHeader[39] = 0x00;
  // Constant 0x0272:
  iclHeader[40] = 0x02;
  iclHeader[41] = 0x72;
  // Reserved 0x0000:
  iclHeader[42] = 0x00;
  iclHeader[43] = 0x00;
  // Last field (placeholder for now, will fill after writing JPEG):
  iclHeader[44] = 0x00;
  iclHeader[45] = 0x00;
  // Write the header (46 bytes so far) to the ICL file
  iclFile.write(iclHeader, sizeof(iclHeader));
  // Begin writing JPEG data:
  writeJPEGHeader(iclFile, bmpWidth, bmpHeight);
  // JPEG encoding: process image in 8x8 MCU blocks with 4:2:0 subsampling.
  // For 4:2:0, we process 16x16 pixel macroblocks (4 Y blocks, 1 Cb, 1 Cr per MCU).
  // Compute number of MCU blocks horizontally and vertically:
  uint16_t mcuXcount = (bmpWidth + 15) / 16;
  uint16_t mcuYcount = (bmpHeight + 15) / 16;
  // Allocate buffers for one MCU (Y, Cb, Cr blocks)
  uint8_t mcuY[4][64], mcuCb[64], mcuCr[64];
  int prevDC_Y = 0, prevDC_Cb = 0, prevDC_Cr = 0;
  // BMPs are typically stored bottom-to-top, so we iterate from bottom row to top.
  for (uint16_t my = 0; my < mcuYcount; ++my) {
    for (uint16_t mx = 0; mx < mcuXcount; ++mx) {
      // Initialize MCU blocks with 128 (to handle padding outside image)
      memset(mcuY, 128, sizeof(mcuY));
      memset(mcuCb, 128, sizeof(mcuCb));
      memset(mcuCr, 128, sizeof(mcuCr));
      // Load up to 16x16 pixels for this MCU
      for (uint8_t y = 0; y < 16; ++y) {
        int imgY = bmpHeight - 1 - (my * 16 + y); // BMP index (bottom origin)
        if (imgY < 0) break; // outside image vertically
        // Seek to the beginning of row imgY in BMP data
        uint32_t seekPos = dataOffset + imgY * rowSize + mx * 16 * (bmpBpp / 8);
        bmpFile.seek(seekPos);
        // Read up to 16 pixels (depending on width)
        uint8_t pixelBuffer[48]; // enough for 16 pixels * 3 bytes (24bpp)
        uint8_t count = bmpFile.read(pixelBuffer, (bmpBpp/8) * min((uint16_t)16, (uint16_t)(bmpWidth - mx*16)));
        // If BMP is 8bpp, map palette index to grayscale (assuming grayscale palette)
        for (uint8_t x = 0; x < 16; ++x) {
          if (x >= bmpWidth - mx*16) {
            // beyond image width
            break;
          }
          if (bmpBpp == 24) {
            // BMP stores BGR
            uint8_t B = pixelBuffer[x*3 + 0];
            uint8_t G = pixelBuffer[x*3 + 1];
            uint8_t R = pixelBuffer[x*3 + 2];
            // Compute Y, Cb, Cr for this pixel (ITU-R BT.601 conversion)
            // Y range 0-255, Cb/Cr range 0-255 with 128 as center
            uint8_t Y  = (uint8_t) constrain(( 0.299f*R + 0.587f*G + 0.114f*B), 0, 255);
            uint8_t Cb = (uint8_t) constrain(( -0.1687f*R - 0.3313f*G + 0.5f*B + 128), 0, 255);
            uint8_t Cr = (uint8_t) constrain(( 0.5f*R - 0.4187f*G - 0.0813f*B + 128), 0, 255);
            // Which 8x8 block within MCU:
            uint8_t blockIndex = (y / 8) * 2 + (x / 8);
            uint8_t withinX = x % 8;
            uint8_t withinY = y % 8;
            mcuY[blockIndex][withinY * 8 + withinX] = Y;
            // Subsample Cb/Cr for each 16x16 MCU: each Cb/Cr covers 2x2 Y pixels
            if ((x % 2 == 0) && (y % 2 == 0)) {
              // Map 16x16 region to 8x8 Cb/Cr block coordinates
              uint8_t cx = x / 2;
              uint8_t cy = y / 2;
              mcuCb[cy * 8 + cx] = Cb;
              mcuCr[cy * 8 + cx] = Cr;
            }
          } else if (bmpBpp == 8) {
            // 8-bit BMP: pixelBuffer contains indices to palette
            uint8_t idx = pixelBuffer[x];
            // Palette format: B,G,R,0 for each index (4 bytes per palette entry)
            uint8_t B = palette[idx*4 + 0];
            uint8_t G = palette[idx*4 + 1];
            uint8_t R = palette[idx*4 + 2];
            // Convert palette to grayscale using lum. formula (assuming grayscale palette in input)
            uint8_t Y = (uint8_t) constrain((0.299f*R + 0.587f*G + 0.114f*B), 0, 255);
            uint8_t Cb = 128, Cr = 128; // grayscale means Cb/Cr at midpoint
            uint8_t blockIndex = (y / 8) * 2 + (x / 8);
            uint8_t withinX = x % 8;
            uint8_t withinY = y % 8;
            mcuY[blockIndex][withinY * 8 + withinX] = Y;
            if ((x % 2 == 0) && (y % 2 == 0)) {
              uint8_t cx = x / 2;
              uint8_t cy = y / 2;
              mcuCb[cy * 8 + cx] = Cb;
              mcuCr[cy * 8 + cx] = Cr;
            }
          }
        }
      }
      // Encode the 4 luminance blocks and 1 chroma (Cb and Cr) blocks for this MCU
      // Actually, with our encodeMCU, we process one 8x8 at a time, so we call it for each Y block and each chroma block.
      // But easier: encode 4 Y blocks separately, then Cb, then Cr:
      for (int by = 0; by < 4; ++by) {
        encodeMCU(iclFile, mcuY[by], mcuCb, mcuCr, prevDC_Y, prevDC_Cb, prevDC_Cr);
      }
      // (Note: A more optimal approach is to encode an MCU as a whole, but we reused encodeMCU for simplicity by multiple calls.)
    }
  }
  // Flush any remaining bits in Huffman buffer
  flushBits(iclFile);
  // Write EOI (End Of Image) marker
  iclFile.write(0xFF); iclFile.write(0xD9);
  // Done writing JPEG data. Now update the ICL header's last field and CRC.
  size_t fileSize = iclFile.size();
  // Calculate offset of JPEG start: it's 46 bytes (header+info block) in our output file.
  size_t jpegStartOffset = sizeof(iclHeader);
  size_t jpegDataLength = fileSize - jpegStartOffset;
  // Compute last field = (file_length - jpegStartOffset - 0x0272)
  uint16_t lastField = (uint16_t)((fileSize - jpegStartOffset - 0x0272) & 0xFFFF);
  iclHeader[44] = uint8_t(lastField >> 8);
  iclHeader[45] = uint8_t(lastField & 0xFF);
  // Compute CRC-16 (Modbus) from offset 8 to end of file.
  // CRC-16 Modbus uses polynomial 0xA001 (reverse of 0x8005), init 0xFFFF, and no XORout.
  uint16_t crc = 0xFFFF;
  iclFile.seek(8);
  for (size_t i = 8; i < fileSize; ++i) {
    int c = iclFile.read();
    if (c < 0) break;
    crc ^= (uint8_t)c;
    for (int b = 0; b < 8; ++b) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc = crc >> 1;
    }
  }
  // Fill CRC in header (big-endian)
  iclHeader[6] = uint8_t(crc >> 8);
  iclHeader[7] = uint8_t(crc & 0xFF);
  // Fill length field (fileSize - 8, big-endian)
  uint32_t lengthField = fileSize - 8;
  iclHeader[8] = uint8_t(lengthField >> 24);
  iclHeader[9] = uint8_t((lengthField >> 16) & 0xFF);
  iclHeader[10] = uint8_t((lengthField >> 8) & 0xFF);
  iclHeader[11] = uint8_t(lengthField & 0xFF);
  // Write updated header back to file (at beginning)
  iclFile.seek(0);
  iclFile.write(iclHeader, sizeof(iclHeader));
  // Clean up
  iclFile.close();
  bmpFile.close();
  free(rowBuf);
  return true;
}

void setup() {
  Serial.begin(115200);
  // Initialize SD card (adjust SS pin if needed for your setup)
  if (!SD.begin(/*SS pin*/)) {
    Serial.println("Card Mount Failed");
    return;
  }
  // Example: convert "4n.BMP" to "4n_out.icl"
  if (convertBmpToIcl("/4n.BMP", "/4n_out.icl", 4)) {
    Serial.println("4n.BMP converted to 4n_out.icl");
  }
  if (convertBmpToIcl("/5n.BMP", "/5n_out.icl", 5)) {
    Serial.println("5n.BMP converted to 5n_out.icl");
  }
}

void loop() {
  // Nothing in loop
} 

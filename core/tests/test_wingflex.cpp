#include "Check.h"
#include "a320/WingFlexReports.h"

using namespace a320::wingflex;

TEST(wingflex_fcu_output_report) {
  FcuOutput o;
  o.loc = true;
  o.appr = true;
  o.hdgDashed = true;
  o.spd = 1234;
  o.hdg = 360;
  o.alt = 35000;
  o.vs = -1;
  const Payload p = buildFcu(o);
  CHECK(p[0] == 0xF2 && p[1] == 0xE1 && p[2] == 0x03);
  CHECK(p[6] == (0x01 | 0x20));  // LOC and APPR (MobiFlight FcuCubeReport bits)
  CHECK(p[7] == 0x02);           // HDG dashed
  CHECK(p[8] == 0x01);           // power
  CHECK(p[11] == 0xC0 && p[12] == 0xC0);
  CHECK(p[15] == 0x04 && p[16] == 0xD2);  // 1234
  CHECK(p[17] == 0x01 && p[18] == 0x68);  // 360
  CHECK(p[21] == 0xFF && p[22] == 0xFF);  // -1 in two's complement
}

TEST(wingflex_fcu_input_report) {
  uint8_t r[64] = {0xF2, 0xE1, 0x03, 0x02, 0x01, 0x02};
  r[7] = 0x20 | 0x80;  // AP1 and A/THR
  r[8] = 0x04;         // APPR
  r[6] = 0x80;         // ALT 1000
  r[11] = 0x02;        // SPD knob two clicks right
  r[13] = 0xFF;        // ALT knob one click left
  r[15] = 0x40;
  FcuInput in;
  CHECK(parseFcu(r, sizeof(r), in));
  CHECK(in.button[kAp1] && in.button[kAthr] && in.button[kAppr] && in.button[kAlt1000] && !in.button[kAp2]);
  CHECK(in.knob[kSpdKnob] == 2 && in.knob[kAltKnob] == -1 && in.knob[kHdgKnob] == 0);
  CHECK(in.backlight == 0x40);
  r[2] = 0x05;
  CHECK(!parseFcu(r, sizeof(r), in));  // an EFIS report
}

TEST(wingflex_brightness_ignores_jitter) {
  CHECK(steadyBrightness(0xC0, 0xC2) == 0xC0);
  CHECK(steadyBrightness(0xC0, 0xBE) == 0xC0);
  CHECK(steadyBrightness(0xC0, 0x80) == 0x80);
  CHECK(steadyBrightness(0xFB, 0xFF) == 0xFF);  // the end stop is always reached
  CHECK(steadyBrightness(0x03, 0x00) == 0x00);
}

TEST(wingflex_efis_reports) {
  EfisOutput o;
  o.ls = true;
  o.masterWarn = true;
  const Payload p = buildEfis(o);
  CHECK(p[0] == 0xF2 && p[2] == 0x05 && p[3] == 0x03 && p[5] == 0x03);
  CHECK(p[6] == (0x01 | 0x20));      // MASTER WARN, LS
  CHECK(p[7] == (0x08 | 0x80));      // host light control, STD
  CHECK(p[8] == 0x01 && p[13] == 0x03 && p[14] == 0x02);
  CHECK(p[15] == 0x03 && p[16] == 0xF5);  // 1013
  uint8_t r[64] = {0xF2, 0xE1, 0x05, 0x02, 0x01, 0x04};
  r[6] = 0x20;   // LS
  r[7] = 0x20;   // rotary LS
  r[8] = 0x08;   // range 20
  r[12] = 0xFE;  // baro two clicks left
  EfisInput in;
  CHECK(parseEfis(r, sizeof(r), in));
  CHECK(in.button[kLs] && in.button[kNdLs] && in.button[kRange20] && !in.button[kRange10]);
  CHECK(in.baroKnob == -2);
}

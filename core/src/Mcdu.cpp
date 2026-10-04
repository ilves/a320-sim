#include "a320/Mcdu.h"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "a320/Systems.h"
#include "a320/Units.h"

namespace a320 {
namespace {

constexpr int kRows = A320_MCDU_ROWS;
constexpr int kCols = A320_MCDU_COLS;
constexpr size_t kScratchMax = 22;

const char* kFormatError = "FORMAT ERROR";
const char* kOutOfRange = "ENTRY OUT OF RANGE";
const char* kNotAllowed = "NOT ALLOWED";
const char* kNotInDatabase = "NOT IN DATA BASE";

// Writes into the 14 x 24 character grid. Label rows (odd) default to the small font.
class Screen {
 public:
  explicit Screen(A320McduDisplay& d) : d_(d) {
    for (int r = 0; r < kRows; ++r) {
      std::memset(d_.text[r], ' ', kCols);
      d_.text[r][kCols] = '\0';
      std::memset(d_.color[r], A320_MCDU_WHITE, kCols);
      std::memset(d_.small[r], (r % 2 == 1 && r < 13) ? 1 : 0, kCols);
    }
  }

  void put(int row, int col, const std::string& t, A320McduColor color, int small = -1) {
    if (row < 0 || row >= kRows) return;
    for (size_t i = 0; i < t.size(); ++i) {
      const int c = col + static_cast<int>(i);
      if (c < 0 || c >= kCols) continue;
      d_.text[row][c] = t[i];
      d_.color[row][c] = static_cast<uint8_t>(color);
      if (small >= 0) d_.small[row][c] = static_cast<uint8_t>(small);
    }
  }
  void left(int row, const std::string& t, A320McduColor c, int small = -1) { put(row, 0, t, c, small); }
  void right(int row, const std::string& t, A320McduColor c, int small = -1) {
    put(row, kCols - static_cast<int>(t.size()), t, c, small);
  }
  void centre(int row, const std::string& t, A320McduColor c, int small = -1) {
    put(row, (kCols - static_cast<int>(t.size())) / 2, t, c, small);
  }

 private:
  A320McduDisplay& d_;
};

std::string fmt(const char* f, ...) {
  char buf[64];
  va_list args;
  va_start(args, f);
  std::vsnprintf(buf, sizeof(buf), f, args);
  va_end(args);
  return buf;
}

std::string freqText(double mhz) { return fmt("%.2f", mhz); }
std::string boxes(int n) { return std::string(static_cast<size_t>(n), '#'); }

bool parseInt(const std::string& s, int& out) {
  if (s.empty()) return false;
  size_t i = (s[0] == '-' || s[0] == '+') ? 1 : 0;
  if (i == s.size()) return false;
  for (size_t k = i; k < s.size(); ++k)
    if (!std::isdigit(static_cast<unsigned char>(s[k]))) return false;
  out = std::atoi(s.c_str());
  return true;
}

bool parseNumber(const std::string& s, double& out) {
  if (s.empty()) return false;
  bool digit = false, dot = false;
  for (size_t k = 0; k < s.size(); ++k) {
    const char c = s[k];
    if (std::isdigit(static_cast<unsigned char>(c))) digit = true;
    else if (c == '.' && !dot) dot = true;
    else if (!((c == '-' || c == '+') && k == 0)) return false;
  }
  if (!digit) return false;
  out = std::atof(s.c_str());
  return true;
}

double wrap360(double d) {
  d = std::fmod(d, 360.0);
  return d < 0.0 ? d + 360.0 : d;
}

double runwayLengthM(const McduContext& ctx, int i) {
  const Runway& r = ctx.airport.runways[static_cast<size_t>(i)];
  const Enu a = ctx.frame.toEnu(r.start), b = ctx.frame.toEnu(r.end);
  return std::hypot(b.e - a.e, b.n - a.n);
}

int findIls(const Airport& airport, const std::string& entry) {
  // "ILK", "109.30" or "ILK/109.30"
  std::string ident = entry, freq;
  const size_t slash = entry.find('/');
  if (slash != std::string::npos) {
    ident = entry.substr(0, slash);
    freq = entry.substr(slash + 1);
  } else if (!entry.empty() && std::isdigit(static_cast<unsigned char>(entry[0]))) {
    ident.clear();
    freq = entry;
  }
  double mhz = 0.0;
  if (!freq.empty() && !parseNumber(freq, mhz)) return -2;
  for (size_t i = 0; i < airport.runways.size(); ++i) {
    const IlsSpec& ils = airport.runways[i].ils;
    if (ils.ident.empty()) continue;
    const bool identOk = ident.empty() || ident == ils.ident;
    const bool freqOk = freq.empty() || std::fabs(mhz - ils.frequencyMHz) < 0.006;
    if (identOk && freqOk) return static_cast<int>(i);
  }
  return -1;
}

std::string runwayName(const McduContext& ctx, int i) {
  return i >= 0 && i < static_cast<int>(ctx.airport.runways.size()) ? ctx.airport.runways[static_cast<size_t>(i)].ident
                                                                     : std::string();
}

// Bearing (magnetic) and distance from the aircraft to a runway threshold.
void toThreshold(const McduContext& ctx, int i, double& brgMag, double& distNm) {
  const Enu t = ctx.frame.toEnu(ctx.airport.runways[static_cast<size_t>(i)].threshold);
  const double dn = t.n - ctx.state.northM, de = t.e - ctx.state.eastM;
  brgMag = wrap360(std::atan2(de, dn) * kRadToDeg - ctx.airport.magneticVariationDeg);
  distNm = std::hypot(dn, de) / kNmToM;
}

const char* phaseName(const McduContext& ctx) {
  const A320State& s = ctx.state;
  if (s.onGround) return ctx.fms.flown ? "DONE" : "PREFLIGHT";
  if (s.flapsLever > 0 && ctx.fms.flown && s.verticalSpeedFpm < 300.0) return "APPROACH";
  if (s.radioAltFt < 1500.0 && s.verticalSpeedFpm > 300.0) return "TAKE OFF";
  if (s.verticalSpeedFpm > 500.0) return "CLIMB";
  if (s.verticalSpeedFpm < -500.0) return "DESCENT";
  return "CRUISE";
}

}  // namespace

ConfigSpeeds computeConfigSpeeds(double weightLbs) {
  ConfigSpeeds c;
  c.vlsFullKt = computeSpeedLimits(4, false, 35.0, weightLbs, true, false).vlsKt;
  c.vls3Kt = computeSpeedLimits(3, false, 20.0, weightLbs, true, false).vlsKt;
  c.fKt = 1.22 * computeSpeedLimits(2, false, 15.0, weightLbs, false, false).vsKt;
  c.sKt = 1.23 * computeSpeedLimits(0, false, 0.0, weightLbs, false, false).vsKt;
  c.greenDotKt = 2.0 * weightLbs * 0.45359237 / 1000.0 + 85.0;  // the usual 2 x tonnes + 85 rule
  return c;
}

double computeVapp(const Fms& fms, const ConfigSpeeds& speeds, const Airport& airport) {
  if (fms.vappKt > 0.0) return fms.vappKt;
  double correction = 5.0;
  const int rwy = fms.arrRunway >= 0 ? fms.arrRunway : fms.depRunway;
  if (fms.windKt > 0 && rwy >= 0 && rwy < static_cast<int>(airport.runways.size())) {
    const double course = airport.runways[static_cast<size_t>(rwy)].ils.courseMagDeg;
    const double headwind = fms.windKt * std::cos((fms.windDirMag - course) * kDegToRad);
    correction = std::clamp(headwind / 3.0, 5.0, 15.0);
  }
  return std::round((fms.conf3 ? speeds.vls3Kt : speeds.vlsFullKt) + correction);
}

void Mcdu::reset() {
  page_ = Page::Fpln;
  scratch_.clear();
  message_.clear();
  tmpyDep_ = tmpyArr_ = -1;
}

void Mcdu::typeChar(char c) {
  message_.clear();
  if (scratch_ == "CLR") scratch_.clear();
  if (scratch_.size() < kScratchMax) scratch_ += c;
}

void Mcdu::press(int key, McduContext& ctx) {
  if (key >= 0 && key < 128) {
    const char c = static_cast<char>(std::toupper(key));
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '/' || c == ' ' || c == '+' || c == '-')
      typeChar(c);
    return;
  }
  if (key >= A320_MCDU_LSK1L && key < A320_MCDU_LSK1R + 6) {
    const bool right = key >= A320_MCDU_LSK1R;
    lineSelect(key - (right ? A320_MCDU_LSK1R : A320_MCDU_LSK1L), right, ctx);
    return;
  }
  switch (key) {
    case A320_MCDU_CLR:
      // A message goes first, then one character per press; on an empty scratchpad CLR is
      // armed to delete the field at the next line select key.
      if (!message_.empty()) message_.clear();
      else if (scratch_ == "CLR") scratch_.clear();
      else if (!scratch_.empty()) scratch_.pop_back();
      else scratch_ = "CLR";
      return;
    case A320_MCDU_PLUSMINUS:
      message_.clear();
      if (!scratch_.empty() && scratch_.back() == '-') scratch_.back() = '+';
      else typeChar('-');
      return;
    case A320_MCDU_INIT: page_ = Page::Init; break;
    case A320_MCDU_FPLN:
    case A320_MCDU_AIRPORT: page_ = Page::Fpln; break;
    case A320_MCDU_RADNAV: page_ = Page::RadNav; break;
    case A320_MCDU_PERF: page_ = ctx.state.onGround && !ctx.fms.flown ? Page::PerfTakeoff : Page::PerfAppr; break;
    case A320_MCDU_PROG: page_ = Page::Prog; break;
    case A320_MCDU_MENU: page_ = Page::Menu; break;
    case A320_MCDU_NEXTPAGE:
      if (page_ == Page::PerfTakeoff) page_ = Page::PerfAppr;
      else if (page_ == Page::PerfAppr) page_ = Page::PerfTakeoff;
      break;
    case A320_MCDU_DIR:
    case A320_MCDU_DATA:
    case A320_MCDU_FUELPRED:
    case A320_MCDU_SECFPLN:
    case A320_MCDU_ATCCOMM:
      show("NOT AVAILABLE IN SIM");
      break;
    default:
      break;  // slew keys and OVFY: no scrolling pages yet
  }
}

void Mcdu::lineSelect(int line, bool right, McduContext& ctx) {
  switch (page_) {
    case Page::Init: lineSelectInit(line, right, ctx); return;
    case Page::Fpln:
      if (!right && line == 0) page_ = Page::LatRevOrigin;
      else if (!right && (line == 2 || line == 5)) page_ = Page::LatRevDest;
      return;
    case Page::LatRevOrigin:
      if (!right && line == 0) page_ = Page::Departure;
      else if (!right && line == 5) page_ = Page::Fpln;
      return;
    case Page::LatRevDest:
      if (right && line == 0) page_ = Page::Arrival;
      else if (!right && line == 5) page_ = Page::Fpln;
      return;
    case Page::Departure:
    case Page::Arrival: lineSelectRunways(line, right, ctx); return;
    case Page::RadNav: lineSelectRadNav(line, right, ctx); return;
    case Page::PerfTakeoff: lineSelectTakeoff(line, right, ctx); return;
    case Page::PerfAppr: lineSelectAppr(line, right, ctx); return;
    case Page::Menu:
      if (!right && line == 0) page_ = Page::Fpln;
      return;
    case Page::Prog: return;
  }
}

void Mcdu::lineSelectInit(int line, bool right, McduContext& ctx) {
  Fms& f = ctx.fms;
  const bool clr = scratch_ == "CLR";
  if (right || scratch_.empty()) return;
  if (line == 2) {
    if (clr) { f.flightNumber.clear(); accepted(); return; }
    if (scratch_.size() > 8 || scratch_.find_first_of("/. ") != std::string::npos) return show(kFormatError);
    f.flightNumber = scratch_;
    accepted();
  } else if (line == 4) {
    int ci = 0;
    if (clr) { f.costIndex = -1; accepted(); return; }
    if (!parseInt(scratch_, ci)) return show(kFormatError);
    if (ci < 0 || ci > 999) return show(kOutOfRange);
    f.costIndex = ci;
    accepted();
  } else if (line == 5) {
    if (clr) { f.cruiseFl = -1; accepted(); return; }
    std::string e = scratch_.substr(0, scratch_.find('/'));
    if (e.rfind("FL", 0) == 0) e = e.substr(2);
    int fl = 0;
    if (!parseInt(e, fl)) return show(kFormatError);
    if (fl < 10 || fl > 398) return show(kOutOfRange);
    f.cruiseFl = fl;
    accepted();
  } else if (line == 0 || line == 1) {
    show(kNotInDatabase);  // no company routes or alternates in this sim
  } else {
    show(kNotAllowed);
  }
}

void Mcdu::lineSelectRunways(int line, bool right, McduContext& ctx) {
  const bool dep = page_ == Page::Departure;
  int& tmpy = dep ? tmpyDep_ : tmpyArr_;
  const int count = static_cast<int>(ctx.airport.runways.size());
  if (line == 5) {
    if (tmpy >= 0 && right) {  // INSERT*
      if (dep) {
        ctx.fms.depRunway = tmpy;
      } else {
        ctx.fms.arrRunway = tmpy;
        ctx.fms.vappKt = 0.0;
      }
      tmpy = -1;
      page_ = Page::Fpln;
    } else if (tmpy >= 0 && !right) {  // ERASE
      tmpy = -1;
    } else if (!right) {
      page_ = Page::Fpln;  // RETURN
    }
    return;
  }
  if (right) return;
  const int first = dep ? 1 : 2;  // the list starts below the active line (and VIA on ARRIVAL)
  const int index = line - first;
  if (index < 0 || index >= count || index > 2) return;
  if (dep && !ctx.state.onGround) return show(kNotAllowed);
  tmpy = index;
}

void Mcdu::lineSelectRadNav(int line, bool right, McduContext& ctx) {
  Fms& f = ctx.fms;
  const bool clr = scratch_ == "CLR";
  if (scratch_.empty()) return;
  if (right || line == 0 || line == 1 || line == 4) {
    if (clr) { accepted(); return; }
    return show(kNotInDatabase);  // VOR and ADF: none in this sim's data base
  }
  if (line == 2) {
    if (clr) {
      f.manualIls = -1;
      f.manualCrsMagDeg = -1.0;
      accepted();
      return;
    }
    const int i = findIls(ctx.airport, scratch_);
    if (i == -2) return show(kFormatError);
    if (i < 0) return show(kNotInDatabase);
    f.manualIls = i;
    f.manualCrsMagDeg = -1.0;
    accepted();
  } else if (line == 3) {
    if (clr) { f.manualCrsMagDeg = -1.0; accepted(); return; }
    if (f.tunedIls() < 0) return show(kNotAllowed);
    int crs = 0;
    if (!parseInt(scratch_, crs) || scratch_[0] == '-' || scratch_[0] == '+') return show(kFormatError);
    if (crs > 360) return show(kOutOfRange);
    f.manualCrsMagDeg = crs % 360;
    accepted();
  }
}

void Mcdu::lineSelectTakeoff(int line, bool right, McduContext& ctx) {
  Fms& f = ctx.fms;
  const bool clr = scratch_ == "CLR";
  if (right && line == 5) { page_ = Page::PerfAppr; return; }
  if (!ctx.state.onGround || f.flown) {
    if (!scratch_.empty()) show(kNotAllowed);
    return;
  }
  if (!right && line <= 2) {
    double* field = line == 0 ? &f.v1Kt : line == 1 ? &f.vrKt : &f.v2Kt;
    if (clr) { *field = 0.0; accepted(); return; }
    int kt = 0;
    if (scratch_.empty()) {
      // Sim help: an empty scratchpad copies the computed suggestion shown next to the field.
      const SpeedLimits lim = computeSpeedLimits(ctx.state.flapsLever, ctx.state.onePlusF != 0, ctx.state.flapDeg,
                                                 ctx.weightLbs, true, true);
      const TakeoffSpeeds t = computeTakeoffSpeeds(lim.vsKt);
      *field = line == 0 ? t.v1Kt : line == 1 ? t.vrKt : t.v2Kt;
      return;
    }
    if (!parseInt(scratch_, kt)) return show(kFormatError);
    if (kt < 90 || kt > 350) return show(kOutOfRange);
    const double old = *field;
    *field = kt;
    const bool order = (f.v1Kt <= 0.0 || f.vrKt <= 0.0 || f.v1Kt <= f.vrKt) &&
                       (f.vrKt <= 0.0 || f.v2Kt <= 0.0 || f.vrKt <= f.v2Kt) &&
                       (f.v1Kt <= 0.0 || f.v2Kt <= 0.0 || f.v1Kt <= f.v2Kt);
    if (!order) {
      *field = old;
      return show("V1/VR/V2 DISAGREE");
    }
    accepted();
    return;
  }
  if (scratch_.empty()) return;
  if (right && line == 2) {  // FLAPS/THS, e.g. 1/UP1.0
    if (clr) { f.flapsThs.clear(); accepted(); return; }
    const std::string conf = scratch_.substr(0, scratch_.find('/'));
    int c = 0;
    if (!parseInt(conf, c) || c < 1 || c > 3) return show(kFormatError);
    f.flapsThs = scratch_;
    accepted();
  } else if (right && line == 3) {  // FLEX TO TEMP
    if (clr) { f.flexTempC = -100; accepted(); return; }
    int t = 0;
    if (!parseInt(scratch_, t)) return show(kFormatError);
    if (t < -99 || t > 99) return show(kOutOfRange);
    f.flexTempC = t;
    accepted();
  } else if (!right && line == 3) {  // TRANS ALT
    int a = 0;
    if (!parseInt(scratch_, a)) return show(kFormatError);
    if (a < 1000 || a > 39000) return show(kOutOfRange);
    f.transAltFt = a;
    accepted();
  } else if (!right && line == 4) {  // THR RED/ACC
    const size_t slash = scratch_.find('/');
    int red = f.thrRedFt, acc = f.accFt;
    const std::string a = scratch_.substr(0, slash), b = slash == std::string::npos ? "" : scratch_.substr(slash + 1);
    if ((!a.empty() && !parseInt(a, red)) || (!b.empty() && !parseInt(b, acc))) return show(kFormatError);
    if (red < 400 || red > 5000 || acc < red || acc > 5000) return show(kOutOfRange);
    f.thrRedFt = red;
    f.accFt = acc;
    accepted();
  }
}

void Mcdu::lineSelectAppr(int line, bool right, McduContext& ctx) {
  Fms& f = ctx.fms;
  const bool clr = scratch_ == "CLR";
  if (line == 5) {
    if (!right) page_ = Page::PerfTakeoff;  // PREV PHASE
    return;
  }
  if (right && line == 2) { f.conf3 = true; return; }
  if (right && line == 3) { f.conf3 = false; return; }
  if (scratch_.empty()) return;
  if (!right && line == 0) {  // QNH: hPa or inHg
    if (clr) { f.qnhHpa = 0; accepted(); return; }
    double q = 0.0;
    if (!parseNumber(scratch_, q)) return show(kFormatError);
    if (q >= 22.0 && q <= 31.0) q *= 33.8639;
    if (q < 745.0 || q > 1100.0) return show(kOutOfRange);
    f.qnhHpa = static_cast<int>(std::lround(q));
    accepted();
  } else if (!right && line == 1) {  // TEMP
    if (clr) { f.tempC = -100; accepted(); return; }
    int t = 0;
    if (!parseInt(scratch_, t)) return show(kFormatError);
    if (t < -99 || t > 99) return show(kOutOfRange);
    f.tempC = t;
    accepted();
  } else if (!right && line == 2) {  // MAG WIND dir/speed
    if (clr) { f.windDirMag = f.windKt = -1; accepted(); return; }
    const size_t slash = scratch_.find('/');
    int dir = 0, kt = 0;
    if (slash == std::string::npos || !parseInt(scratch_.substr(0, slash), dir) ||
        !parseInt(scratch_.substr(slash + 1), kt))
      return show(kFormatError);
    if (dir < 0 || dir > 360 || kt < 0 || kt > 200) return show(kOutOfRange);
    f.windDirMag = dir % 360;
    f.windKt = kt;
    accepted();
  } else if (!right && line == 3) {  // TRANS FL
    std::string e = scratch_;
    if (e.rfind("FL", 0) == 0) e = e.substr(2);
    int fl = 0;
    if (!parseInt(e, fl)) return show(kFormatError);
    if (fl < 10 || fl > 390) return show(kOutOfRange);
    f.transFl = fl;
    accepted();
  } else if (!right && line == 4) {  // VAPP
    if (clr) { f.vappKt = 0.0; accepted(); return; }
    int v = 0;
    if (!parseInt(scratch_, v)) return show(kFormatError);
    if (v < 90 || v > 350) return show(kOutOfRange);
    f.vappKt = v;
    accepted();
  } else if (right && (line == 0 || line == 1)) {  // MDA (BARO) / DH (RADIO)
    int& field = line == 0 ? f.mdaFt : f.dhFt;
    if (clr) { field = -1; accepted(); return; }
    if (line == 1 && scratch_ == "NO") { field = 0; accepted(); return; }
    int ft = 0;
    if (!parseInt(scratch_, ft)) return show(kFormatError);
    if (ft < 0 || ft > (line == 0 ? 5000 : 1000)) return show(kOutOfRange);
    field = ft;
    (line == 0 ? f.dhFt : f.mdaFt) = -1;  // one or the other
    accepted();
  }
}

void Mcdu::render(const McduContext& ctx, A320McduDisplay& out) const {
  Screen s(out);
  const A320State& st = ctx.state;
  const Fms& f = ctx.fms;
  const std::string icao = ctx.airport.icao;
  const int count = static_cast<int>(ctx.airport.runways.size());
  const ConfigSpeeds cs = computeConfigSpeeds(ctx.weightLbs);

  switch (page_) {
    case Page::Init: {
      s.centre(0, "INIT", A320_MCDU_WHITE);
      s.left(1, " CO RTE", A320_MCDU_WHITE);
      s.right(1, "FROM/TO  ", A320_MCDU_WHITE);
      s.left(2, "----------", A320_MCDU_WHITE);
      s.right(2, icao + "/" + icao, A320_MCDU_CYAN);
      s.left(3, "ALTN/CO RTE", A320_MCDU_WHITE);
      s.left(4, "----/---------", A320_MCDU_WHITE);
      s.left(5, "FLT NBR", A320_MCDU_WHITE);
      if (f.flightNumber.empty()) s.left(6, boxes(8), A320_MCDU_AMBER);
      else s.left(6, f.flightNumber, A320_MCDU_CYAN);
      s.left(7, "LAT", A320_MCDU_WHITE);
      s.right(7, "LONG", A320_MCDU_WHITE);
      const double lat = std::fabs(st.latDeg), lon = std::fabs(st.lonDeg);
      s.left(8, fmt("%02d%04.1f%c", static_cast<int>(lat), std::fmod(lat, 1.0) * 60.0, st.latDeg >= 0 ? 'N' : 'S'),
             A320_MCDU_GREEN);
      s.right(8, fmt("%03d%04.1f%c", static_cast<int>(lon), std::fmod(lon, 1.0) * 60.0, st.lonDeg >= 0 ? 'E' : 'W'),
              A320_MCDU_GREEN);
      s.left(9, "COST INDEX", A320_MCDU_WHITE);
      if (f.costIndex < 0) s.left(10, boxes(3), A320_MCDU_AMBER);
      else s.left(10, fmt("%d", f.costIndex), A320_MCDU_CYAN);
      s.left(11, "CRZ FL/TEMP", A320_MCDU_WHITE);
      s.right(11, "TROPO", A320_MCDU_WHITE);
      if (f.cruiseFl < 0) s.left(12, boxes(5) + "/" + boxes(3) + "`", A320_MCDU_AMBER);
      else s.left(12, fmt("FL%03d/%d`", f.cruiseFl, 15 - f.cruiseFl * 2 / 10), A320_MCDU_CYAN);
      s.right(12, "36090", A320_MCDU_CYAN, 1);
      break;
    }
    case Page::Fpln: {
      s.left(0, " FROM", A320_MCDU_WHITE, 1);
      if (!f.flightNumber.empty()) s.right(0, f.flightNumber + " ", A320_MCDU_WHITE, 1);
      s.right(1, "SPD/ALT   ", A320_MCDU_WHITE);
      const std::string origin = icao + runwayName(ctx, f.depRunway);
      const int elevFt = static_cast<int>(std::lround(ctx.airport.reference.altM / kFtToM));
      s.left(2, origin, A320_MCDU_GREEN);
      s.right(2, fmt(" ---/%6d", elevFt), A320_MCDU_GREEN);
      s.centre(4, "-F-PLN DISCONTINUITY-", A320_MCDU_WHITE);
      const std::string dest = icao + runwayName(ctx, f.arrRunway);
      s.left(6, dest, A320_MCDU_GREEN);
      s.right(6, fmt(" ---/%6d", elevFt), A320_MCDU_GREEN);
      s.centre(8, "---- END OF F-PLN ----", A320_MCDU_WHITE);
      s.centre(10, "-- NO ALTN F-PLN --", A320_MCDU_WHITE);
      s.left(11, " DEST    TIME  DIST EFOB", A320_MCDU_WHITE);
      const int target = f.arrRunway >= 0 ? f.arrRunway : f.depRunway;
      std::string dist = "----";
      if (target >= 0) {
        double brg = 0.0, nm = 0.0;
        toThreshold(ctx, target, brg, nm);
        dist = fmt("%4.0f", nm);
      }
      s.left(12, dest, A320_MCDU_WHITE);
      s.put(12, 9, "----", A320_MCDU_WHITE);
      s.put(12, 14, dist, A320_MCDU_WHITE);
      s.right(12, fmt("%4.1f", st.fuelKg / 1000.0), A320_MCDU_WHITE);
      break;
    }
    case Page::LatRevOrigin:
    case Page::LatRevDest: {
      const bool origin = page_ == Page::LatRevOrigin;
      s.centre(0, "LAT REV FROM " + icao + runwayName(ctx, origin ? f.depRunway : f.arrRunway), A320_MCDU_WHITE);
      if (origin) s.left(2, "<DEPARTURE", A320_MCDU_WHITE);
      else s.right(2, "ARRIVAL>", A320_MCDU_WHITE);
      s.left(12, "<RETURN", A320_MCDU_WHITE);
      break;
    }
    case Page::Departure:
    case Page::Arrival: {
      const bool dep = page_ == Page::Departure;
      const int active = dep ? f.depRunway : f.arrRunway;
      const int tmpy = dep ? tmpyDep_ : tmpyArr_;
      const int shown = tmpy >= 0 ? tmpy : active;
      const A320McduColor lineColor = tmpy >= 0 ? A320_MCDU_YELLOW : A320_MCDU_GREEN;
      s.centre(0, dep ? "DEPARTURES FROM " + icao : "ARRIVAL TO " + icao, A320_MCDU_WHITE);
      if (dep) {
        s.left(1, " RWY      SID     TRANS", A320_MCDU_WHITE);
        s.left(2, shown >= 0 ? " " + runwayName(ctx, shown) : " ---", shown >= 0 ? lineColor : A320_MCDU_WHITE);
        s.put(2, 9, "------  ------", A320_MCDU_WHITE);
        s.centre(3, "AVAILABLE RUNWAYS", A320_MCDU_WHITE);
      } else {
        s.left(1, " APPR     VIA     STAR", A320_MCDU_WHITE);
        s.left(2, shown >= 0 ? " ILS" + runwayName(ctx, shown) : " ------", shown >= 0 ? lineColor : A320_MCDU_WHITE);
        s.put(2, 9, "------  ------", A320_MCDU_WHITE);
        s.left(3, " VIA", A320_MCDU_WHITE);
        s.right(3, "TRANS ", A320_MCDU_WHITE);
        s.left(4, " ------", A320_MCDU_WHITE);
        s.right(4, "------", A320_MCDU_WHITE);
        s.centre(5, "APPROACHES", A320_MCDU_WHITE);
      }
      const int firstLine = dep ? 1 : 2;
      for (int i = 0; i < count && i < 3; ++i) {
        const int row = 2 * (firstLine + i) + 2;
        const Runway& r = ctx.airport.runways[static_cast<size_t>(i)];
        const std::string name = dep ? r.ident : "ILS" + r.ident;
        s.left(row, fmt("<%-6s %5.0fM", name.c_str(), runwayLengthM(ctx, i)), A320_MCDU_CYAN);
        s.left(row + 1, fmt("  CRS%03.0f   %s/%s", r.ils.courseMagDeg, r.ils.ident.c_str(),
                            freqText(r.ils.frequencyMHz).c_str()),
               A320_MCDU_CYAN, 1);
      }
      if (tmpy >= 0) {
        s.left(12, "<ERASE", A320_MCDU_AMBER);
        s.right(12, "INSERT*", A320_MCDU_AMBER);
      } else {
        s.left(12, "<RETURN", A320_MCDU_WHITE);
      }
      break;
    }
    case Page::RadNav: {
      s.centre(0, "RADIO NAV", A320_MCDU_WHITE);
      s.left(1, "VOR1/FREQ", A320_MCDU_WHITE);
      s.right(1, "FREQ/VOR2", A320_MCDU_WHITE);
      s.left(2, "[ ]/[  . ]", A320_MCDU_CYAN);
      s.right(2, "[  . ]/[ ]", A320_MCDU_CYAN);
      s.left(3, "CRS", A320_MCDU_WHITE);
      s.right(3, "CRS", A320_MCDU_WHITE);
      s.left(4, "[ ]", A320_MCDU_CYAN);
      s.right(4, "[ ]", A320_MCDU_CYAN);
      s.left(5, "ILS /FREQ", A320_MCDU_WHITE);
      s.left(7, "CRS", A320_MCDU_WHITE);
      const int ils = f.tunedIls();
      if (ils >= 0 && ils < count) {
        const IlsSpec& spec = ctx.airport.runways[static_cast<size_t>(ils)].ils;
        const int small = f.manualIls >= 0 ? 0 : 1;  // auto-tuned values in small font
        s.left(6, spec.ident + "/" + freqText(spec.frequencyMHz), A320_MCDU_CYAN, small);
        const double crs = f.manualCrsMagDeg >= 0.0 ? f.manualCrsMagDeg : spec.courseMagDeg;
        s.left(8, fmt("%03.0f", crs), A320_MCDU_CYAN, f.manualCrsMagDeg >= 0.0 ? 0 : small);
      } else {
        s.left(6, "[  ]/[   . ]", A320_MCDU_CYAN);
        s.left(8, "[ ]", A320_MCDU_CYAN);
      }
      s.left(9, "ADF1/FREQ", A320_MCDU_WHITE);
      s.right(9, "FREQ/ADF2", A320_MCDU_WHITE);
      s.left(10, "[ ]/[    .]", A320_MCDU_CYAN);
      s.right(10, "[    .]/[ ]", A320_MCDU_CYAN);
      break;
    }
    case Page::PerfTakeoff: {
      const std::string rwy = runwayName(ctx, f.depRunway);
      s.centre(0, rwy.empty() ? "TAKE OFF" : "TAKE OFF RWY " + rwy, A320_MCDU_GREEN);
      const SpeedLimits lim = computeSpeedLimits(st.flapsLever, st.onePlusF != 0, st.flapDeg, ctx.weightLbs, true, true);
      const TakeoffSpeeds sug = computeTakeoffSpeeds(lim.vsKt);
      const char* labels[3] = {"V1", "VR", "V2"};
      const double entered[3] = {f.v1Kt, f.vrKt, f.v2Kt};
      const double suggested[3] = {sug.v1Kt, sug.vrKt, sug.v2Kt};
      const bool locked = !st.onGround || f.flown;
      for (int i = 0; i < 3; ++i) {
        s.left(1 + 2 * i, labels[i], A320_MCDU_WHITE);
        if (entered[i] > 0.0) s.left(2 + 2 * i, fmt("%.0f", entered[i]), locked ? A320_MCDU_GREEN : A320_MCDU_CYAN);
        else s.left(2 + 2 * i, boxes(3), A320_MCDU_AMBER);
        // Sim help: the computed speed, small in the label line; LSK with an empty scratchpad copies it.
        if (entered[i] <= 0.0 && !locked) s.put(1 + 2 * i, 3, fmt("%.0f", suggested[i]), A320_MCDU_CYAN, 1);
      }
      s.put(1, 7, "FLP RETR", A320_MCDU_WHITE);
      s.put(2, 7, fmt("F=%.0f", cs.fKt), A320_MCDU_GREEN);
      s.put(3, 7, "SLT RETR", A320_MCDU_WHITE);
      s.put(4, 7, fmt("S=%.0f", cs.sKt), A320_MCDU_GREEN);
      s.put(5, 7, "CLEAN", A320_MCDU_WHITE);
      s.put(6, 7, fmt("O=%.0f", cs.greenDotKt), A320_MCDU_GREEN);
      s.right(1, "RWY", A320_MCDU_WHITE);
      s.right(2, rwy.empty() ? "---" : rwy, A320_MCDU_GREEN);
      s.right(3, "TO SHIFT", A320_MCDU_WHITE);
      s.right(4, "[M][ ]*", A320_MCDU_CYAN);
      s.right(5, "FLAPS/THS", A320_MCDU_WHITE);
      s.right(6, f.flapsThs.empty() ? "[ ]/[   ]" : f.flapsThs, A320_MCDU_CYAN);
      s.left(7, "TRANS ALT", A320_MCDU_WHITE);
      s.left(8, fmt("%d", f.transAltFt), A320_MCDU_CYAN, 1);
      s.right(7, "FLEX TO TEMP", A320_MCDU_WHITE);
      s.right(8, f.flexTempC > -100 ? fmt("%d`", f.flexTempC) : "[ ]`", A320_MCDU_CYAN);
      s.left(9, "THR RED/ACC", A320_MCDU_WHITE);
      s.left(10, fmt("%d/%d", f.thrRedFt, f.accFt), A320_MCDU_CYAN, 1);
      s.right(9, "ENG OUT ACC", A320_MCDU_WHITE);
      s.right(10, fmt("%d", f.accFt), A320_MCDU_CYAN, 1);
      s.right(11, "NEXT ", A320_MCDU_WHITE);
      s.right(12, "PHASE>", A320_MCDU_WHITE);
      break;
    }
    case Page::PerfAppr: {
      s.centre(0, "APPR", A320_MCDU_GREEN);
      s.left(1, "QNH", A320_MCDU_WHITE);
      if (f.qnhHpa > 0) s.left(2, fmt("%d", f.qnhHpa), A320_MCDU_CYAN);
      else s.left(2, boxes(4), A320_MCDU_AMBER);
      s.put(1, 9, "FINAL", A320_MCDU_WHITE);
      if (f.arrRunway >= 0) s.put(2, 9, "ILS" + runwayName(ctx, f.arrRunway), A320_MCDU_GREEN);
      s.right(1, "BARO", A320_MCDU_WHITE);
      s.right(2, f.mdaFt >= 0 ? fmt("%d", f.mdaFt) : "[   ]", A320_MCDU_CYAN);
      s.left(3, "TEMP", A320_MCDU_WHITE);
      s.left(4, f.tempC > -100 ? fmt("%d`", f.tempC) : "[ ]`", A320_MCDU_CYAN);
      s.put(3, 9, "FLP RETR", A320_MCDU_WHITE);
      s.put(4, 9, fmt("F=%.0f", cs.fKt), A320_MCDU_GREEN);
      s.right(3, "RADIO", A320_MCDU_WHITE);
      s.right(4, f.dhFt > 0 ? fmt("%d", f.dhFt) : f.dhFt == 0 ? "NO" : "[   ]", A320_MCDU_CYAN);
      s.left(5, "MAG WIND", A320_MCDU_WHITE);
      s.left(6, f.windKt >= 0 ? fmt("%03d`/%d", f.windDirMag, f.windKt) : "[ ]`/[ ]", A320_MCDU_CYAN);
      s.put(5, 9, "SLATS", A320_MCDU_WHITE);
      s.put(6, 9, fmt("S=%.0f", cs.sKt), A320_MCDU_GREEN);
      s.right(5, "LDG CONF", A320_MCDU_WHITE);
      if (f.conf3) s.right(6, "CONF3", A320_MCDU_GREEN);
      else s.right(6, "CONF3*", A320_MCDU_CYAN, 1);
      s.left(7, "TRANS FL", A320_MCDU_WHITE);
      s.left(8, fmt("FL%03d", f.transFl), A320_MCDU_CYAN, 1);
      s.put(7, 9, "CLEAN", A320_MCDU_WHITE);
      s.put(8, 9, fmt("O=%.0f", cs.greenDotKt), A320_MCDU_GREEN);
      if (f.conf3) s.right(8, "FULL*", A320_MCDU_CYAN, 1);
      else s.right(8, "FULL", A320_MCDU_GREEN);
      s.left(9, "VAPP", A320_MCDU_WHITE);
      s.put(9, 6, "VLS", A320_MCDU_WHITE);
      s.left(10, fmt("%.0f", computeVapp(f, cs, ctx.airport)), A320_MCDU_CYAN, f.vappKt > 0.0 ? 0 : 1);
      s.put(10, 6, fmt("%.0f", f.conf3 ? cs.vls3Kt : cs.vlsFullKt), A320_MCDU_GREEN);
      s.left(11, " PREV", A320_MCDU_WHITE);
      s.left(12, "<PHASE", A320_MCDU_WHITE);
      break;
    }
    case Page::Prog: {
      s.centre(0, phaseName(ctx), A320_MCDU_GREEN);
      s.left(1, " CRZ", A320_MCDU_WHITE);
      s.left(2, f.cruiseFl > 0 ? fmt(" FL%03d", f.cruiseFl) : " -----", f.cruiseFl > 0 ? A320_MCDU_CYAN : A320_MCDU_WHITE);
      const int target = f.arrRunway >= 0 ? f.arrRunway : f.depRunway;
      s.left(5, " BRG /DIST", A320_MCDU_WHITE);
      s.right(5, "TO   ", A320_MCDU_WHITE);
      if (target >= 0) {
        double brg = 0.0, nm = 0.0;
        toThreshold(ctx, target, brg, nm);
        s.left(6, fmt(" %03.0f`/%5.1f", brg, nm), A320_MCDU_GREEN);
        s.right(6, icao + runwayName(ctx, target), A320_MCDU_CYAN);
      }
      s.left(9, " ILS", A320_MCDU_WHITE);
      const int ils = f.tunedIls();
      if (ils >= 0 && ils < count) {
        const IlsSpec& spec = ctx.airport.runways[static_cast<size_t>(ils)].ils;
        s.left(10, " " + spec.ident + "/" + freqText(spec.frequencyMHz), A320_MCDU_GREEN);
        if (st.ilsFreqMHz > 0.0 && st.dmeNm > 0.0) s.right(10, fmt("DME %.1f", st.dmeNm), A320_MCDU_GREEN);
      } else {
        s.left(10, " NOT TUNED", A320_MCDU_AMBER);
      }
      break;
    }
    case Page::Menu: {
      s.centre(0, "MCDU MENU", A320_MCDU_WHITE);
      s.left(2, "<FMGC", A320_MCDU_GREEN);
      s.left(4, "<ATSU", A320_MCDU_WHITE);
      s.left(6, "<AIDS", A320_MCDU_WHITE);
      s.left(8, "<CFDS", A320_MCDU_WHITE);
      break;
    }
  }

  if (!message_.empty()) s.left(13, message_, A320_MCDU_WHITE);
  else s.left(13, scratch_, A320_MCDU_WHITE);
}

}  // namespace a320

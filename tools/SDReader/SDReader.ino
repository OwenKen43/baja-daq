// SDReader: read and check the hub's log files over USB. No card reader needed.
// Same SD wiring as HubStageD. Open the Serial Monitor at 115200, set line ending to "Newline", then type:
//   ls         list the files on the card and their sizes
//   check 1    analyze LOG0001.CSV: lines per ID, timing gaps, lost frames (counter jumps)
//   head 1     print the first 20 lines of LOG0001.CSV
//   tail 1     print the last 20 lines (use this for the power-pull test)
//   cat 1      print the whole file (long!)
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ---- SD card pins (same as the hub) ----
constexpr int      SD_SCK_PIN  = 5;
constexpr int      SD_MISO_PIN = 21;
constexpr int      SD_MOSI_PIN = 19;
constexpr int      SD_CS_PIN   = 32;
constexpr uint32_t SD_SPI_HZ   = 4000000;

constexpr int MAX_IDS = 16;

// One row of results per CAN ID, filled in by check.
struct IdCheck {
  bool     used;
  uint32_t id;
  uint32_t lines;
  int64_t  firstUs, lastUs;
  int64_t  minGapUs, maxGapUs, sumGapUs;
  uint32_t gaps;
  bool     haveCounter;
  uint32_t lastCounter;
  uint32_t lost;       // counter jumped forward by more than 1
  uint32_t restarts;   // counter went backwards (sender rebooted)
};

static bool mountCard() {
  SD.end();
  if (SD.begin(SD_CS_PIN, SPI, SD_SPI_HZ)) return true;
  Serial.println("ERROR: no SD card (check wiring / insert card), then try again");
  return false;
}

static File openLog(int n) {
  char name[16];
  snprintf(name, sizeof(name), "/LOG%04d.CSV", n);
  File f = SD.open(name, FILE_READ);
  if (!f) Serial.printf("ERROR: can't open %s (type ls to see what's there)\n", name);
  return f;
}

static void listFiles() {
  File root = SD.open("/");
  if (!root) { Serial.println("ERROR: can't open root folder"); return; }
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory()) Serial.printf("  %-14s %10lu bytes\n", f.name(), (unsigned long)f.size());
    f.close();
  }
  root.close();
}

static void catFile(int n) {
  File f = openLog(n);
  if (!f) return;
  uint8_t buf[512];
  while (f.available()) {
    size_t got = f.read(buf, sizeof(buf));   // read in 512-byte blocks: much faster than byte by byte
    Serial.write(buf, got);
  }
  Serial.printf("\n=== end of file (%lu bytes) ===\n", (unsigned long)f.size());
  f.close();
}

static void headFile(int n, int count) {
  File f = openLog(n);
  if (!f) return;
  int lines = 0;
  while (f.available() && lines < count) {
    char c = f.read();
    Serial.write(c);
    if (c == '\n') lines++;
  }
  f.close();
}

static void tailFile(int n, int count) {
  File f = openLog(n);
  if (!f) return;
  // Pass 1: count the lines.
  uint32_t total = 0;
  uint8_t buf[512];
  while (f.available()) {
    size_t got = f.read(buf, sizeof(buf));
    for (size_t i = 0; i < got; i++) if (buf[i] == '\n') total++;
  }
  // Pass 2: skip to the last `count` lines and print them.
  uint32_t skip = (total > (uint32_t)count) ? total - count : 0;
  f.seek(0);
  uint32_t seen = 0;
  while (f.available()) {
    char c = f.read();
    if (seen >= skip) Serial.write(c);
    if (c == '\n') seen++;
  }
  Serial.printf("=== %lu lines in file ===\n", (unsigned long)total);
  f.close();
}

// Analyze one log file: per-ID line counts, timing gaps, and lost frames.
static void checkFile(int n) {
  File f = openLog(n);
  if (!f) return;

  static IdCheck rows[MAX_IDS];
  memset(rows, 0, sizeof(rows));
  uint32_t lineNo = 0, badLines = 0;
  char line[96];

  while (f.available()) {
    // Read one line into `line` (cut off at 95 characters, which never happens with our format).
    size_t len = 0;
    while (f.available()) {
      char c = f.read();
      if (c == '\n') break;
      if (len < sizeof(line) - 1) line[len++] = c;
    }
    line[len] = '\0';
    lineNo++;
    if (lineNo == 1) continue;          // header row: t_us,id,dlc,data
    if (len == 0) continue;

    // Parse "t_us,id,dlc,data"   e.g. 5012345,101,4,000001F3
    long long t;
    unsigned long id;
    unsigned dlc;
    char data[20] = "";
    int fields = sscanf(line, "%lld,%lx,%u,%19s", &t, &id, &dlc, data);
    if (fields < 3) { badLines++; continue; }

    // Find or create this ID's row.
    IdCheck *r = nullptr;
    for (int i = 0; i < MAX_IDS && !r; i++) if (rows[i].used && rows[i].id == id) r = &rows[i];
    for (int i = 0; i < MAX_IDS && !r; i++) if (!rows[i].used) { r = &rows[i]; r->used = true; r->id = id; }
    if (!r) continue;

    // Timing
    if (r->lines == 0) {
      r->firstUs = t;
    } else {
      int64_t gap = t - r->lastUs;
      if (r->gaps == 0 || gap < r->minGapUs) r->minGapUs = gap;
      if (r->gaps == 0 || gap > r->maxGapUs) r->maxGapUs = gap;
      r->sumGapUs += gap;
      r->gaps++;
    }
    r->lastUs = t;
    r->lines++;

    // Counter: our test nodes put a 32-bit counter in the first 4 data bytes (8 hex characters).
    if (dlc >= 4 && strlen(data) >= 8) {
      char hex8[9];
      memcpy(hex8, data, 8);
      hex8[8] = '\0';
      uint32_t counter = strtoul(hex8, nullptr, 16);
      if (r->haveCounter) {
        if (counter > r->lastCounter + 1) r->lost += counter - r->lastCounter - 1;
        else if (counter <= r->lastCounter) r->restarts++;
      }
      r->lastCounter = counter;
      r->haveCounter = true;
    }
  }
  f.close();

  Serial.printf("LOG%04d.CSV: %lu lines (incl. header), %lu unreadable\n",
                n, (unsigned long)lineNo, (unsigned long)badLines);
  for (int i = 0; i < MAX_IDS; i++) {
    const IdCheck &r = rows[i];
    if (!r.used) continue;
    double spanS = (r.lastUs - r.firstUs) / 1e6;
    Serial.printf("  ID 0x%03lX  lines %6lu  span %7.1f s", (unsigned long)r.id, (unsigned long)r.lines, spanS);
    if (r.gaps > 0) {
      Serial.printf("  gap avg %.3f ms  min %.3f  max %.3f", (double)r.sumGapUs / r.gaps / 1000.0,
                    r.minGapUs / 1000.0, r.maxGapUs / 1000.0);
    }
    if (r.haveCounter) {
      Serial.printf("  lost %lu  restarts %lu", (unsigned long)r.lost, (unsigned long)r.restarts);
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);
  delay(1000);
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  Serial.println("SDReader ready. Commands: ls | check N | head N | tail N | cat N");
  if (mountCard()) listFiles();
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  String cmdLine = Serial.readStringUntil('\n');
  cmdLine.trim();
  if (cmdLine.length() == 0) return;

  char cmd[8] = "";
  int n = 0;
  sscanf(cmdLine.c_str(), "%7s %d", cmd, &n);

  if (!mountCard()) return;   // re-mount every time, so you can swap cards between commands
  if      (strcmp(cmd, "ls") == 0)    listFiles();
  else if (strcmp(cmd, "check") == 0) checkFile(n);
  else if (strcmp(cmd, "head") == 0)  headFile(n, 20);
  else if (strcmp(cmd, "tail") == 0)  tailFile(n, 20);
  else if (strcmp(cmd, "cat") == 0)   catFile(n);
  else Serial.println("Commands: ls | check N | head N | tail N | cat N");
}

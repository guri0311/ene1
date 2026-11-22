// SD ライブラリの模擬。PC のディレクトリを SD カードに見立てる
#pragma once
#include "Arduino.h"
#include <memory>

#define FILE_WRITE 1

extern bool g_sdCardPresent;      // カードが挿さっているか
extern std::string g_sdDir;       // SD に見立てたディレクトリ
extern int g_sdFlushCount;

class File : public Print {
  std::shared_ptr<FILE> f;
 public:
  File() {}
  explicit File(FILE *fp) : f(fp, [](FILE *p) { if (p) fclose(p); }) {}
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    if (!f || !g_sdCardPresent) { setWriteError(); return 0; }
    return fwrite(b, 1, n, f.get());
  }
  void flush() {
    if (f && g_sdCardPresent) { fflush(f.get()); g_sdFlushCount++; simAdvanceUs(20000); }
  }
  void close() { f.reset(); }
  explicit operator bool() const { return (bool)f; }
};

// SD ライブラリはファイルを探すたびにルートディレクトリを先頭から読む (1 ブロック 16 エントリ、
// 読み込み 1 回約 1.5ms)。ファイル数に比例して遅くなる様子を再現する
int simSdFileCount();
inline void simSdDirScanCost() { simAdvanceUs((uint64_t)(simSdFileCount() / 16 + 1) * 1500); }

class SDClass {
  bool mounted = false;
 public:
  bool begin(uint8_t) {
    if (!g_sdCardPresent) { simAdvanceUs(2000000); mounted = false; return false; }  // 初期化タイムアウト約 2 秒
    simAdvanceUs(50000);
    mounted = true;
    return true;
  }
  bool exists(const char *name) {
    if (!mounted || !g_sdCardPresent) return false;
    simSdDirScanCost();
    std::string p = g_sdDir + "/" + name;
    FILE *fp = fopen(p.c_str(), "rb");
    if (fp) fclose(fp);
    return fp != nullptr;
  }
  File open(const char *name, int) {
    if (!mounted || !g_sdCardPresent) return File();
    simSdDirScanCost();
    std::string p = g_sdDir + "/" + name;
    return File(fopen(p.c_str(), "ab"));
  }
};
extern SDClass SD;

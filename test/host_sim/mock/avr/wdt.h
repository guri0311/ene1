// ウォッチドッグの模擬: wdt_reset の間隔を記録し、8 秒を超えたら sim.cpp が報告する
#pragma once
#define WDTO_8S 9
void simWdtEnable(int);
void simWdtReset();
#define wdt_enable(x) simWdtEnable(x)
#define wdt_reset() simWdtReset()

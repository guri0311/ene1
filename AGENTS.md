# AGENTS.md — このリポジトリで作業する AI エージェントへ

Antigravity・Claude Code など、どのエージェントもまずこれを読む（人が読んでもよい）。

## 最初に読むもの

1. [docs/HANDOVER.md](docs/HANDOVER.md)：これまでの経緯・決めたこと・車とレースの前提・シミュレーションの結果・**未解決の質問**・**次の作業**（引き継ぎの本体）
2. [README.md](README.md)：使い方、ビルド、テストのしかた
3. 配線を触るなら [docs/WIRING.md](docs/WIRING.md)、動かないときは [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

大会（2026 Ene-1 SUZUKA Challenge KV-40）は **2026 年 12 月 20 日（日）**。それまでは、実車で使える状態を壊さないことを何より優先する。

## ユーザー（チーム）とのやりとり

- **返事は必ず日本語**。むずかしい言葉や略語を避け、やさしい言葉で、結論から書く
- 車・部品・規則についての事実は、チームに確かめてから決める。分からない値は「見積もり」と書き、HANDOVER の「未解決の質問」に足す
- 「マージして」と言われたら：PR を作る → CI の 2 つの job（host-sim と avr-emu）がどちらも ✅ になるのを待つ → マージする。言われていないのに PR を作ったりマージしたりしない
- 作業が終わったら、何をしたか・何を確かめたか（テストの結果）・残っていることを短く伝える

## リポジトリに入れてはいけないもの

- 大会の規則書の PDF（主催者の資料）。必要な数値と条文の番号だけを HANDOVER に書く
- 車の写真、チーム名・ドライバー名、メールアドレスなどの個人の情報
- 作り直せる出力：`tools/suzuka_sim/out/`、`tools/log_viewer/dist/`、ビルドの成果物（.gitignore 済み）

## 構成

```
Ene1PowerMeter/Ene1PowerMeter.ino   ファームウェア（Arduino Nano。1 ファイル）
docs/WIRING.md, docs/wiring.svg     配線図・電線の一覧・規則との対応・電源を入れる前の確認
docs/TROUBLESHOOTING.md             動かないときの調べ方
docs/HANDOVER.md                    引き継ぎ資料
tools/log_viewer/                   テレメトリー画面（index.html + logviewer-core.js、build-single.mjs で 1 ファイル版）
tools/suzuka_sim/                   鈴鹿サーキットの走行シミュレーション（suzuka-sim.js、cli.mjs、digitize_elevation.py）
test/host_sim/                      スケッチを PC 用にコンパイルして動かすテスト（Arduino・Wire・SD などの模擬）
test/avr_emu/                       実機と同じ .hex を ATmega328P エミュレーター（avr8js）で動かすテスト
test/log_viewer/                    ログビューアの計算のテスト（host_sim が作った CSV を読む）
.github/workflows/test.yml          push と PR のたびに上の 3 つを回す CI
```

## コマンド

```bash
bash test/host_sim/run_tests.sh                    # 約 1 分。SIM_SANITIZE=1 を付けるとメモリ破壊・未定義動作の検査付き
bash test/log_viewer/run_tests.sh                  # ログビューアの計算
cd test/avr_emu && npm ci && node run.mjs          # 約 6〜8 分。arduino-cli・dosfstools・mtools が要る（README の「テスト」）
arduino-cli compile --fqbn arduino:avr:nano Ene1PowerMeter
node tools/suzuka_sim/cli.mjs [--firmware] [--set cdA=0.04]   # 鈴鹿シミュ。CSV は tools/suzuka_sim/out/
node tools/log_viewer/build-single.mjs             # テレメトリー画面の 1 ファイル版（tools/log_viewer/dist/）
```

CI で固定しているバージョン：arduino-cli 1.1.1、arduino:avr 1.8.8、SD 1.3.0、Node 22（avr-emu）。

## 変更するときの決まり

### ファームウェア（Ene1PowerMeter/Ene1PowerMeter.ino）

- **いちばん大事な要件は「何が起きても、積算 mAh の記録と、いまの電流の表示だけは続ける」**。INA228・LCD・SD のどれが壊れても止まらない作りを崩さない（例：INA228 が読めなくても止まらず、最後の積算を `LAST Q` で出す。電源が切れても EEPROM から続きを数える）
- Arduino Nano の RAM は 2KB。グローバル変数で 1,244 バイト使い、エミュレーターで測った空きの最小は 335 バイトしかない。文字列は `F()` に入れ、配列や `String` を増やさない。変えたら RAM の空き（シリアルの `# RAM min free`、エミュレーターのログ）を確かめる
- Arduino IDE は関数のプロトタイプを自動で作るので、構造体の定義は、それを使うどの関数よりも上に置く（置かないと `'SavedRecord' was not declared` のようなエラーになる）
- EEPROM の記録の形（20 バイト × 50 スロット、CRC16）を変えたら `EE_MAGIC` を変える（いまは 0xE1A2。0xE1A1 は距離の入る前の 16 バイトの形）
- CSV の列を変えたら、`tools/log_viewer/logviewer-core.js` と `test/log_viewer` も合わせる。古い CSV（速さ・距離の列がないもの）も読めるままにする
- ピン（A4/A5 I2C、D2 ボタン、D3 速度センサー、D10〜D13 SD）を変えたら、README・docs/WIRING.md・docs/wiring.svg も直す
- 変えたら host_sim と avr_emu の両方を回す。テストを飛ばしたり消したりして通さない

### 鈴鹿シミュレーション（tools/suzuka_sim/suzuka-sim.js）

- UMD 形式で、Node（cli.mjs）とブラウザ（ログビューアの「鈴鹿シミュ」ボタン）の両方から読む。両方で動くことを確かめる
- 値には「チームから聞いた値」「資料から読んだ値」「見積もり」がある。`DEFAULTS` のコメントにどれかが分かるように書き、HANDOVER の前提の表も直す
- モデルを変えて結果が変わったら、README と HANDOVER に書いた数字も書き直す

### ログビューア（tools/log_viewer）

- サーバーなしで、ファイルを開くだけで動くこと。データを外へ送らないこと（読むのはフォントだけ）
- `index.html` に `<script src=...>` を足したら、`build-single.mjs` の一覧にも足す

### 文書・コメント・コミット

- 日本語で、やさしい言葉で書く（いまの文に合わせる）
- コミットのメッセージは日本語 1 行で「〜する」「〜を追加」の形。鈴鹿シミュの変更は先頭に「鈴鹿シミュ: 」を付けてきた
- 作業ごとにブランチを切る（これまでは `claude/ene1-<内容>`）。main に直接 push しない

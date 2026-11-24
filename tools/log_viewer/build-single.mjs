// ログビューアを 1 つの HTML ファイルにまとめる (配布用)
//   node tools/log_viewer/build-single.mjs [出力先]
//   出力先の初期値: tools/log_viewer/dist/ene1-telemetry.html
//
// index.html が読む logviewer-core.js と ../suzuka_sim/suzuka-sim.js を <script> の中に入れる。
// できたファイルは、それだけをブラウザで開けば動く (スマホやタブレットで、別ファイルを読めない
// 開き方をしても動く)。フォントは Google Fonts から読むが、オフラインでも端末のフォントで動く。
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const out = path.resolve(process.argv[2] || path.join(HERE, 'dist', 'ene1-telemetry.html'));

const read = (p) => fs.readFileSync(path.join(HERE, p), 'utf8');
let html = read('index.html');
const scripts = [
  ['<script src="logviewer-core.js"></script>', read('logviewer-core.js')],
  ['<script src="../suzuka_sim/suzuka-sim.js"></script>', read('../suzuka_sim/suzuka-sim.js')],
];
for (const [tag, js] of scripts) {
  if (!html.includes(tag)) { console.error(`index.html に ${tag} が見つからない`); process.exit(1); }
  if (/<\/script/i.test(js)) { console.error(`${tag} の中身に </script があり、HTML に入れられない`); process.exit(1); }
  html = html.replace(tag, `<script>\n${js}\n</script>`);
}
html = html.replace(
  'このファイルと logviewer-core.js を同じフォルダに置き、ブラウザで開くだけで使える。',
  'このファイル 1 つで動く (logviewer-core.js と suzuka-sim.js を中に入れた配布版。tools/log_viewer/build-single.mjs で作る)。',
);
fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out, html);
console.log(`${path.relative(process.cwd(), out)} (${Math.round(html.length / 1024)} KB)`);

// 配布するときに添える使い方 (zip にするなら、この 2 つを入れる)
const howto = path.join(path.dirname(out), '使い方.txt');
fs.writeFileSync(howto, `Ene-1 テレメトリー

${path.basename(out)} をブラウザ (Chrome・Edge・Safari・Firefox) で開くと使えます。
インターネットにつながっていなくても動きます (フォントだけ端末のものになります)。
データは端末の外に送りません。

使い方
- 「CSV を開く」で、電力計の SD カードの LOG000.CSV などをまとめて選ぶ (画面にドラッグしてもよい)
  Nano のリセットで分かれたファイルは、1 つの走行につなぎます
- 上の BUDGET (使える量 mAh) と RACE (レース時間 分) を入れると、予算との差を出します
- ▶ で走行を再生し、電流計・電圧・積算・予算との差を、その時点の値で見られます
- グラフを横にドラッグすると範囲を選べます。ダブルクリックで全体に戻ります
- 「鈴鹿シミュ」で、鈴鹿サーキットの走行シミュレーションを表示します
- 「サンプル」で作りもののデータを表示します

元のファイルはリポジトリの tools/log_viewer/ にあります。
`);
console.log(path.relative(process.cwd(), howto));

# サツマイモモータドライバー用ファームウェア

## 概要
本リポジトリは、サツマイモモータドライバー用のファームウェアを提供します。  
通信方式は **CAN** を使用し、CAN ID は起動時のID決定ダイヤルの位置によって決定されます。  

---

## 仕様
- **通信方法**：CAN
- **CAN ID**：1〜8（起動時のID決定ダイヤルで設定）
- **LED動作**：
  | 状態 | LED |
  | ---- | --- |
  | CAN確立 & ID一致 | 緑点灯 |
  | CAN確立 & ID不一致 | 赤点灯 |
  | CAN未確立 | 消灯 |

---

## ファームウェア
ビルド済みバイナリはこちらからダウンロードできます：  
[**satumaimo.bin**](https://github.com/omuct-robotclub/satumaimo-farm/blob/main/STM32CubeIDE/Debug/satumaimo.bin)

---

## 書き込み方法
1. サツマイモマザーとPCをSTlinkで接続
2. エクスプローラのSTlink画面にバイナリファイルを配置

---

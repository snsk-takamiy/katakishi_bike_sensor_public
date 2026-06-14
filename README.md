# Katakishi Bike Sensor Firmware

M5Stack AtomS3 で自転車の傾きと車輪回転を取得し、PC へ USB シリアル CSV と Wi-Fi UDP/OSC で送信する PlatformIO / Arduino ファームウェアです。

## 概要

- 傾き: AtomS3 内蔵 MPU から `roll` / `pitch` を取得し、左右/中心の `lean_state` に変換します。
- 車輪速度: ホールセンサーの磁石通過を GPIO 入力で検出し、速度 `speed_kmh` を計算します。
- 出力: USB CDC シリアルへ CSV、Wi-Fi 経由で OSC を送信します。
- 表示: AtomS3 LCD に傾き、速度、ホール状態、IP アドレス、接続状態を表示します。

## 接続図

![チャリ_ケイデンスセンサー取説](./img/チャリ_ケイデンスセンサー取説.jpg)

## ピンアサイン

| 用途 | AtomS3 側 | センサー側 | 備考 |
| --- | --- | --- | --- |
| ホールセンサー信号 | `G2` / `GPIO2` | `signal` | ファームウェアでは `HALL_PIN = 2` |
| 電源 | `3.3V` 推奨 | `VCC` | 画像の例は `5V` 接続。GPIO へ 5V 信号が出るモジュールは直結しない |
| GND | `GND` | `GND` | AtomS3 とセンサーの GND を共通化 |
| 傾き | 内蔵 MPU | - | 外部配線なし |
| 表示 | 内蔵 LCD | - | 外部配線なし |

ホール入力はデジタル入力として扱います。磁石の向きやセンサー個体差で反応が逆になる場合は、`src/main.cpp` の `HALL_ACTIVE_LEVEL` を調整してください。

## ネットワーク設定

| 環境 | 役割 | IP アドレス | OSC アドレス |
| --- | --- | --- | --- |
| `bike1_ap` | SoftAP | `192.168.4.1` | `/bike1/tilt`, `/bike1/wheel` |
| `bike2_sta` | STA | `192.168.4.2` | `/bike2/tilt`, `/bike2/wheel` |
| `bike3_sta` | STA | `192.168.4.3` | `/bike3/tilt`, `/bike3/wheel` |

- AP SSID: `katakishi-bike-sensor`
- AP password: `bike-sensor`
- OSC 送信先: `192.168.4.10:9000`

## シリアル出力

起動時にヘッダーを出力し、以降は CSV で送信します。

```csv
roll,lean_state,speed_kmh,speed_window_pulses,hall_state,hall_active
```

## ビルド

```sh
/Users/takamiy/.platformio/penv/bin/pio run
/Users/takamiy/.platformio/penv/bin/pio run -e bike1_ap
/Users/takamiy/.platformio/penv/bin/pio run -e bike2_sta
/Users/takamiy/.platformio/penv/bin/pio run -e bike3_sta
```

アップロードとシリアルモニターは、接続先デバイスを確認してから実行してください。

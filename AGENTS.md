# AGENTS.md

## Repository Instructions

- 日本語で回答してください。

## Project Context

- Target: 自転車の傾きと車輪速度をセンシングし、PC に USB シリアル通信と OSC 通信で送るファームウェア。
- Hardware: M5Stack AtomS3、内蔵 MPU、49E リニアホールセンサー、車輪側磁石。
- Framework: PlatformIO / Arduino.
- Primary environments: `bike1_ap`, `bike2_sta`, `bike3_sta`.

## Firmware Design

- Architecture: `millis()` ベースの周期処理を基本にし、センサー取得、表示更新、USB シリアル送信、OSC 送信を分離する。
- Main modules:
  - `SensorState`: roll、pitch、左右/中心状態、速度 km/h、パルス数、速度計算窓内パルス数、ホールデジタル状態を保持する。
  - `ImuReader`: AtomS3 内蔵 MPU を初期化し、`M5.Imu.update()` 後に roll/pitch を取得する。
  - `WheelSensor`: 49E のデジタル入力を読み、エッジ検出で磁石通過を検出して速度 km/h を算出する。
  - `SerialReporter`: USB シリアルへ必要なセンサー値を CSV で出力する。
  - `OscReporter`: SoftAP と UDP/OSC 送信を担当する。
  - `DisplayView`: AtomS3 の画面に傾きと速度をグラフィカルに表示する。
- Timing/concurrency assumptions:
  - 速度算出周期は 1000ms。
  - 画面更新と通信送信は独立した周期にし、長い `delay()` は避ける。
  - 49E はデジタル入力のポーリングで扱い、割り込み前提にはしない。
- Persistence/configuration:
  - マイコンをアクセスポイント化するため、開発初期は SoftAP の SSID/パスワードをコード上に平文で保持してよい。
  - 本番用の秘密情報を追加する場合は、後で `include/secrets.h` などに分離する。

## Hardware Interfaces

- Pin map:
  - 49E ホールセンサー出力: `G2` / `GPIO2`。
  - 49E 電源: 3.3V 駆動を推奨。5V 出力モジュールを GPIO に直結しない。
  - 傾き: AtomS3 内蔵 MPU を使用する。
  - 表示: AtomS3 内蔵 LCD を使用する。
- Buses/protocols:
  - 内蔵 MPU と LCD は M5Stack ライブラリの AtomS3 対応 API を優先する。
  - PC への有線出力は USB CDC シリアル。
  - PC への無線出力は Wi-Fi SoftAP/STA + UDP OSC。
- Electrical constraints:
  - ホール入力は `INPUT_PULLUP`、磁石検出時 LOW を初期前提にする。
  - 実機で反応が逆の場合は `HALL_ACTIVE_LEVEL` を `HIGH` に変更する。
  - デジタル入力は 50ms のソフトウェアデバウンスを入れ、状態が安定してからエッジとして扱う。

## Connectivity

- Network/protocol choices:
  - PlatformIO env で AP 機能あり 1 台、AP 機能なし 2 台を切り替えてビルドする。
  - `bike1_ap`: AtomS3 を SoftAP として起動し、IP は `192.168.4.1`。
  - `bike2_sta`: AtomS3 を STA として `katakishi-bike-sensor` へ接続し、固定 IP は `192.168.4.2`。
  - `bike3_sta`: AtomS3 を STA として `katakishi-bike-sensor` へ接続し、固定 IP は `192.168.4.3`。
  - AP SSID: `katakishi-bike-sensor`。
  - AP password: `bike-sensor`。
  - 受け取り PC の IP: `192.168.4.10`。
  - OSC 送信先: `192.168.4.10:9000`。
  - STA モードの接続完了時は、シリアルに `local_ip` と OSC 送信先を出力する。
- OSC addresses:
  - `bike1_ap`: `/bike1/tilt`, `/bike1/wheel`。
  - `bike2_sta`: `/bike2/tilt`, `/bike2/wheel`。
  - `bike3_sta`: `/bike3/tilt`, `/bike3/wheel`。
  - `tilt`: `roll`, `lean_state` を送る。
  - `wheel`: `speed_kmh`, `hall_state` を送る。
- Lean state:
  - `-1`: 左。
  - `0`: 中心。
  - `1`: 右。
  - `roll` は自転車の左右傾き、`pitch` は前後傾きとして扱う。
  - 初期閾値は中心 `abs(roll) < 5deg`、左右判定 `abs(roll) >= 10deg` とし、間をヒステリシス領域にする。
- Provisioning/secrets policy:
  - 開発初期は SoftAP の認証情報をコードに置いてよい。
  - 外部 Wi-Fi 接続や個人認証情報を扱う場合はコミットしない。
- OTA/update policy: 現時点では OTA は対象外。

## Display

- 画面には左右/中心状態、roll、速度 km/h、ホールデジタル状態を表示する。pitch は表示しない。
- 傾きは水平器風のバーまたは円で、速度はバーまたはゲージでグラフィカルに示す。
- 画面描画はセンサー取得や通信をブロックしない周期で更新する。

## Build And Test

- Build all: `/Users/takamiy/.platformio/penv/bin/pio run`
- Build AP device: `/Users/takamiy/.platformio/penv/bin/pio run -e bike1_ap`
- Build STA device 2: `/Users/takamiy/.platformio/penv/bin/pio run -e bike2_sta`
- Build STA device 3: `/Users/takamiy/.platformio/penv/bin/pio run -e bike3_sta`
- Test: 現時点では自動テストなし。純粋な閾値判定や速度計算を切り出した場合は unit test を追加する。
- Upload: 明示的な指示があるまで実行しない。
- Serial monitor: 明示的な指示があるまで実行しない。

## Current Plan

- M5AtomS3 内蔵 MPU で roll/pitch を取得し、左右/中心の `lean_state` に変換する。
- 49E ホールセンサーを `G2` / `GPIO2` に接続し、デジタル入力のエッジ検出で磁石通過を検出する。
- タイヤ周長は 2.0m とし、1000ms ごとのパルス数から速度 km/h を算出する。
- 速度計算は `speed_kmh = pulse_delta * 2.0m / elapsed_seconds * 3.6` とする。
- 速度計算後は速度計算用の窓内パルス数を 0 に戻す。累積パルス数は別に保持する。
- USB シリアルと OSC の両方で PC に値を送る。
- USB シリアル出力は CSV とし、ヘッダー行を起動時に出力する。CSV は `roll,lean_state,speed_kmh,speed_window_pulses,hall_state,hall_active` とする。
- `bike1_ap` は SoftAP を起動し、`bike2_sta` と `bike3_sta` はその AP へ接続する。
- 全台が PC `192.168.4.10:9000` へ UDP OSC を送信する。未接続時は UDP 送信をスキップする。
- OSC は env ごとに `/bike1/tilt` と `/bike1/wheel`、`/bike2/tilt` と `/bike2/wheel`、`/bike3/tilt` と `/bike3/wheel` を使う。
- AtomS3 の画面には、傾き状態と速度をグラフィカルに表示する。
- 49E の初期判定値は、`INPUT_PULLUP`、検出時 LOW、デバウンス 50ms とする。

## Open Questions

- PC 側 OSC 送信先 `192.168.4.10` を固定 IP のまま運用するか、接続クライアント検出や設定 UI を追加するか。
- 49E のデジタル出力が実機で反転する場合は、`HALL_ACTIVE_LEVEL` を調整する。
- 傾きの左右方向が実機の取り付け向きに対して roll の正負どちらになるか。逆の場合は `leftRight` の符号を反転する。

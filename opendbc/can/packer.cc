#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "opendbc/can/common.h"

/*
CANメッセージの生データバイト列 (std::vector<uint8_t> msg) に、
特定の信号 (Signal) の値 (ival) をビット単位で埋め込む 処理を実施
*/
void set_value(std::vector<uint8_t> &msg, const Signal &sig, int64_t ival) {
  // 書き込み開始位置のバイトインデックス（LSB基準）
  int i = sig.lsb / 8;

  // SG_ ENGINE_RPM : 23|16@0+ (1,0) [0|15000] "rpm" EON　  の場合
  // sig.size = 16　が入る
  int bits = sig.size;

  /*
  1ULL = Unsigned Logn Long = 1を64bitで表現
  (1ULL << 3) => 0b1000 (2進数) => 8 (10進数)
  
  a &= b; => a に b をビットANDして、その結果を a に代入する

  ival &= ((1ULL << 3) - 1);　の場合、 b = 0b0111 = 7
  ival　の　下位3桁のみを抽出する　という意味
  */
  if (sig.size < 64) {
    ival &= ((1ULL << sig.size) - 1);
  }

  /*
  ival の値を、sig.lsb（最下位ビット）から sig.size ビット分だけ、msg[i] に埋め込む
    msg[i] は CAN メッセージのバイト列（std::vector<uint8_t>）
    ival は埋め込む整数値
    sig には以下が入っている：
      sig.lsb: ビット列の書き始め位置（Least Significant Bit）
      sig.size: 信号のビット幅（1～64ビット）
      sig.is_little_endian: ビット順序（IntelかMotorolaか）

  詳細の解説 : Littel Endian
    i = 5 / 8 = 0;  // msg[]のindex
    sig.lsb = 5;    // 最下位ビットは "bit5"
    sig.size = 11;  // 合計 11ビット分を使用
     ↓
    msg[i] = msg[0]　が対象
    開始ビット = 5
    終了ビット = 5 + 11 - 1 = 15

               7 6 5 4 3 2 1 0
    msg[0] : [ x x x . . . . . ]  ← bit5〜7 に最初の3bit（shift = 5）
    msg[1] : [ x x x x x x x x ] ← 残り8bitをフルで格納（shift = 0）
    msg[2] : [ . . . . . . . . ] ← 使わない

    i=0
      bits  = 11
      shift = 5
      size  = 8-5 = 3
      
      1: クリア（ゼロにする）対象のビットを計算：
        msg[i] &= ~(((1ULL << size) - 1) << shift);

        1. (1ULL << 3) = 0b0000 1000 = 8
        2. 8 - 1 = 7 = 0b0000 0111
        3. 0b0000 0111 << 5 = 0b1110 0000
        4. ~0b1110 0000     = 0b0001 1111
        5. msg[0] &= 0b0001 1111;
           → msg[0] の bit5〜7がゼロになる（それ以外はそのまま）

      2: 値の一部を埋め込む: 
        msg[i] |= (ival & ((1ULL << size) - 1)) << shift;

        1. ival = 0b010110100011
        2. ival & 0b0000 0111 
          = ival & 0x07 
          = 0x07がマスクとなって、ivalの下位3bitを抽出
          = 0b011
        3. 0b011 << 5 = 0b0110 0000
          = shiftして msg[0]の bit5〜7　に　ivalの下位3bitを入力

    i=1
      bits  = 8     // 残りビット数
      shift = 0     // 下位ビットから詰めるので、バイトの先頭から
      size  = 8     // 今回はフル8bit使う

      1: クリア（ゼロにする）対象のビットを計算：
        msg[i] &= ~(((1ULL << size) - 1) << shift);

        1. (1ULL << 8) = 0b1_0000_0000 = 256
        2. 256 - 1 = 255 = 0b1111_1111
        3. 0b1111_1111 << 0 = 0b1111_1111
        4. ~0b1111_1111     = 0b0000_0000
        5. msg[1] &= 0b0000_0000;
           → msg[1] のすべてのビットがゼロになる（完全クリア）

      2: 値の一部を埋め込む: 
        msg[i] |= (ival & ((1ULL << size) - 1)) << shift;

        1. ival  = 0b010110100 （i=0でshift済み）
        2. ival & 0xFF = 0b01011010 = 0x5A
        3. 0x5A << 0 = 0x5A
        3. msg[1] |= 0x5A
           → msg[1] に ival の上位8bitをそのまま格納

  */
  while (i >= 0 && i < msg.size() && bits > 0) {
    int shift = (int)(sig.lsb / 8) == i ? sig.lsb % 8 : 0;
    int size = std::min(bits, 8 - shift);

    // msg[i]の入力対象ビットだけ 0 にする
    msg[i] &= ~(((1ULL << size) - 1) << shift);

    // ivalから入力対象のデータを抽出し、上記で 0 にしたmsg[i]の場所に
    // ival のデータを入力する
    msg[i] |= (ival & ((1ULL << size) - 1)) << shift;

    // 入力が完了したsize分、データ長を減じる
    bits -= size;

    // 入力済みのビットは不要なため、右にシフトして切り捨てる
    ival >>= size;

    /*
    Little Endian の場合： i = 5 → 6 → 7 と 右方向（インデックス増加）
    Big Endian の場合：    i = 5 → 4 → 3... のように 左方向（インデックス減少）
    */
    i = sig.is_little_endian ? i+1 : i-1;
  }
}

CANPacker::CANPacker(const std::string& dbc_name) {
  // DBCファイル名をもとに構造体 DBC* を取得（内部的には *.dbc をパース済みのデータ構造）
  dbc = dbc_lookup(dbc_name);

  // dbc_lookup() が失敗（nullptrを返す）しないことを保証。
  // 例：dbc_name が存在しない場合は assert により即時停止。
  assert(dbc);

  // すべてのメッセージ定義（BO_）についてループ
  for (const auto& msg : dbc->msgs) {
    // そのメッセージに含まれる全シグナル（SG_）をループ
    for (const auto& sig : msg.sigs) {
      // [CAN ID][シグナル名] = シグナル定義（ビット位置、長さ、エンディアンなど）
      signal_lookup[msg.address][sig.name] = sig;
    }
  }
}

/*
DBC定義をもとに、複数の信号値（SignalPackValue）を
1つのCANメッセージバイト列（std::vector<uint8_t>）にパック（エンコード）
CANPacker::pack() は「CAN送信の直前に、各信号値をCANフレームにエンコードする」ための関数で、
OpenPilotやCANシミュレータなどで車に信号を書き込むときの最終ステップ

詳細
  1. CAN ID（address） に対応する CANメッセージの構造（BO_） を dbc から探す。

- 与えられた信号リスト（signals）に含まれる各信号について：
  2. 対応する Signal 構造体（SG_）を取得。
  3. 実数値を 符号付き整数（int64_t） に変換。
  4. set_value() を使って対応する ビット列に埋め込む（bit packing）。

- 追加で次を処理：
  5. DBC定義に COUNTER シグナルがあれば、自動でインクリメントし埋め込む。
  6. CHECKSUM シグナルがあれば、計算して埋め込む。

出力
  - 最終的に、信号値をすべて埋め込んだ CANメッセージのバイト列（8バイトなど） を返す。
*/
std::vector<uint8_t> CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &signals) {
  // CAN ID で　メッセージを検索
  auto msg_it = dbc->addr_to_msg.find(address);

  // 見つからない場合の処理
  if (msg_it == dbc->addr_to_msg.end()) {
    LOGE("undefined address %d", address);
    return {};
  }

  // msg_it->second->size = CANメッセージのバイトサイズ
  // そのサイズ分だけ uint8_t の配列を 0 初期化で確保 → ret に値を書き込んでいく。
  std::vector<uint8_t> ret(msg_it->second->size, 0);

  // カウンタ信号（COUNTER）がユーザー入力で与えられたかどうかのフラグ。
  // もし与えられていなければ、後で自動でインクリメント値をセットする。
  // set all values for all given signal/value pairs
  bool counter_set = false;

  // signals = ユーザーがこのメッセージで設定したい各信号（SG_）名と、その実数値
  for (const auto& sigval : signals) {
    // DBCに登録された [CAN ID][Signal名] から信号定義を探す
    auto sig_it = signal_lookup[address].find(sigval.name);

    // つからなければ警告出してスキップ（処理は続行）
    if (sig_it == signal_lookup[address].end()) {
      // TODO: do something more here. invalid flag like CANParser?
      LOGE("undefined signal %s - %d\n", sigval.name.c_str(), address);
      continue;
    }

    // 信号定義（Signal 構造体）への参照を取得
    const auto &sig = sig_it->second;

    /*
    実数値（sigval.value）から 内部整数値（ival）へ変換
    DBCの変換式は以下：
        physical_value = raw_value * factor + offset
    よってその逆：
        raw_value = (physical_value - offset) / factor
    */
    int64_t ival = (int64_t)(round((sigval.value - sig.offset) / sig.factor));

    /*
    負数の場合、2の補数表現に変換する処理
    CAN信号では、負の値を送る場合「2の補数（two’s complement）」としてビットに埋め込む必要がある
    例：
      11ビットのフィールドに ival = -3 を格納したい場合：
      最大値（符号なし） = 2^11 = 2048
      -3 を 2の補数表現にするには：
      2^11 + (-3) = 2048 - 3 = 2045

    2の補数の作り方-1(-3を対象)
      1. +3のビット列を作る
      2. ビットを反転
      3. +1する

    2の補数の作り方-2(-3を対象)
      1. Nビットの最大値 - 3

    上記の例では、2045が情報として送られる.
    11ビットの場合
      singed : -1024 ～ +1023
      unsigned : 0〜2047
    であるため、dbcの符号によって
    入力値 = 2045　は
      signedの場合、最大値を超えているので自動的に負数と認識 --> -3
      unsignedの場合、そのまま2045として認識
    となる
    */
    if (ival < 0) {
      ival = (1ULL << sig.size) + ival;
    }
    set_value(ret, sig, ival);

    // 信号が "COUNTER" である場合に、カウンター値を記録する
    // FIXME: Type is only assigned if DBC has a ChecksumState
    if (sig.type == COUNTER || sig.name == "COUNTER") {
      counters[address] = sigval.value;
      counter_set = true;
    }
  }

  // set message counter
  /*
  CAN信号（SG_）の中から「カウンター（COUNTER）」のシグナルを探している

  std::find_if(begin, end, predicate);
    std::find_if は、C++標準ライブラリの <algorithm> に含まれる関数で、「条件を満たす最初の要素」を探すための関数
    predicate：条件を表すラムダ式や関数（戻り値が true ならヒット）

  ラムダ式の意味
    for (const auto& pair : signal_lookup[address]) {
      std::string name = pair.first;
      Signal sig      = pair.second;
    }    
  */
  auto sig_it_counter = std::find_if(signal_lookup[address].begin(), signal_lookup[address].end(), [](const auto& pair) {
    return pair.second.type == COUNTER || pair.first == "COUNTER";
  });

  /*
  ifの条件
    counter_set == false: signals 引数の中に "COUNTER" がなかった場合
    sig_it_counter が見つかった場合
  */
  if (!counter_set && sig_it_counter != signal_lookup[address].end()) {
    const auto& sig = sig_it_counter->second;

    // この CAN メッセージアドレスのカウンタがまだ初期化されていない場合、0 で初期化
    if (counters.find(address) == counters.end()) {
      counters[address] = 0;
    }

    // メッセージバイト列 ret の中に、COUNTER シグナルの位置に現在のカウント値をセットする
    set_value(ret, sig, counters[address]);

    /*
    カウント値を1つ進める
    sig.size ビットのカウンタとして循環（オーバーフローを防ぐ）
    例：サイズが 2 ビット の場合 (1 << 2) = 4 で割ったあまりにする => 0〜3 で繰り返す
    */
    counters[address] = (counters[address] + 1) % (1 << sig.size);
  }

  // set message checksum
  /*
  CANメッセージには信頼性のために「チェックサム」を付ける場合があります。
  このコードでは、DBCファイルに定義された CHECKSUM タイプのシグナルがあれば、
  自動的にその値を計算して埋め込んでいます。

  ラムダ式
    for (auto it = signal_map.begin(); it != signal_map.end(); ++it) {
      if (it->second.type > COUNTER) {
        sig_it_checksum = it;
        break;  // 最初に見つかった1つで十分
      }
    }

  pair.second.type > COUNTER　について
  pair.second.type　の構造体は SignalType
  SignalTypeが　XX_CHECKSUMの場合、
  (sig_it_checksum != signal_lookup[address].end())　が Trueになる

    enum SignalType {
      DEFAULT,
      COUNTER,
      HONDA_CHECKSUM,
      TOYOTA_CHECKSUM,
      PEDAL_CHECKSUM,
      VOLKSWAGEN_MQB_MEB_CHECKSUM,
      XOR_CHECKSUM,
      SUBARU_CHECKSUM,
      CHRYSLER_CHECKSUM,
      HKG_CAN_FD_CHECKSUM,
      FCA_GIORGIO_CHECKSUM,
      TESLA_CHECKSUM,
    };


  */
  auto sig_it_checksum = std::find_if(signal_lookup[address].begin(), signal_lookup[address].end(), [](const auto& pair) {
    return pair.second.type > COUNTER;
  });

  //SignalTypeが　XX_CHECKSUMの場合、
  if (sig_it_checksum != signal_lookup[address].end()) {
    const auto &sig = sig_it_checksum->second;

    // チェックサムの計算を実行し、成功したら、値を登録する
    // ret : CANデータ
    if (sig.calc_checksum != nullptr) {
      unsigned int checksum = sig.calc_checksum(address, sig, ret);
      set_value(ret, sig, checksum);
    }
  }

  return ret;
}

// This function has a definition in common.h and is used in PlotJuggler
const Msg* CANPacker::lookup_message(uint32_t address) {
  return dbc->addr_to_msg.at(address);
}

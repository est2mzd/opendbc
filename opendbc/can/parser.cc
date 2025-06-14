#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "opendbc/can/common.h"

/*
この関数は、CANメッセージのバイト列（msg）から、sig で指定された ビット範囲 を読み取り、
そのビット列を 整数（raw value）として復元する処理を行います。

ret に最終的に格納されるもの
  ret には、信号の "生の数値"（補正やスケーリングされる前の値）が入ります。

単位変換（km/h や Nmなど）やオフセットはまだ適用されていない、「そのままのビット列から導出された整数」です。
*/
int64_t get_raw_value(const std::vector<uint8_t> &msg, const Signal &sig) {
  // 結果となるraw値を格納
  int64_t ret = 0;

  // 解析の開始位置（byte単位）: MSBが含まれるバイト
  int i = sig.msb / 8;

  // 取得すべきビット数（信号の長さ）
  int bits = sig.size;

  /*
  背景：CAN信号は複数バイトにまたがる可能性がある
  例えば、11ビット長の信号（sig.size = 11）が sig.lsb = 12 にあるとします。これは次のように分かれます：
    sig.lsb = 12 → msg[1] の ビット5
    sig.msb = 22 → msg[2] の ビット7
  このとき、読み込みは複数バイトに分割してループで処理する必要があります。  

  whileループの例
    sig.msb = 22のとき
    int i = sig.msb / 8 = 2
    -----------------------------------------------
    lsb  = 2*8 = 16
    msb  = 22
    size = 22-16 + 1 = 7
    msg[2]　に　7ビット分データを入力. 16〜22bit
    bits = 11 - 7 = 4
    -----------------------------------------------
    little endianの場合
    i = 1
    -----------------------------------------------
    lsb = 12
    msb = 2*8-1 = 15
    size = 15 - 12 + 1 = 4
    msg[1]　に　4ビット分入力. 12〜15bit
    bits = 4 - 4 = 0
    ----------------------------------------------
    bits == 0 ---> ループ終了
    -----------------------------------------------
  */
  while (i >= 0 && i < msg.size() && bits > 0) {
    // i番目のバイトにおけるビットの範囲を決定
    int lsb = (int)(sig.lsb / 8) == i ? sig.lsb : i*8;
    int msb = (int)(sig.msb / 8) == i ? sig.msb : (i+1)*8 - 1;
    int size = msb - lsb + 1;

    /*
    msg[i] の中から、「そのバイト内の lsb 位置」から始まる size ビットを抽出する

    Step-1 : (1ULL << size) - 1)
      sizeビット分だけ「1」が立ったビットマスクを生成するための式
      size = 3 → 0b111
      size = 5 → 0b11111
      size = 7 → 0b01111111

    Step-2 : (msg[i] >> (lsb - (i*8))) 
      そのバイトの中で、必要なビットの最下位位置を右端（LSB）に持ってくる処理
    */
    uint64_t d = (msg[i] >> (lsb - (i*8))) & ((1ULL << size) - 1);

    // 取り出した d を、ret の適切な位置に左シフトして配置
    ret |= d << (bits - size);

    bits -= size;

    // Little Endianの場合は i-
    i = sig.is_little_endian ? i-1 : i+1;
  }
  return ret;
}

/*
CANParser::UpdateCans()で、parse(can.nanos, frame.dat)のように使わる

この関数は、受信した CAN メッセージ（dat）の中から必要な信号の値を取り出し（数値化）て、
チェックサムやカウンタが正しいかを確認します。
問題がなければ、その値を vals や all_vals に保存し、最後に受信した時刻 last_seen_nanos を更新します。

dat
  受信した CAN メッセージの生バイト列（最大 8 バイト）です。
  データの内容は std::vector<uint8_t> で受け取ります

parse_sigs
  この CAN メッセージ内に含まれる各シグナル（信号）の定義リストです。
  Signal 構造体により、「ビット位置・ビット長・符号付き/なし・スケーリング情報」などが定義されています。  
*/
bool MessageState::parse(uint64_t nanos, const std::vector<uint8_t> &dat) {
  // parse_sigs : このクラスの変数. CAN信号構造体の配列
  std::vector<double> tmp_vals(parse_sigs.size());
  bool checksum_failed = false;
  bool counter_failed = false;

  for (int i = 0; i < parse_sigs.size(); i++) {
    const auto &sig = parse_sigs[i];

    // CANバイトデータ + dbcの定義 => CANの生値を取得
    int64_t tmp = get_raw_value(dat, sig);
    
    /*
    符号付きの値（signed）として受け取るために、2の補数表現で負の数を正しく復元する。
      (tmp >> (sig.size-1)) => tmp の 最上位ビット（MSB） を右端に移動し、符号の判定準備
      最上位ビット & 0x1 => 符号ビットが 1 なら 負の数
      tmp -= (1ULL << sig.size) => 負の数の場合、最大値を引いて2の補数にする
    補足
      signed    : -128 〜 +127
      unsigned  : 0 〜 255
      2の補数の例：
        受信値 tmp = 255（0xFF） → MSB = 1 → tmp - 256 = -1
        受信値 tmp = 128（0x80） → MSB = 1 → tmp - 256 = -128
     */
    if (sig.is_signed) {
      tmp -= ((tmp >> (sig.size-1)) & 0x1) ? (1ULL << sig.size) : 0;
    }

    //DEBUG("parse 0x%X %s -> %ld\n", address, sig.name, tmp);

    // calc_checksum自体が nullではない　且つ　calc_checksumの結果が !=tmpの場合、checksum失敗
    if (!ignore_checksum) {
      if (sig.calc_checksum != nullptr && sig.calc_checksum(address, sig, dat) != tmp) {
        checksum_failed = true;
      }
    }

    // COUNTERタイプ　且つ　
    if (!ignore_counter) {
      if (sig.type == SignalType::COUNTER && !update_counter_generic(tmp, sig.size)) {
        counter_failed = true;
      }
    }

    // 物理量 = 生の値 × 倍率 + オフセット
    tmp_vals[i] = tmp * sig.factor + sig.offset;
  }

  // only update values if both checksum and counter are valid
  if (checksum_failed || counter_failed) {
    LOGE_100("0x%X message checks failed, checksum failed %d, counter failed %d", address, checksum_failed, counter_failed);
    return false;
  }

  for (int i = 0; i < parse_sigs.size(); i++) {
    vals[i] = tmp_vals[i];
    all_vals[i].push_back(vals[i]);
  }
  last_seen_nanos = nanos;

  return true;
}

// CANメッセージに含まれる カウンタ信号（連番） が
// **前回の値から 1 増えているか（ロールオーバーあり）**を確認する。
bool MessageState::update_counter_generic(int64_t v, int cnt_size) {
  /*
  カウンタ信号 v が「前回の値 +1」になっているかを調べる
  ((1 << cnt_size) -1) => カウンタの ビット幅 cnt_size　で　マスクを作る
  実際に受信したカウンタ値 v と、上で計算した 期待される次の値 が一致しなければ、カウンタの異常と判断
  */
  if (((counter + 1) & ((1 << cnt_size) -1)) != v) {
    // カウンタ異常であれば、 counter_failをインクリメント
    counter_fail = std::min(counter_fail + 1, MAX_BAD_COUNTER);
    if (counter_fail > 1) {
      INFO("0x%X COUNTER FAIL #%d -- %d -> %d\n", address, counter_fail, counter, (int)v);
    }
  } else if (counter_fail > 0) {
    // 正常　且つ counter_failが正 => デクリメント
    counter_fail--;
  }

  // カウンタを更新
  counter = v;

  // カウンタ異常回数が上限を超えていない場合、Trueを返す
  return counter_fail < MAX_BAD_COUNTER;
}

/*
使われ方 : openpilot/opendbc/car/rivian/radar_interface.py
  messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)
*/
CANParser::CANParser(int abus, const std::string& dbc_name, const std::vector<std::pair<uint32_t, int>> &messages)
  : bus(abus) {

  // DBC（CANメッセージの定義ファイル）から定義情報を取得
  dbc = dbc_lookup(dbc_name);

  // 失敗すれば assert でクラッシュ
  assert(dbc);

  // 非常に大きな初期値（= 2^64 − 1）を設定
  bus_timeout_threshold = std::numeric_limits<uint64_t>::max();

  for (const auto& [address, frequency] : messages) {
    // 上記の使われ方のように設定した際, address(CAN ID)の重複があればエラーを投げる
    // openpilot/opendbc/car/hyundai/radar_interface.py
    //     addressの例 : RADAR_START_ADDR = 0x500
    // disallow duplicate message checks
    if (message_states.find(address) != message_states.end()) {
      std::stringstream is;
      is << "Duplicate Message Check: " << address;
      throw std::runtime_error(is.str());
    }

    // CAN ID を登録
    MessageState &state = message_states[address];
    state.address = address;
    // state.check_frequency = op.check_frequency,

    /*
    各CANメッセージに対して、「10回連続で受信できなかったら無効と判断する」ための時間しきい値（check_threshold）を計算.
    さらに、全体のバス監視用に最も短い（= 頻度が高い）メッセージを基準に bus_timeout_threshold を設定.

    */
    // msg is not valid if a message isn't received for 10 consecutive steps
    if (frequency > 0) {
      // 周波数 frequency に対応する1ステップの時間（ナノ秒単位）＝ 1e9 / frequency
      state.check_threshold = (1000000000ULL / frequency) * 10;

      // 10回受信が途絶えたら「異常」と判断するための変数を作成
      // bus timeout threshold should be 10x the fastest msg
      bus_timeout_threshold = std::min(bus_timeout_threshold, state.check_threshold);
    }

    // CANメッセージ定義（Msg構造体） を DBC から取得
    const Msg *msg = dbc->addr_to_msg.at(address);
    state.name = msg->name;
    state.size = msg->size;
    assert(state.size <= 64);  // max signal size is 64 bytes

    // track all signals for this message
    state.parse_sigs = msg->sigs;
    state.vals.resize(msg->sigs.size());
    state.all_vals.resize(msg->sigs.size());
  }
}

/*
この CANParser コンストラクタは、DBCファイルに含まれる全てのCANメッセージを対象として、
自動的に MessageState を初期化・登録するバージョンです。
  直前のコンストラクタは、手動でCANメッセージを登録する場合。
*/
CANParser::CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter)
  : bus(abus) {
  // Add all messages and signals
  
  dbc = dbc_lookup(dbc_name);
  assert(dbc);

  for (const auto& msg : dbc->msgs) {
    MessageState state = {
      .name = msg.name,
      .address = msg.address,
      .size = msg.size,
      .ignore_checksum = ignore_checksum,
      .ignore_counter = ignore_counter,
    };

    for (const auto& sig : msg.sigs) {
      state.parse_sigs.push_back(sig);
      state.vals.push_back(0);
      state.all_vals.push_back({});
    }

    message_states[state.address] = state;
  }
}

/*
update() は、新しいCANデータがバッチで届いたとき、
つまり「周期ごとに1回」や「受信バッファからまとめて取得したとき」に1回ずつ呼ばれるのが想定される

この関数群は、「ある時点で受信したCANデータ（can_data）」を対象に：
 1. シグナル値を更新（UpdateCans）
 2. タイムアウト・カウンタ異常の検出（UpdateValid）
 3. 更新されたCANアドレス一覧を返す（update）
という流れで1フレーム分の処理を完結させます。
*/
std::set<uint32_t> CANParser::update(const std::vector<CanData> &can_data) {
  // Clear all_values
  for (auto &state : message_states) {
    // 状態量をクリア=初期化する
    for (auto &vals : state.second.all_vals) vals.clear();
  }

  std::set<uint32_t> updated_addresses;

  for (const auto &c : can_data) {
    if (first_nanos == 0) {
      first_nanos = c.nanos;
    }

    // シグナル値を更新
    UpdateCans(c, updated_addresses);

    // タイムアウト・カウンタ異常の検出
    UpdateValid(c.nanos);
  }

  // 更新されたCANアドレス一覧を返す
  return updated_addresses;
}

void CANParser::UpdateCans(const CanData &can, std::set<uint32_t> &updated_addresses) {
  //DEBUG("got %zu messages\n", can.frames.size());

  // この時点で「対象のバスからのデータが1つも来ていない」と仮定して初期化
  bool bus_empty = true;

  /*
  frame.src はこのフレームが来た CAN バス番号（例：0、1、2など）
  bus は CANParser が担当しているバス番号 => 一致しない場合は、処理対象外としてスキップ
  */
  for (const auto &frame : can.frames) {
    if (frame.src != bus) {
      // DEBUG("skip %d: wrong bus\n", cmsg.getAddress());
      continue;
    }
    bus_empty = false;

    // 今回の CAN ID が DBCファイルに入っているか検索
    auto state_it = message_states.find(frame.address);

    // 入っていない場合、無視して continue
    if (state_it == message_states.end()) {
      // DEBUG("skip %d: not specified\n", cmsg.getAddress());
      continue;
    }

    // CANフレームの長さが 64ビット以上の場合、無視して、continue
    if (frame.dat.size() > 64) {
      DEBUG("got message longer than 64 bytes: 0x%X %zu\n", frame.address, frame.dat.size());
      continue;
    }

    // TODO: this actually triggers for some cars. fix and enable this
    //if (dat.size() != state_it->second.size) {
    //  DEBUG("got message with unexpected length: expected %d, got %zu for %d", state_it->second.size, dat.size(), cmsg.getAddress());
    //  continue;
    //}

    // CAN フレームから必要な情報を取得し、成功したら updated_addresses　に追加する
    if (state_it->second.parse(can.nanos, frame.dat)) {
      updated_addresses.insert(state_it->first);
    }
  }

  // update bus timeout
  if (!bus_empty) {
    // 最後に有効なCANフレームを受信した時刻（ナノ秒）を記録
    last_nonempty_nanos = can.nanos;
  }

  // CANバスが「しばらく何も受信していない」＝タイムアウト状態かどうかを判定
  bus_timeout = (can.nanos - last_nonempty_nanos) > bus_timeout_threshold;
}



void CANParser::UpdateValid(uint64_t nanos) {
  // 8秒以上経過したら、未受信メッセージの警告を表示してもよいと判断
  const bool show_missing = (nanos - first_nanos) > 8e9;

  // フラグ初期化：すべての信号が正常である前提から始める
  // 全体として「未受信 or タイムアウトがない」か
  bool _valid = true;

  // カウンタ異常がないか
  bool _counters_valid = true;


  for (const auto& kv : message_states) {
    const auto& state = kv.second;

    // カウンタ異常が一定回数（MAX_BAD_COUNTER）以上続いているか？
    if (state.counter_fail >= MAX_BAD_COUNTER) {
      // 1つでも異常があればフラグを false にする
      _counters_valid = false;
    }

    // まだ一度もこのメッセージを受信していないか？
    const bool missing = state.last_seen_nanos == 0;

    // 最後に受信した時刻から、しきい値（check_threshold）を超えていればタイムアウトとみなす
    const bool timed_out = (nanos - state.last_seen_nanos) > state.check_threshold;

    // ① このメッセージには受信チェックが有効で、 ② かつ、未受信またはタイムアウトしていたら
    if (state.check_threshold > 0 && (missing || timed_out)) {
      // 8秒以上経過しており、バス全体が止まっていないときだけログを出す
      if (show_missing && !bus_timeout) {
        if (missing) {
          // 一度も受信されていない（初期状態）
          LOGE_100("0x%X '%s' NOT SEEN", state.address, state.name.c_str());
        } else if (timed_out) {
          // 以前は受信していたが、しきい値を超えて届かなくなった
          LOGE_100("0x%X '%s' TIMED OUT", state.address, state.name.c_str());
        }
      }

      // いずれかの信号が missing または timed out → 全体を invalid とみなす
      _valid = false;
    }
  }

  /*
    直前のチェック（_valid）に問題がなければカウントをリセット（0）
    問題があれば、連続エラー回数を +1
  */
  can_invalid_cnt = _valid ? 0 : (can_invalid_cnt + 1);

  /*
    最終的なCANの有効性判定：
    ① 連続エラー回数が閾値未満
    ② すべてのカウンタが正常（_counters_valid）  
  */
  can_valid = (can_invalid_cnt < CAN_INVALID_CNT) && _counters_valid;
}

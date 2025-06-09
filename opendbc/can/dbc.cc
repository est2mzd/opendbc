#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <iterator>
#include <cstring>
#include <clocale>

#include "opendbc/can/common.h"
#include "opendbc/can/common_dbc.h"

/*
std::regex とは？
C++の標準ライブラリ <regex> に含まれる 正規表現（Regular Expression）を扱うクラス です。
  std::regex pattern("正規表現");
のように書くことで、文字列のパターンマッチング を行うためのオブジェクトを作成できます。

正規表現 | 意味                    |  例
.      | 任意の1文字              | "a.c" は "abc", "acc" にマッチ
*      | 直前の文字が0回以上繰り返し | "a*" は "", "a", "aa", ...
+      | 直前の文字が1回以上        | "a+" は "a", "aa", ...
?      | 直前の文字が0回または1回   | "ab?c" は "ac" または "abc"
[]     | 文字クラス（いずれか1文字） | "[abc]" は a, b, または c
[^]    | 否定文字クラス            | "[^abc]" は a/b/c 以外
\d     | 数字（0-9）              | "\\d+" は 1桁以上の数字
\w     | 単語文字（英数字+_）       | "\\w+" は英数字単語
^      | 行頭                    | "^abc" は "abc" で始まる文字列にマッチ
$      | 行末                    | "abc$" は "abc" で終わる文字列にマッチ
`      | `                      | OR（または）
*/

/*
(\w+) | 英数字とアンダースコア1文字以上
*     | スペースが0個以上
:     | コロン
*/

// BO_ 304 GAS_PEDAL_2: 8 PCM
std::regex bo_regexp(R"(^BO_ (\w+) (\w+) *: (\w+) (\w+))");

/*
通常の信号: 常に有効, エンジン回転数、速度など
SG_  NAME               : START|LENGTH@ENDIAN+/- (FACTOR,OFFSET) [MIN|MAX] "UNIT" NODE
     └──────┬───────┘
            シグナル名のみ

Multiplex信号: 特定の条件下でのみ有効,  同じCAN IDで複数パターンのデータを切り替えられる
SG_  NAME   MUX_SELECTOR : START|LENGTH@ENDIAN+/- (FACTOR,OFFSET) [MIN|MAX] "UNIT" NODE
     └─┬──┘ └────┬────┘
       名     Mやm0など
*/

// SG_ ENGINE_TORQUE_ESTIMATE : 7|16@0- (1,0) [-1000|1000] "Nm" EON
std::regex sg_regexp( R"(^SG_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");

// opendbcには存在しないかも
std::regex sgm_regexp(R"(^SG_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");

// VAL_ 401 GEAR_SHIFTER 32 "L" 16 "S" 8 "D" 4 "N" 2 "R" 1 "P";
std::regex val_regexp(R"(VAL_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*))");

/*
文字列を "（ダブルクォーテーション）で分割する
R"(...)" : C++のraw string literal。エスケープせずに ", \ などを書ける
[\"]+    : 1個以上の "（ダブルクォート）にマッチする
*/
 
// 例
std::regex val_split_regexp{R"([\"]+)"};  // split on "

// str.findの返り値 : prefix が見つかった位置(std::size_t) or Not Found(std::string::npos)
inline bool startswith(const std::string& str, const char* prefix) {
  return str.find(prefix, 0) == 0;
}

/*
inline: 「関数の本体をそのまま呼び出し場所に埋め込むように」というヒントをコンパイラに与える
std::initializer_list : 中括弧 {} で渡した複数の値を関数やコンストラクタに一括で渡すための仕組み
*/
inline bool startswith(const std::string& str, std::initializer_list<const char*> prefix_list) {
  for (auto prefix : prefix_list) {
    if (startswith(str, prefix)) return true;
  }
  return false;
}

/*
文字列 s の 前後の空白類（スペース・タブ・改行など）を削除する。
t は削除したい文字のセット（デフォルトは空白文字全部）。
  " \t\n\r\f\v" の意味（6文字）

  文字 | 名前            | 意味（表示） | ASCIIコード
  ' ' | Space（スペース） | 半角空白（通常の空白）	32 (0x20)
  \t  | Horizontal Tab | 水平タブ（Tabキー）	9 (0x09)
  \n  | Line Feed (LF) | 改行（UNIX系の行末）	10 (0x0A)
  \r  | Carriage       | Return (CR)	復帰（Windowsの行末で使用、\r\n）	13 (0x0D)
  \f  | Form Feed      | 改ページ（印刷用途）	12 (0x0C)
  \v  | Vertical Tab   | 垂直タブ（ほとんど使われない）	11 (0x0B)
*/
inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
  // 末尾から前方に検索し、「t に含まれない最初の文字の位置」を見つける。
  // +1：その直後が「余計な空白の始まり」なので、そこから後ろを erase()。
  s.erase(s.find_last_not_of(t) + 1);

  // 先頭から「t に含まれない最初の文字の位置」を探す。
  // erase(0, その位置)：前方の空白を削除。
  return s.erase(0, s.find_first_not_of(t));
}

// dbc_parse()　で使われる関数
ChecksumState* get_checksum(const std::string& dbc_name) {
  ChecksumState* s = nullptr;
  if (startswith(dbc_name, {"honda_", "acura_"})) {
    s = new ChecksumState({4, 2, 3, 5, false, HONDA_CHECKSUM, &honda_checksum});
  } else if (startswith(dbc_name, {"toyota_", "lexus_"})) {
    s = new ChecksumState({8, -1, 7, -1, false, TOYOTA_CHECKSUM, &toyota_checksum});
  } else if (startswith(dbc_name, "hyundai_canfd_generated")) {
    s = new ChecksumState({16, -1, 0, -1, true, HKG_CAN_FD_CHECKSUM, &hkg_can_fd_checksum});
  } else if (startswith(dbc_name, {"vw_mqb", "vw_mqbevo", "vw_meb"})) {
    s = new ChecksumState({8, 4, 0, 0, true, VOLKSWAGEN_MQB_MEB_CHECKSUM, &volkswagen_mqb_meb_checksum});
  } else if (startswith(dbc_name, "vw_pq")) {
    s = new ChecksumState({8, 4, 0, -1, true, XOR_CHECKSUM, &xor_checksum});
  } else if (startswith(dbc_name, "subaru_global_")) {
    s = new ChecksumState({8, -1, 0, -1, true, SUBARU_CHECKSUM, &subaru_checksum});
  } else if (startswith(dbc_name, "chrysler_")) {
    s = new ChecksumState({8, -1, 7, -1, false, CHRYSLER_CHECKSUM, &chrysler_checksum});
  } else if (startswith(dbc_name, "fca_giorgio")) {
    s = new ChecksumState({8, -1, 7, -1, false, FCA_GIORGIO_CHECKSUM, &fca_giorgio_checksum});
  } else if (startswith(dbc_name, "comma_body")) {
    s = new ChecksumState({8, 4, 7, 3, false, PEDAL_CHECKSUM, &pedal_checksum});
  } else if (startswith(dbc_name, "tesla_model3_party")) {
    s = new ChecksumState({8, -1, 0, -1, true, TESLA_CHECKSUM, &tesla_checksum, &tesla_setup_signal});
  }
  return s;
}

/*
CAN通信において、DBCファイルの SG_ 行は基本的な構文情報（start_bit, size, endian, factor, offsetなど）
しか持ちません。しかし、実際の車載制御には追加情報が必要

set_signal_type() 関数の存在目的は、
DBCファイルに定義された各 Signal に対して、型（type）やチェックサム計算関数など、
通信制御で必要な追加情報を割り当てること
*/
void set_signal_type(Signal& s, ChecksumState* chk, const std::string& dbc_name, int line_num) {
  // デフォルトではチェックサム関数は未設定
  s.calc_checksum = nullptr;

  // FIXME: always assign COUNTER type without explicit ChecksumState
  if (chk) {
    if (chk->setup_signal) {
      chk->setup_signal(s, dbc_name, line_num);
    }

    // 特殊なペダル信号名（CHECKSUM_PEDAL / COUNTER_PEDAL）に対して、s.type を上書き設定
    pedal_setup_signal(s, dbc_name, line_num);

    // Signal名が "CHECKSUM" の場合、チェックサム型と計算関数を登録
    //   例  :  SG_ CHECKSUM : 59|4@0+ (1,0) [0|3] "" EON
    if (s.name == "CHECKSUM") {
      s.type = chk->checksum_type;
      s.calc_checksum = chk->calc_checksum;
    
    //   例  :  SG_ COUNTER : 61|2@0+ (1,0) [0|15] "" EON
    } else if (s.name == "COUNTER") {
      s.type = COUNTER;
    }

    if (s.type > COUNTER) {
      DBC_ASSERT(chk->checksum_size == -1 || s.size == chk->checksum_size, s.name << " is not " << chk->checksum_size << " bits long");
      DBC_ASSERT(chk->checksum_start_bit == -1 || (s.start_bit % 8) == chk->checksum_start_bit, s.name << " starts at wrong bit");
      DBC_ASSERT(chk->little_endian == s.is_little_endian, s.name << " has wrong endianness");
      DBC_ASSERT(chk->calc_checksum != nullptr, "Checksum calculate function not supplied for " << s.name);
    }  else if (s.type == COUNTER) {
      DBC_ASSERT(chk->counter_size == -1 || s.size == chk->counter_size, s.name << " is not " << chk->counter_size << " bits long");
      DBC_ASSERT(chk->counter_start_bit == -1 || (s.start_bit % 8) == chk->counter_start_bit, s.name << " starts at wrong bit");
      DBC_ASSERT(chk->little_endian == s.is_little_endian, s.name << " has wrong endianness");
    }
  }
}

DBC* dbc_parse_from_stream(const std::string &dbc_name, std::istream &stream, ChecksumState *checksum, bool allow_duplicate_msg_name) {
  uint32_t address = 0;
  std::set<uint32_t> address_set;
  std::set<std::string> msg_name_set;
  std::map<uint32_t, std::set<std::string>> signal_name_sets;
  std::map<uint32_t, std::vector<Signal>> signals;
  DBC* dbc = new DBC;
  dbc->name = dbc_name;
  // 小数点などの数値フォーマットで , を使わず . に統一するため（特に欧州環境対策）
  std::setlocale(LC_NUMERIC, "C");

  // used to find big endian LSB from MSB and size
  // Big Endian（Motorola形式） の start_bit（MSB）から LSBの位置を逆算するためのテーブルの作成
  // be_bits = [7,6,5,4,3,2,1,0, 15,14,...] 
  std::vector<int> be_bits;
  for (int i = 0; i < 64; i++) {
    for (int j = 7; j >= 0; j--) {
      be_bits.push_back(j + i * 8);
    }
  }

  std::string line;
  int line_num = 0;
  std::smatch match;
  // TODO: see if we can speed up the regex statements in this loop, SG_ is specifically the slowest
  while (std::getline(stream, line)) {
    line = trim(line);
    line_num += 1;
    // BO_ で始まっている場合
    if (startswith(line, "BO_ ")) {
      // new group
      // 更に BO_ 以降の文字列の妥当性をチェック
      bool ret = std::regex_match(line, match, bo_regexp);
      // Falseの場合、エラーを出す
      DBC_ASSERT(ret, "bad BO: " << line);

      // dbc->msgsの末尾に 新しいオブジェクトを追加し、msg に代入
      Msg& msg = dbc->msgs.emplace_back();
      address = msg.address = std::stoul(match[1].str());  // could be hex
      msg.name = match[2].str();
      msg.size = std::stoul(match[3].str());

      // check for duplicates
      // 同じメッセージIDが複数出てこないようチェック（CANではIDの一意性が重要）
      DBC_ASSERT(address_set.find(address) == address_set.end(), "Duplicate message address: " << address << " (" << msg.name << ")");
      address_set.insert(address);

      // オプションで、メッセージ名の重複も禁止できる（allow_duplicate_msg_name = false のとき）
      if (!allow_duplicate_msg_name) {
        DBC_ASSERT(msg_name_set.find(msg.name) == msg_name_set.end(), "Duplicate message name: " << msg.name);
        msg_name_set.insert(msg.name);
      }
    
    // SG_ で始まるとき
    } else if (startswith(line, "SG_ ")) {
      // new signal
      int offset = 0;

      /*
      更に SG_ 以降の文字列の妥当性をチェック
      sg_regexp と sgm_regexp で マッチしたときの match[] 配列の構造が異なるため、
      後の match[n] アクセスで正しい位置から値を取り出せるように offset を調整     
      
      match　に入るもの: 詳細は sg_regexp で()で囲われているものを参照
        例 : SG_ ENGINE_RPM : 23|16@1+ (0.125,0) [0|16383.75] "rpm"  EON
        match[0]	全体マッチ（行全体）
        match[1]	ENGINE_RPM
        match[2]	23
        match[3]	16
        match[4]	1
        match[5]	+
        match[6]	0.125
        match[7]	0        
      */
      if (!std::regex_search(line, match, sg_regexp)) {
        bool ret = std::regex_search(line, match, sgm_regexp);
        DBC_ASSERT(ret, "bad SG: " << line);
        offset = 1;
      }

      // SG_　の各データを保存
      Signal& sig = signals[address].emplace_back();
      sig.name = match[1].str();
      sig.start_bit = std::stoi(match[offset + 2].str());
      sig.size = std::stoi(match[offset + 3].str());
      sig.is_little_endian = std::stoi(match[offset + 4].str()) == 1;
      sig.is_signed = match[offset + 5].str() == "-";
      sig.factor = std::stod(match[offset + 6].str());
      sig.offset = std::stod(match[offset + 7].str());

      // その他の必要な情報を sig に追加
      set_signal_type(sig, checksum, dbc_name, line_num);

      // Endian違いの処理
      if (sig.is_little_endian) {
        sig.lsb = sig.start_bit;
        sig.msb = sig.start_bit + sig.size - 1;
      } else {
        auto it = find(be_bits.begin(), be_bits.end(), sig.start_bit);
        sig.lsb = be_bits[(it - be_bits.begin()) + sig.size - 1];
        sig.msb = sig.start_bit;
      }
      DBC_ASSERT(sig.lsb < (64 * 8) && sig.msb < (64 * 8), "Signal out of bounds: " << line);

      // Check for duplicate signal names
      DBC_ASSERT(signal_name_sets[address].find(sig.name) == signal_name_sets[address].end(), "Duplicate signal name: " << sig.name);
      signal_name_sets[address].insert(sig.name);

    // VAL_ で始まる場合
    // VAL_ 401 GEAR_SHIFTER 32 "L" 16 "S" 8 "D" 4 "N" 2 "R" 1 "P";
    } else if (startswith(line, "VAL_ ")) {
      // new signal value/definition
      bool ret = std::regex_search(line, match, val_regexp);
      DBC_ASSERT(ret, "bad VAL: " << line);

      auto& val = dbc->vals.emplace_back();
      // CAN ID
      val.address = std::stoul(match[1].str());  // could be hex
      // Signal名
      val.name = match[2].str();
      
      // match[3] = 32 "L" 16 "S" 8 "D" 4 "N" 2 "R" 1 "P"... というラベル列全体の文字列
      auto defvals = match[3].str();

      /*
      "（ダブルクォート） で分割して、文字列リストに分解
      std::sregex_token_iterator は、正規表現を使って文字列を分割・トークン化するためのイテレータ
      構文 : it{検索開始位置, 検索終了位置, 正規表現, -1=マッチ「しない」部分を抽出（つまり "P" → P）}
      */
      std::sregex_token_iterator it{defvals.begin(), defvals.end(), val_split_regexp, -1};

      // convert strings to UPPER_CASE_WITH_UNDERSCORES
      std::vector<std::string> words{it, {}};
      for (auto& w : words) {
        w = trim(w); // 前後の空白除去
        std::transform(w.begin(), w.end(), w.begin(), ::toupper); // 大文字へ変換
        std::replace(w.begin(), w.end(), ' ', '_'); //スペースを_に変換
      }

      // join string
      // val.def_val = "32 L 16 S 8 D 4 N 2 R 1 P" となる
      std::stringstream s;
      std::copy(words.begin(), words.end(), std::ostream_iterator<std::string>(s, " "));
      val.def_val = s.str();
      val.def_val = trim(val.def_val);
    }
  }

  // dbc->msgs = BO_行で定義されたMsgの構造体のリスト
  for (auto& m : dbc->msgs) {
    // SG_ 行で一時的に保持していた信号群 を 各 BO_ に対応する m.sigs に格納する
    m.sigs = signals[m.address]; // = BO_[CAN_ID] の中に 各SG_の情報がある
    dbc->addr_to_msg[m.address] = &m; // address から can message
    dbc->name_to_msg[m.name] = &m;    // BO_名   から can message
  }

  // bc->vals = VAL_行で定義されたMsgの構造体のリスト
  for (auto& v : dbc->vals) {
    v.sigs = signals[v.address];
  }


  return dbc;
}

// DBCファイルをパース（解読）して DBC 構造体ポインタを返す関数
DBC* dbc_parse(const std::string& dbc_path) {
  // DBCファイルを読み込むための入力ストリームを開く
  std::ifstream infile(dbc_path);

  // ファイルが開けなかった場合は nullptr を返す（例：ファイルが存在しない）
  if (!infile) return nullptr;

  // 入力されたパスからファイル名（例："honda_accord.dbc"）だけを取得
  const std::string dbc_name = std::filesystem::path(dbc_path).filename();

  // チェックサム計算用の状態オブジェクトを生成（DBCファイル名から適切な種類を選ぶ）
  std::unique_ptr<ChecksumState> checksum(get_checksum(dbc_name));

  // ストリームとチェックサムを用いて、実際のパース処理を行う
  return dbc_parse_from_stream(dbc_name, infile, checksum.get());
}


const std::string get_dbc_root_path() {
  char *basedir = std::getenv("BASEDIR");
  if (basedir != NULL) {
    return std::string(basedir) + "/opendbc/dbc";
  } else {
    return DBC_FILE_PATH; // SConscriptで定義されている
  }
}

const DBC* dbc_lookup(const std::string& dbc_name) {
  // 排他制御のためのmutex
  static std::mutex lock;

  // DBCファイルのキャッシュマップ
  static std::map<std::string, DBC*> dbcs;
  
  // ファイルパスを初期化
  std::string dbc_file_path = dbc_name;

  // もしファイルが存在しない場合は、ルートパス＋.dbc拡張子を補って探す
  if (!std::filesystem::exists(dbc_file_path)) {
    dbc_file_path = get_dbc_root_path() + "/" + dbc_name + ".dbc";
  }

  // 排他ロックを取得
  std::unique_lock lk(lock);

  // 既にキャッシュされているか確認. map_key = dbc_name
  // 見つかれば、it->first = dbc_name, it->second=DBC*　となる
  auto it = dbcs.find(dbc_name);

  // キャッシュされていなければ、新たにパースして追加
  if (it == dbcs.end()) {
    it = dbcs.insert(it, {dbc_name, dbc_parse(dbc_file_path)});
  }

  // 該当するDBC構造体ポインタを返す（const指定で読み取り専用）
  return it->second;
}

std::vector<std::string> get_dbc_names() {
  // DBCルートパスを取得（例: "/home/user/opendbc/dbc"）
  static const std::string& dbc_file_path = get_dbc_root_path();

  // 結果を格納するベクター
  std::vector<std::string> dbcs;

  /*
  ディレクトリ内の全エントリをループ（ファイル・フォルダ含む）
  
  std::filesystem::directory_iterator i(dbc_file_path), end
    これは、iの初期化 と end の定義を同時にしている
  */
  
  
  
  for (std::filesystem::directory_iterator i(dbc_file_path), end; i != end; i++) {
    // ディレクトリは除外（ファイルだけ対象）
    if (!is_directory(i->path())) {
      // ファイル名（例："honda_accord.dbc"）を取得
      std::string filename = i->path().filename();

      // ファイル名が "_" で始まらず、".dbc" で終わるものだけ対象
      if (!startswith(filename, "_") && endswith(filename, ".dbc")) {

        // 拡張子 ".dbc" を除いたベース名をベクターに追加
        dbcs.push_back(filename.substr(0, filename.length() - 4));
      }
    }
  }
  return dbcs;
}

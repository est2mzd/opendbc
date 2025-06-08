#pragma once

#ifdef SWAGLOG
#include SWAGLOG
#else

#define CLOUDLOG_DEBUG 10
#define CLOUDLOG_INFO 20
#define CLOUDLOG_WARNING 30
#define CLOUDLOG_ERROR 40
#define CLOUDLOG_CRITICAL 50

/*
マクロの引数。3つある：
① lvl → ログレベル（使ってない）
② fmt → フォーマット文字列
③ ... → 可変引数（任意の数の値）
printf(fmt "\n", ## __VA_ARGS__) : フォーマット文字列に改行を付けて printf を呼ぶ処理
*/
#define cloudlog(lvl, fmt, ...) printf(fmt "\n", ## __VA_ARGS__)

/*
引数名  |意図（将来的な機能のため）            |現在の動作
------------------------------------------------
burst  |バースト数（例：1秒あたりの最大出力数） |無視される
millis |インターバル時間（ms単位）            |無視される
lvl    |ログレベル（INFO, DEBUGなど）        |無視される
fmt    |printf のフォーマット文字列          |使用される
...    |可変引数（任意の値）                 |使用される
*/
#define cloudlog_rl(burst, millis, lvl, fmt, ...) printf(fmt "\n", ##__VA_ARGS__)

#define LOGD(fmt, ...) cloudlog(CLOUDLOG_DEBUG, fmt, ## __VA_ARGS__)
#define LOG(fmt, ...) cloudlog(CLOUDLOG_INFO, fmt, ## __VA_ARGS__)
#define LOGW(fmt, ...) cloudlog(CLOUDLOG_WARNING, fmt, ## __VA_ARGS__)
#define LOGE(fmt, ...) cloudlog(CLOUDLOG_ERROR, fmt, ## __VA_ARGS__)

#define LOGD_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_DEBUG, fmt, ## __VA_ARGS__)
#define LOG_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_INFO, fmt, ## __VA_ARGS__)
#define LOGW_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_WARNING, fmt, ## __VA_ARGS__)
#define LOGE_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_ERROR, fmt, ## __VA_ARGS__)

#endif

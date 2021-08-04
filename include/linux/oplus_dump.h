/*
 * Copyright (C), 2019, OPLUS Mobile Comm Corp., Ltd.
 * File: - linux/oplus_dump.h
 * Description: Util about dump in the period of debugging.
 * Version: 1.0
 * Date: 2019/11/01
 *
 *----------------------Revision History: ---------------------------
 *   <author>        <date>         <version>         <desc>
 *    Bin.Xu       2019/11/01        1.0              created
 *-------------------------------------------------------------------
 */
extern void __attribute__((unused)) get_fdump_passwd(const char *val);
extern void __attribute__((unused)) OPLUS_DUMP(const char s[24]);
extern void __attribute__((unused)) oplus_key_process(int key, int val);
extern void __attribute__((unused)) dump_process(int key, int val);
extern void __attribute__((unused)) dump_post_process(int key, int val);
extern void __attribute__((unused)) oplus_key_event(int type, int key, int val);


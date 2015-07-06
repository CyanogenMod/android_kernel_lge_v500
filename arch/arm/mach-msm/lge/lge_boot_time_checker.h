#ifndef _LGE_BOOT_TIME_CHECKER_H
#define _LGE_BOOT_TIME_CHECKER_H
#define SBL1_LOG_ADDR 0x2A01FC00
#define SBL1_LOG_SIZE 1024
#define SBL2_LOG_ADDR 0x2E05DA00
#define SBL2_LOG_SIZE 1536
// [altev][factory], sunghun1.jung@lgepartner.com, 20140108, Support Trickle Charging Scenario for VZW {
#if defined(CONFIG_MACH_APQ8064_ALTEV)
// Consider Increaded SBL3 Size. 2MB -> 3MB
#define SBL3_LOG_ADDR (0x900FDA00 + 0x100000)
#define SBL3_LOG_SIZE (1536)
#define LK_START_TIME_ADDR (0x900FF9F0 + 0x100000)
#define LK_END_TIME_ADDR (0x900FF9F4 + 0x100000)
#else
#define SBL3_LOG_ADDR 0x900FDA00
#define SBL3_LOG_SIZE 1536
#define LK_START_TIME_ADDR 0x900FF9F0
#define LK_END_TIME_ADDR 0x900FF9F4
#endif /*CONFIG_MACH_APQ8064_ALTEV  */

#endif

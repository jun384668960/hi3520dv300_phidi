
// type define
typedef signed char         INT8;           /* 8 bits, [-128, 127] */
typedef short               INT16;          /* 16 bits */
typedef int                 INT32;          /* 32 bits */
typedef long long           INT64;          /* 64 bits */

typedef unsigned char       UINT8;          /* 8 bits, [0, 255] */
typedef unsigned short      UINT16;         /* 16 bits */
typedef unsigned int        UINT32;         /* 32 bits */
typedef unsigned long long  UINT64;         /* 64 bits */
typedef unsigned short      WCHAR;         /* 16 bits */

extern unsigned char Amba_app_srt_edesen_crypt(unsigned char *ck235_des_data,unsigned char *pDataBuffer);
extern void AppLibCard_ConfigDefault(void);
extern void AppLibVideoEnc_StartPipe(void);
extern void AppLibVideoDec_Reset(void);
extern int srt_check(void);



#define PARAMETERS_FILE "/sdcard/para.dat"


struct tParameters 
{
	unsigned short regAddr;
	unsigned short value;
};

extern struct tParameters  *pgParaBuffer;

extern int parser_open(char *pfilename);
extern void parser_close(void);



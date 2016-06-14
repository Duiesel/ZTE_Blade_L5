/* Copyright (c) 2013, Shockley International Trading CO, Ltd.. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Version: 1.0
 * Creater: Sean Wang
 * Shockley International Trading CO, Ltd.
 * Note:
 * This is first version, will read file from kernel directly, in the future, will 
 * change to read file and parsing in user space.
 */


#include <linux/kernel.h>  
#include <linux/types.h> 
//#include <linux/delay.h>  
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include<linux/slab.h>
//#include <linux/config.h> 
#include <linux/fs.h> 
#include <linux/sched.h> 
#include <linux/file.h> 
#include <asm/processor.h> 
#include <asm/uaccess.h>
 
/* 
  file I/O in kernel module 
*/ 
#define EOF (-1) 
#define SEEK_SET 0 
#define SEEK_CUR 1 
#define SEEK_END 2 
 

typedef uint8_t bool_t;

#define FALSE 0  
#define TRUE  1 

#define PARSER_TAG [<<<<<<<<<<<<<<PARSER>>>>>>>>>>>>>]

//static DEFINE_MUTEX(parse_access);


struct tParameters 
{
	unsigned short regAddr;
	unsigned short value;
};

struct tParameters  *pgParaBuffer = NULL;

// Context : User 
// Parameter : 
// filename : filename to open 
// flags : 
// O_RDONLY, O_WRONLY, O_RDWR 
// O_CREAT, O_EXCL, O_TRUNC, O_APPEND, O_NONBLOCK, O_SYNC, ... 
// mode : file creation permission. 
// S_IRxxx S_IWxxx S_IXxxx (xxx = USR, GRP, OTH), S_IRWXx (x = U, G, O) 
// Return : 
// file pointer. if error, return NULL 
struct file *klib_fopen(const char *filename, int flags, int mode) 
{  
    //struct file *filp = filp_open(filename, flags, mode); 
    struct file *filp = filp_open(filename, O_RDWR, 0664); 
 
    return (IS_ERR(filp)) ? NULL : filp; 
} 
 
// Context : User 
// Parameter : 
// filp : file pointer 
// Return : 
void klib_fclose(struct file *filp) 
{ 
    if (filp) 
        filp_close(filp, NULL);
} 

//usage: copy file's text lines to a buffer.			
//pf: input the file handle.
//return: lines in file, fail is -1
int copy2Buffer(char *pfilename)
{
	int filesize = 0, lines=0;
	mm_segment_t oldfs; 
	loff_t pos=0;
	struct file *pf = NULL;
	
	//open file
	pf = klib_fopen(pfilename, O_RDONLY, S_IRUGO|S_IWUSR); 
	if(pf == NULL)
	{
		printk(KERN_ERR "PARSER_TAG open file %s, fail~!\n", pfilename);  		
		return 0;
	}
	//get file size
	filesize = vfs_llseek(pf,0,SEEK_END);
	if(filesize == 0) return 0;
	
	//alloc memory for file read
	printk(KERN_ERR "PARSER_TAG filesize=%d\n", filesize);
	pgParaBuffer=(struct tParameters  *)kzalloc(filesize, GFP_KERNEL);
	if(pgParaBuffer)
		printk(KERN_ERR "PARSER_TAG pgParaBuffer=%x\n", (unsigned int )pgParaBuffer);
	else 
		return 0;
	//read whole file to buffer
	oldfs = get_fs(); 
	pos=0;
	set_fs(KERNEL_DS); 
	vfs_read(pf,pgParaBuffer,filesize,&pos);
	set_fs(oldfs); 	
	//printk(KERN_ERR "%s", pFilebuffer);
	
	//close file
	klib_fclose(pf);
	
	lines = filesize/sizeof(struct tParameters);

	#if 1 //follow code only for debug
	{
	int i=0;
	for(i=0;i<lines;i++)
		printk(KERN_ERR "line %d [%x,%x]", i, pgParaBuffer[i].regAddr,pgParaBuffer[i].value);
		
	}
	#endif

	return lines;

}

//usage: open the text file for parser
//pfilename: input the file path
//return: lines in file, fail is -1
int parser_open(char *pfilename)
{
	return (copy2Buffer(pfilename));
}

//usage: close the text file
void parser_close(void)
{

	if(pgParaBuffer)
		kfree(pgParaBuffer);
	printk(KERN_ERR "PARSER_TAG parser closed\n");
}


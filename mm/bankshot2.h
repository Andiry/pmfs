#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/blkdev.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/bio.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/types.h>
//#include <linux/radix-tree.h>
#include <linux/buffer_head.h> /* invalidate_bh_lrus() */
//#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/file.h>

#include <asm/uaccess.h>

#define bs2_dbg(s, args ...)	pr_info(s, ## args)
#define bs2_info(s, args ...)	pr_info(s, ## args)

#define BANKSHOT2_RESERVE_SPACE	(4 << 20)
#define BANKSHOT2_NUM_MINORS	1

/* Pmfs.h */
#define	CACHELINE_SIZE	64
#define	BANKSHOT2_BLOCK_TYPE_4K	0
#define	BANKSHOT2_SB_SIZE	512
#define	BANKSHOT2_SUPER_MAGIC	0xDEAD
#define	BANKSHOT2_BLOCK_TYPE_4K	0
#define	BANKSHOT2_ROOT_INO	1
#define	BANKSHOT2_INODE_SIZE	128
#define	BANKSHOT2_INODE_BITS	7
#define	BANKSHOT2_DEFAULT_JOURNAL_SIZE	(4 << 20)

/* PMFS supported data blocks */
/* Currently only support 4K block */
#define BANKSHOT2_BLOCK_TYPE_4K     0
#define BANKSHOT2_BLOCK_TYPE_2M     1
#define BANKSHOT2_BLOCK_TYPE_1G     2
#define BANKSHOT2_BLOCK_TYPE_MAX    3
#define	META_BLK_SHIFT	9

/* INODE HINT Start at 3 */
#define	BANKSHOT2_FREE_INODE_HINT_START	3

/* Bankshot */
#define DISK 0
#define CACHE 1


/* ========================= Data structures =============================== */


/*
 * Structure of an inode in PMFS. Things to keep in mind when modifying it.
 * 1) Keep the inode size to within 96 bytes if possible. This is because
 *    a 64 byte log-entry can store 48 bytes of data and we would like
 *    to log an inode using only 2 log-entries
 * 2) root must be immediately after the qw containing height because we update
 *    root and height atomically using cmpxchg16b in pmfs_decrease_btree_height 
 * 3) i_size, i_ctime, and i_mtime must be in that order and i_size must be at
 *    16 byte aligned offset from the start of the inode. We use cmpxchg16b to
 *    update these three fields atomically.
 */
struct bankshot2_inode {
	/* first 48 bytes */
	__le16	i_rsvd;         /* reserved. used to be checksum */
	u8	    height;         /* height of data b-tree; max 3 for now */
	u8	    i_blk_type;     /* data block size this inode uses */
	__le32	i_flags;            /* Inode flags */
	__le64	root;               /* btree root. must be below qw w/ height */
	__le64	i_size;             /* Size of data in bytes */
	__le32	i_ctime;            /* Inode modification time */
	__le32	i_mtime;            /* Inode b-tree Modification time */
	__le32	i_dtime;            /* Deletion Time */
	__le16	i_mode;             /* File mode */
	__le16	i_links_count;      /* Links count */
	__le64	i_blocks;           /* Blocks count */

	/* second 48 bytes */
	__le64	i_xattr;            /* Extended attribute block */
	__le32	i_uid;              /* Owner Uid */
	__le32	i_gid;              /* Group Id */
	__le32	i_generation;       /* File version (for NFS) */
	__le32	i_atime;            /* Access time */

	struct {
		__le32 rdev;    /* major/minor # */
	} dev;              /* device inode */
	__le32 padding;     /* pad to ensure truncate_item starts 8-byte aligned */
};

typedef struct bankshot2_journal {
	__le64     base;
	__le32     size;
	__le32     head;
	/* the next three fields must be in the same order and together.
	 * tail and gen_id must fall in the same 8-byte quadword */
	__le32     tail;
	__le16     gen_id;   /* generation id of the log */
	__le16     pad;
	__le16     redo_logging;
} bankshot2_journal_t;

/* persistent data structure to describe a single log-entry */
/* every log entry is max CACHELINE_SIZE bytes in size */
typedef struct {
	__le64   addr_offset;
	__le32   transaction_id;
	__le16   gen_id;
	u8       type;  /* normal, commit, or abort */
	u8       size;
	char     data[48];
} bankshot2_logentry_t;

/* volatile data structure to describe a transaction */
typedef struct bankshot2_transaction {
	u32              transaction_id;
	u16              num_entries;
	u16              num_used;
	u16              gen_id;
	u16              status;
	bankshot2_journal_t  *t_journal;
	bankshot2_logentry_t *start_addr;
	struct bankshot2_transaction *parent;
} bankshot2_transaction_t;

/*
 * Structure of the super block in PMFS
 * The fields are partitioned into static and dynamic fields. The static fields
 * never change after file system creation. This was primarily done because
 * pmfs_get_block() returns NULL if the block offset is 0 (helps in catching
 * bugs). So if we modify any field using journaling (for consistency), we 
 * will have to modify s_sum which is at offset 0. So journaling code fails.
 * This (static+dynamic fields) is a temporary solution and can be avoided
 * once the file system becomes stable and pmfs_get_block() returns correct
 * pointers even for offset 0.
 */
struct bankshot2_super_block {
	/* static fields. they never change after file system creation.
	 * checksum only validates up to s_start_dynamic field below */
	__le16		s_sum;              /* checksum of this sb */
	__le16		s_magic;            /* magic signature */
	__le32		s_blocksize;        /* blocksize in bytes */
	__le64		s_size;             /* total size of fs in bytes */
	char		s_volume_name[16];  /* volume name */
	/* points to the location of pmfs_journal_t */
	__le64          s_journal_offset;
	/* points to the location of struct pmfs_inode for the inode table */
	__le64          s_inode_table_offset;

	__le64  	s_start_dynamic; 

	/* all the dynamic fields should go here */
	/* s_mtime and s_wtime should be together and their order should not be
	 * changed. we use an 8 byte write to update both of them atomically */
	__le32		s_mtime;            /* mount time */
	__le32		s_wtime;            /* write time */
	/* fields for fast mount support. Always keep them together */
	__le64		s_num_blocknode_allocated;
	__le64		s_num_free_blocks;
	__le32		s_inodes_count;
	__le32		s_free_inodes_count;
	__le32		s_inodes_used_count;
	__le32		s_free_inode_hint;
};

struct bankshot2_blocknode {
	struct list_head link;
	unsigned long block_low;
	unsigned long block_high;
};

/* job part */

#define STATUS(flag)	((uint8_t)(1 << flag))

struct cache_stats{
	atomic_t hitcount;
	atomic_t misscount;
	atomic_t dirtycount;
	atomic_t cleancount;
	atomic_t evict_count;
	atomic_t evict_chunk_count;
	atomic_t release_chunk_count;
	atomic_t async_wb_chunk_count;
	atomic_t async_release_chunk_count;
	atomic_t write_back;
	atomic_t system_reads;
	atomic_t system_writes;
	atomic64_t system_writes_bytes;
	atomic_t protmiss;
	atomic_t sync_queued_count;
	uint64_t async_wb_blocks;
	uint64_t async_cleaned_blocks;
	int async_triggered;
	atomic_t sync_eviction_triggered;
	
};

typedef enum {
	WAKEUP_ON_COMPLETION=0,
	DO_COMPLETION=1, /*End of job. Could be endio for read or write  */	
	SYS_BIO=2, /*use orig bio to call endio and destroy job descriptor*/
	SYS_BIO_LAST=3, /* we call endio on sys bio only once*/
	FREE_ON_COMPLETION=4
}JOB_TYPE;

/* Valid status flags bit offsets - used to track request */
#define JOB_QUEUED_TO_DISK	1 /* The job is added to the backing store list */
#define JOB_ISSUED_TO_DISK	2 /* The job as been submitted to the backing store */
#define JOB_QUEUED_TO_CACHE	3 /* THe job has been queued for cache io */ 
#define JOB_ISSUED_TO_CACHE	4 /* The job has been submitted to cache */ 
#define JOB_DONE		5  /* The job has completed */ 
#define JOB_ERROR		6 /* The job has error */
#define JOB_ABORT 		7 /* Say the cached block is accessed again, we abort eviction */

struct job_descriptor{
	struct list_head store_queue;  /* This is the queue on which the job is placed before issuing. can be backing store or cache queue. Null if no queue */ 
	struct list_head jobs; /* List of jobs that are issued by the thread. Used to track completion */
	struct bankshot2_device *bs2_dev;
	struct task_struct *job_parent; /* Pointer to the task struct of the thread that initiated this job */
	uint64_t b_offset;
	uint64_t c_offset;
	size_t num_bytes; 
	atomic_t status; /* use atomic operations to set the status of the job queue. Only when the list is global we need locks. I use mb() when operating on it */
//	uint8_t moneta_cmd;
	unsigned long disk_cmd;
	JOB_TYPE type; 
	struct bio *bio;	
	struct bio *sys_bio;
};

struct bankshot2_device {
//	int		brd_number;
//	int		brd_refcnt;
//	loff_t		brd_offset;
//	loff_t		brd_sizelimit;
//	unsigned	brd_blocksize;

//	struct request_queue	*brd_queue;
//	struct gendisk		*brd_disk;
//	struct list_head	brd_list;

	void *virt_addr;
	uint32_t jsize;
	unsigned long num_inodes;
	unsigned long blocksize;
	unsigned long s_blocksize_bits;
	unsigned long phys_addr;
	unsigned long size;
	unsigned long block_start;
	unsigned long block_end;
	unsigned long num_free_blocks;
	unsigned long num_blocknode_allocated;
	kuid_t	uid;
	kgid_t	gid;
	umode_t	mode;
	struct list_head block_inuse_head;
	struct mutex s_lock;
	struct mutex inode_table_mutex;
	unsigned int	s_inodes_count;  /* total inodes count (used or free) */
	unsigned int	s_free_inodes_count;    /* free inodes count */
	unsigned int	s_inodes_used_count;
	unsigned int	s_free_inode_hint;

	struct kmem_cache *bs2_blocknode_cachep;

	struct cdev chardev;
	dev_t chardevnum;

	struct block_device *bs_bdev;
	struct request_queue	*backing_store_rqueue;
	struct bio_set *bio_set;
	struct kmem_cache *job_descriptor_slab;

	struct cache_stats cache_stats;
	struct list_head disk_queue;
	struct list_head cache_queue;

	atomic_t io_limit;
	spinlock_t io_queue_lock;

	int major;
	struct request_queue *queue;
	struct gendisk *gd;
	struct block_device *self_bdev;
	uint64_t bs_sects;

	/*
	 * Backing store of pages and lock to protect it. This is the contents
	 * of the block device.
	 */
//	spinlock_t		brd_lock;
//	struct radix_tree_root	brd_pages;
//	struct brd_cache_info *cache_info;
};

/* ========================= Methods =================================== */

static inline struct bankshot2_super_block *
bankshot2_get_super(struct bankshot2_device *bs2_dev)
{
	return (struct bankshot2_super_block *)bs2_dev->virt_addr;
}

/* If this is part of a read-modify-write of the block,
 * pmfs_memunlock_block() before calling! */
static inline void *bankshot2_get_block(struct bankshot2_device *bs2_dev,
					u64 block)
{
	struct bankshot2_super_block *ps = bankshot2_get_super(bs2_dev);

	return block ? ((void *)ps + block) : NULL;
}

static inline u64
bankshot2_get_block_off(struct bankshot2_device *bs2_dev,
			unsigned long blocknr, unsigned short btype)
{
	return (u64)blocknr << PAGE_SHIFT;
}

static inline unsigned long
bankshot2_get_numblocks(unsigned short btype)
{
//	unsigned long num_blocks;

/*
	if (btype == PMFS_BLOCK_TYPE_4K) {
		num_blocks = 1;
	} else if (btype == PMFS_BLOCK_TYPE_2M) {
		num_blocks = 512;
	} else {
		//btype == PMFS_BLOCK_TYPE_1G 
		num_blocks = 0x40000;
	}
	return num_blocks;
*/
	return 1;
}

static inline unsigned long bankshot2_get_pfn(struct bankshot2_device *bs2_dev,
						u64 block)
{
	return (bs2_dev->phys_addr + block) >> PAGE_SHIFT;
}

static inline void bankshot2_flush_buffer(void *buf, uint32_t len, bool fence)
{
	uint32_t i;
	len = len + ((unsigned long)(buf) & (CACHELINE_SIZE - 1));
	for (i = 0; i < len; i += CACHELINE_SIZE)
		asm volatile ("clflush %0\n" : "+m" (*(char *)(buf+i)));
	/* Do a fence only if asked. We often don't need to do a fence
	 * immediately after clflush because even if we get context switched
	 * between clflush and subsequent fence, the context switch operation
	 * provides implicit fence. */
	if (fence)
		asm volatile ("sfence\n" : : );
}

static inline u64 __bankshot2_find_data_block(struct bankshot2_device *bs2_dev,
		struct bankshot2_inode *pi, unsigned long blocknr)
{
	__le64 *level_ptr;
	u64 bp = 0;
	u32 height, bit_shift;
	unsigned int idx;

	height = pi->height;
	bp = le64_to_cpu(pi->root);

	while (height > 0) {
		level_ptr = bankshot2_get_block(bs2_dev, bp);
		bit_shift = (height - 1) * META_BLK_SHIFT;
		idx = blocknr >> bit_shift;
		bp = le64_to_cpu(level_ptr[idx]);
		if (bp == 0)
			return 0;
		blocknr = blocknr & ((1 << bit_shift) - 1);
		height--;
	}
	return bp;
}

static inline void bankshot2_update_isize(struct bankshot2_inode *pi,
						u64 new_size)
{
//	pmfs_memunlock_inode(inode->i_sb, pi);
	pi->i_size = cpu_to_le64(new_size);
//	pmfs_memlock_inode(inode->i_sb, pi);
}

/* assumes the length to be 4-byte aligned */
static inline void memset_nt(void *dest, uint32_t dword, size_t length)
{
	uint64_t dummy1, dummy2;
	uint64_t qword = ((uint64_t)dword << 32) | dword;

	asm volatile ("movl %%edx,%%ecx\n"
		"andl $63,%%edx\n"
		"shrl $6,%%ecx\n"
		"jz 9f\n"
		"1:      movnti %%rax,(%%rdi)\n"
		"2:      movnti %%rax,1*8(%%rdi)\n"
		"3:      movnti %%rax,2*8(%%rdi)\n"
		"4:      movnti %%rax,3*8(%%rdi)\n"
		"5:      movnti %%rax,4*8(%%rdi)\n"
		"8:      movnti %%rax,5*8(%%rdi)\n"
		"7:      movnti %%rax,6*8(%%rdi)\n"
		"8:      movnti %%rax,7*8(%%rdi)\n"
		"leaq 64(%%rdi),%%rdi\n"
		"decl %%ecx\n"
		"jnz 1b\n"
		"9:     movl %%edx,%%ecx\n"
		"andl $7,%%edx\n"
		"shrl $3,%%ecx\n"
		"jz 11f\n"
		"10:     movnti %%rax,(%%rdi)\n"
		"leaq 8(%%rdi),%%rdi\n"
		"decl %%ecx\n"
		"jnz 10b\n"
		"11:     movl %%edx,%%ecx\n"
		"shrl $2,%%ecx\n"
		"jz 12f\n"
		"movnti %%eax,(%%rdi)\n"
		"12:\n"
		: "=D"(dummy1), "=d" (dummy2) : "D" (dest), "a" (qword), "d" (length) : "memory", "rcx");
}


#if 0
int submit_bio_to_cache(struct brd_device *brd, struct bio *bio);
int brd_cache_open_backing_dev(struct block_device **bdev,
					char* backing_dev_name,
					struct brd_device* brd);
int brd_cache_init(struct brd_device *brd, struct block_device* bdev);
void brd_cache_exit(struct brd_device *brd);
#endif

/* ========================= Interfaces =================================== */

/* bankshot2_char.c */
int bankshot2_init_char(struct bankshot2_device *);
void bankshot2_destroy_char(struct bankshot2_device *);

/* bankshot2_cache.c */
int bankshot2_ioctl_cache_data(struct bankshot2_device *, void *);
int bankshot2_init_cache(struct bankshot2_device *, char *);

/* bankshot2_io.c */
int bankshot2_init_job_queue(struct bankshot2_device *);
void bankshot2_destroy_job_queue(struct bankshot2_device *);
void bankshot2_reroute_bio(struct bankshot2_device *bs2_dev, int idx,
				size_t sector, size_t size,
				struct bio *bio, struct block_device *bdev,
				int where, JOB_TYPE type);

/* bankshot2_block.c */
int bankshot2_init_block(struct bankshot2_device *);
void bankshot2_destroy_block(struct bankshot2_device *);

/* bankshot2_mem.c */
int bankshot2_init_kmem(struct bankshot2_device *);
void bankshot2_destroy_kmem(struct bankshot2_device *);
int bankshot2_init_blockmap(struct bankshot2_device *, unsigned long);
int __bankshot2_alloc_blocks(bankshot2_transaction_t *trans,
	struct bankshot2_device *bs2_dev, struct bankshot2_inode *pi,
	unsigned long file_blocknr, unsigned int num, bool zero);
int bankshot2_alloc_blocks(bankshot2_transaction_t *trans,
		struct bankshot2_device *bs2_dev, struct bankshot2_inode *pi,
		unsigned long file_blocknr, unsigned int num, bool zero);
int bankshot2_new_block(struct bankshot2_device *bs2_dev,
		unsigned long *blocknr, unsigned short btype, int zero);

/* bankshot2_inode.c */
int bankshot2_init_inode_table(struct bankshot2_device *);
struct bankshot2_inode *bankshot2_get_inode(struct bankshot2_device *bs2_dev,
						u64 ino);
u64 bankshot2_find_data_block(struct bankshot2_device *bs2_dev,
			struct bankshot2_inode *pi, unsigned long file_blocknr);

/* bankshot2_super.c */
int bankshot2_init_super(struct bankshot2_device *,
				unsigned long, unsigned long);
void bankshot2_destroy_super(struct bankshot2_device *);

/* bankshot2_xip.c */
int bankshot2_get_xip_mem(struct bankshot2_device *bs2_dev,
			struct bankshot2_inode *pi, pgoff_t pgoff, int create,
			void **kmem, unsigned long *pfn);

/* Mmap code */
unsigned long bankshot2_do_mmap_pgoff(struct bankshot2_device *bs2_dev,
			struct file *file, unsigned long addr,
			unsigned long len, unsigned long prot,
			unsigned long flags, unsigned long pgoff,
			unsigned long *populate);
unsigned long bankshot2_vm_mmap_pgoff(struct bankshot2_device *bs2_dev,
	struct file *file, unsigned long addr,
	unsigned long len, unsigned long prot,
	unsigned long flag, unsigned long pgoff);
unsigned long bankshot2_mmap_region(struct bankshot2_device *bs2_dev,
		struct file *file, unsigned long addr,
		unsigned long len, vm_flags_t vm_flags, unsigned long pgoff);

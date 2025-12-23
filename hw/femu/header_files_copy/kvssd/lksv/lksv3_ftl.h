#ifndef __FEMU_LKSV3_FTL_H
#define __FEMU_LKSV3_FTL_H

#include <assert.h>
#include <execinfo.h>
#include "hw/femu/kvssd/nand.h"
#include "hw/femu/kvssd/latency.h"
#include "hw/femu/kvssd/settings.h"
#define XXH_INLINE_ALL
#include "hw/femu/kvssd/xxhash.h"
#include "hw/femu/kvssd/kv_types.h"
#include "hw/femu/kvssd/utils.h"
#include "hw/femu/kvssd/skiplist.h"
#include "hw/femu/kvssd/cache.h"
#include "hw/femu/kvssd/lksv/cache.h"
#include "hw/femu/kvssd/lsm.h"

extern struct lksv_ssd      *lksv_ssd;
extern struct lksv3_lsmtree *lksv_lsm;

/* DO NOT EDIT */
#define HASH_BYTES 4
#define LEVELLIST_HASH_BYTES 2
#define LEVELLIST_HASH_SHIFTS 16
#define PG_N 32

/*
 * If commented, the simulator runs in OURS+ mode, where values are placed
 * back into the log area during the compaction process.
 */
//#define OURS

struct lksv_ssd {
    char *ssdname;
    struct ssdparams sp;
    struct ssd_channel *ch;
    struct write_pointer wp;
    struct line_mgmt lm;

    /* lockless ring for communication with NVMe IO thread */
    struct rte_ring **to_ftl;
    struct rte_ring **to_poller;
    bool *dataplane_started_ptr;
    QemuThread ftl_thread;

    FemuCtrl *n;
    bool do_reset;
    bool start_log;

    struct kvssd_latency lat;
    const struct kv_lsm_operations *lops;
};

typedef struct per_line_data {
    int        referenced_by_memtable;
    int        referenced_by_files;
    GHashTable *files[LSM_LEVELN];
} per_line_data;

#define per_line_data(line) ((per_line_data *)((line)->private))

/* Functions found in lksv_ftl.c */
uint64_t lksv3_ssd_advance_status(struct femu_ppa *ppa, struct nand_cmd *ncmd);

typedef struct {
    uint32_t   *hashes;
    uint16_t    n;
} lksv_hash_list;

/*
 * The LSM-tree maintains an in-memory data structure that points to runs of
 * levels in the flash. Each run contains a header that holds the locations of
 * KV objects (KV indices) in the flash.
 */
typedef struct lksv_level_list_entry
{
    uint64_t        id;

    kv_key          smallest;
    kv_key          largest;
    struct femu_ppa ppa;

    // not null == cached
    kv_cache_entry  *cache[CACHE_TYPES];

    lksv_hash_list  hash_lists[PG_N];
    uint16_t        pg_start_hashes[PG_N];
    int             hash_list_n;

    // raw format of meta segment. (page size)
    char            *buffer[PG_N];

    int             ref_count;

    // TODO: Change this to bitmap.
    // TODO: 512 to const number.
    bool            value_log_rmap[512];
    uint8_t         level;
} lksv_level_list_entry;

#define LEVEL_LIST_ENTRY_PER_PAGE (PAGESIZE/(32+(PG_N*LEVELLIST_HASH_BYTES)+20))

#define RUNINPAGE (PAGESIZE/(AVGKEYLENGTH+(LEVELLIST_HASH_BYTES*PG_N)+20))
#define MAXKEYINMETASEG ((PAGESIZE - 1024)/(AVGKEYLENGTH+PPA_LENGTH))
#define KEYFORMAT(input) input.len>AVGKEYLENGTH?AVGKEYLENGTH:input.len,input.key

#define KEYBITMAP (PAGESIZE / 16)
#define KEYLEN(a) (a.len)

bool lksv3_should_meta_gc_high(int margin);
bool lksv3_should_data_gc_high(int margin);

enum lksv3_sst_meta_flag {
    KEY_VALUE_PAIR = 0,
    KEY_PPA_PAIR   = 1,
    VALUE_LOG      = 2,
};

#define LKSV3_SSTABLE_FOOTER_BLK_SIZE 16
#define LKSV3_SSTABLE_META_BLK_SIZE 16
#define LKSV3_SSTABLE_STR_IDX_SIZE 4
/*
 * --------------------------------------
 * Page Layout
 * --------------------------------------
 * Data blocks: N bytes (less than 2048)
 *  NB: Key + value
 * --------------------------------------
 * Meta blocks: 16 bytes
 *  2B: data block offset
 *  1B: key length
 *  1B: flags
 *  4B: hash key
 *  1B: shard id
 *  1B: total shard numbers
 *  2B: shard length
 *  2B: value log offset
 *  2B: reserved space
 * --------------------------------------
 * Footer block: 16 bytes
 *  2B: number of KV pairs
 *  2B: start key meta index (for range query)
 *  2B: end key meta index (for range query)
 * 10B: reserved space
 * --------------------------------------
 */
typedef struct lksv_block_meta {
    union {
        struct {
            uint64_t off    : 16;
            uint64_t klen   : 8;
            uint64_t flag   : 8;
            uint64_t hash   : 32;
        } g1;
        uint64_t m1;
    };
    union {
        struct {
            uint64_t sid    : 8;
            uint64_t snum   : 8;
            uint64_t slen   : 16;
            uint64_t voff   : 16;
            uint64_t rsv    : 16;
        } g2;
        uint64_t m2;
    };
} lksv_block_meta;

typedef struct lksv_block_footer {
    union {
        struct {
            uint64_t n      : 16;
            uint64_t skey   : 16;
            uint64_t ekey   : 16;
            uint64_t rsv    : 16;
        } g;
        uint64_t f;
    };
    uint64_t level_list_entry_id;
} lksv_block_footer;

// compaction.h ==============================================

void lksv_compaction_init(void);
void lksv_maybe_schedule_compaction(void);

// lsmtree.h ================================================

typedef struct lksv_version {
    lksv_level_list_entry **files[LSM_LEVELN];
    int                   n_files[LSM_LEVELN];
    int                   m_files[LSM_LEVELN];

    double                compaction_score;
    int                   compaction_level;
} lksv_version;

typedef struct lksv3_lsmtree {
    struct kv_lsm_options *opts;

    uint8_t bottom_level; // Indicates the current bottom level index.
    uint8_t LEVELCACHING;

    struct kv_skiplist *mem;
    struct kv_skiplist *imm;
    struct kv_skiplist *key_only_mem;
    struct kv_skiplist *key_only_imm;
    QemuMutex mu;
    QemuThread comp_thread;
    bool compacting;
    bool compacting_meta_lines[2][1024];
    uint64_t compaction_calls;

    // TODO: introduce multi-versions.
    lksv_version versions;

    struct kv_cache *lsm_cache;

    int header_gc_cnt;
    int data_gc_cnt;

    GHashTable *level_list_entries;
    pthread_spinlock_t level_list_entries_lock;
    uint64_t next_level_list_entry_id;
} lksv3_lsmtree;

// ftl.h =====================================================

void lksv3ssd_init(FemuCtrl *n);

struct femu_ppa lksv3_get_new_meta_page(void);
struct femu_ppa lksv3_get_new_data_page(void);
struct nand_page *lksv3_get_pg(struct femu_ppa *ppa);
struct line *lksv3_get_line(struct femu_ppa *ppa);
struct line *lksv3_get_next_free_line(struct line_partition *lm);
void lksv3_ssd_advance_write_pointer(struct line_partition *lm);
void lksv3_mark_page_invalid(struct femu_ppa *ppa);
void lksv3_mark_page_valid(struct femu_ppa *ppa);
void lksv3_mark_block_free(struct femu_ppa *ppa);
struct line *lksv3_select_victim_meta_line(bool force);
struct line *lksv3_select_victim_data_line(bool force);
void lksv3_mark_line_free(struct femu_ppa *ppa);
struct nand_lun *lksv3_get_lun(struct femu_ppa *ppa);

static inline bool check_voffset(struct femu_ppa *ppa, int voff, uint32_t hash)
{
    struct nand_page *pg = lksv3_get_pg(ppa);
    int offset = PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE - (LKSV3_SSTABLE_META_BLK_SIZE * (voff + 1));
    lksv_block_meta meta = *(lksv_block_meta *) (pg->data + offset);

    return meta.g1.hash == hash;
}

void lksv_open(struct kv_lsm_options *opts);

// version.c

void lksv_lput(lksv_level_list_entry *e);
lksv_level_list_entry *lksv_lget(uint64_t id);
lksv_level_list_entry *lksv_lnew(void);
void lksv_update_compaction_score(void);
lksv_level_list_entry **lksv_overlaps(int level, kv_key smallest, kv_key largest, int *n);

typedef struct lksv_compaction {
    bool                  log_triggered;
    int                   log_triggered_line_id;
    int                   level;
    int                   input_n[2];
    lksv_level_list_entry **inputs[2];
} lksv_compaction;

typedef struct lksv_kv_descriptor {
    kv_key          key;
    kv_value        value;
    uint32_t        hash;
    struct femu_ppa ppa;
    int             value_log_offset;
    int             str_order; // Used by compaction.
} lksv_kv_descriptor;

typedef struct lksv_value_log_writer {
    struct femu_ppa writing_ppa;
    int             left;
    int             wp;
    int             n;
} lksv_value_log_writer;

lksv_value_log_writer *lksv_new_value_log_writer(void);
void lksv_set_value_log_writer(lksv_value_log_writer *w, lksv_kv_descriptor *d);
void lksv_close_value_log_writer(lksv_value_log_writer *w);

typedef struct lksv_file_iterator {
    int level;

    lksv_level_list_entry **files;
    int                   files_n;
    int                   current_file;

    lksv_kv_descriptor    *buffer[8192];
    int                   buffer_n;
    int                   current_buffer;

    bool upper;
    bool fetch_values;
    int fetch_line_id;
#ifndef OURS
    bool reinsert_values;
#endif
} lksv_file_iterator; 

typedef struct lksv_data_seg_writer {
    struct femu_ppa    writing_ppa;
    lksv_kv_descriptor *buffer[8192];
    int                left;
    int                n;

    lksv_level_list_entry **results;
    int                   *result_n;

    int level;
} lksv_data_seg_writer;

void lksv_write_level0_table(kv_skiplist *mem);
void lksv_write_level123_table(lksv_compaction *c);

static inline struct femu_ppa get_next_write_ppa(struct femu_ppa pivot, int offset) {
    kv_assert(pivot.g.ch == 0);
    kv_assert(offset < 64);

    int ch_offset = offset % lksv_ssd->sp.nchs;
    int lun_offset = (offset / lksv_ssd->sp.nchs) % lksv_ssd->sp.luns_per_ch;

    pivot.g.ch += ch_offset;
    pivot.g.lun += lun_offset;

    return pivot;
}

int lksv_gc_meta_femu(void);
void lksv_gc_meta_erase_only(void);
void lksv_gc_data_femu(int lineid);

static inline bool is_pivot_ppa(struct femu_ppa ppa) {
    int pg_n = ppa.g.lun * lksv_ssd->sp.nchs;
    if (ppa.g.ch == 0 && pg_n % PG_N == 0) {
        return true;
    }
    return false;
}

kv_value *lksv_get(kv_key k, NvmeRequest *req);
kv_value *internal_get(int eid, kv_key k, uint32_t hash, NvmeRequest *req);

void lksv_comp_read_delay(struct femu_ppa *ppa);
void lksv_comp_write_delay(struct femu_ppa *ppa);
void lksv_user_read_delay(struct femu_ppa *ppa, NvmeRequest *req);

void lksv_bucket_sort(lksv_kv_descriptor **buffer, int n);

#endif

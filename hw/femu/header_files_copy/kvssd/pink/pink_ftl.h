#ifndef __FEMU_PINK_FTL_H
#define __FEMU_PINK_FTL_H

#include <assert.h>
#include "hw/femu/kvssd/nand.h"
#include "hw/femu/kvssd/latency.h"
#include "hw/femu/kvssd/settings.h"
#include "hw/femu/kvssd/kv_types.h"
#include "hw/femu/kvssd/utils.h"
#include "hw/femu/kvssd/skiplist.h"
#include "hw/femu/kvssd/cache.h"
#include "hw/femu/kvssd/pink/cache.h"
#include "hw/femu/kvssd/lsm.h"

extern struct pink_ssd     *pink_ssd;
extern struct pink_lsmtree *pink_lsm;

// ftl.h ===================================================

//#define CACHE_UPDATE

struct pink_ssd {
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
    pthread_spinlock_t nand_lock;

    FemuCtrl *n;
    bool do_reset;
    bool start_log;

    struct kvssd_latency lat;

    const struct kv_lsm_operations *lops;
};

uint64_t pink_ssd_advance_status(struct femu_ppa *ppa, struct nand_cmd *ncmd);

#define LINE_AGE_MAX 64

struct line_age {
    union {
        struct {
            uint16_t in_page_idx : 10;
            uint16_t line_age : 6;
        } g;
        uint16_t age;
    };
};

typedef struct pink_key_age {
    kv_key k;
    struct line_age line_age;
} pink_key_age;
#define PINK_KEYAGET pink_key_age

/*
 * The LSM-tree maintains an in-memory data structure that points to runs of
 * levels in the flash. Each run contains a header that holds the locations of
 * KV objects (KV indices) in the flash.
 */
typedef struct pink_level_list_entry
{
    uint64_t        id;

    kv_key          smallest;
    kv_key          largest;
    struct femu_ppa ppa;

    // not null == cached
    kv_cache_entry  *cache[CACHE_TYPES];

    // raw format of meta segment. (page size)
    char            *buffer;

    int             ref_count;
} pink_level_list_entry;

#define LEVEL_LIST_ENTRY_PER_PAGE (PAGESIZE/(AVGKEYLENGTH+PPA_LENGTH))
#define MAXKEYINMETASEG ((PAGESIZE - KEYBITMAP - VERSIONBITMAP)/(AVGKEYLENGTH+PPA_LENGTH))
#define KEYFORMAT(input) input.len>AVGKEYLENGTH?AVGKEYLENGTH:input.len,input.key

#define KEYBITMAP (PAGESIZE / 16)
#define VERSIONBITMAP (PAGESIZE / 16)
#define KEYLEN(a) (a.len+sizeof(struct femu_ppa))

bool pink_should_data_gc_high(int margin);
bool pink_should_meta_gc_high(int margin);

void pink_flush_cache_when_evicted(kv_cache_entry *ent);
void pink_maybe_schedule_compaction(void);
void pink_compaction_init(void);

typedef struct pink_version {
    pink_level_list_entry **files[LSM_LEVELN];
    int                   n_files[LSM_LEVELN];
    int                   m_files[LSM_LEVELN];

    double                compaction_score;
    int                   compaction_level;
} pink_version;

typedef struct pink_lsmtree {
    struct kv_lsm_options *opts;

    struct kv_skiplist *mem;
    struct kv_skiplist *imm;
    struct kv_skiplist *key_only_mem;
    struct kv_skiplist *key_only_imm;
    QemuMutex mu;
    QemuThread comp_thread;
    bool compacting;
    uint64_t compaction_calls;

    // TODO: introduce multi-versions.
    pink_version versions;

    struct kv_cache* lsm_cache;

    uint64_t num_data_written;
    uint64_t cache_hit;
    uint64_t cache_miss;
    int header_gc_cnt;
    int data_gc_cnt;

    bool should_d2m;

    GHashTable *level_list_entries;
    pthread_spinlock_t level_list_entries_lock;
    uint64_t next_level_list_entry_id;
} pink_lsmtree;

void pinkssd_init(FemuCtrl *n);

struct femu_ppa get_new_meta_page(void);
struct femu_ppa get_new_data_page(void);
struct nand_page *get_pg(struct femu_ppa *ppa);
struct line *get_line(struct femu_ppa *ppa);
struct line *get_next_free_line(struct line_partition *lm);
void ssd_advance_write_pointer(struct line_partition *lm);
void mark_sector_invalid(struct femu_ppa *ppa);
void mark_page_invalid(struct femu_ppa *ppa);
void mark_page_valid(struct femu_ppa *ppa);
void mark_block_free(struct femu_ppa *ppa);
struct line *select_victim_meta_line(bool force);
struct line *select_victim_data_line(bool force);
void mark_line_free(struct femu_ppa *ppa);

struct nand_lun *get_lun(struct femu_ppa *ppa);

void pink_open(struct kv_lsm_options *opts);

// version.c

void pink_lput(pink_level_list_entry *e);
pink_level_list_entry *pink_lget(uint64_t id);
pink_level_list_entry *pink_lnew(void);
void pink_update_compaction_score(void);

typedef struct pink_compaction {
    int                   level;
    int                   input_n[2];
    pink_level_list_entry **inputs[2];
} pink_compaction;

typedef struct pink_kv_descriptor {
    kv_key          key;
    kv_value        value;
    struct femu_ppa ppa;
    struct line_age data_seg_offset;
} pink_kv_descriptor;

typedef struct pink_data_seg_writer {
    struct femu_ppa writing_ppa;
    pink_kv_descriptor *buffer[8192];
    int             left;
    int             n;
} pink_data_seg_writer;

pink_data_seg_writer *pink_new_data_seg_writer(void);
void pink_set_data_seg_writer(pink_data_seg_writer *w, pink_kv_descriptor *d);
void pink_close_data_seg_writer(pink_data_seg_writer *w);

typedef struct pink_file_iterator {
    pink_level_list_entry **files;
    int                   files_n;
    int                   current_file;

    pink_kv_descriptor    *buffer[8192];
    int                   buffer_n;
    int                   current_buffer;
} pink_file_iterator; 

typedef struct pink_meta_seg_writer {
    struct femu_ppa    writing_ppa;
    pink_kv_descriptor *buffer[8192];
    int                left;
    int                n;

    pink_level_list_entry **results;
    int                   *result_n;

    int level;
} pink_meta_seg_writer;

void pink_comp_read_delay(struct femu_ppa *ppa);
void pink_comp_write_delay(struct femu_ppa *ppa);

void pink_write_level0_table(kv_skiplist *mem);
pink_level_list_entry **pink_overlaps(int level, kv_key smallest, kv_key largest, int *n);
void pink_lsm_adjust_level_multiplier(void);
void pink_write_level123_table(pink_compaction *c);
int pink_gc_meta_femu(void);
int pink_gc_data_femu(void);

kv_value *pink_internal_get(int eid, kv_key k1, NvmeRequest *req);
void pink_user_read_delay(struct femu_ppa *ppa, NvmeRequest *req);
kv_value *pink_get(kv_key k, NvmeRequest *req);

#endif

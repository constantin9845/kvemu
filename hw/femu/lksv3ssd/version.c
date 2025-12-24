#include <pthread.h>
#include "hw/femu/kvssd/lksv/lksv3_ftl.h"

/*
 * lput puts an level list entry, dropping its reference count. If the entry
 * reference count hits zero, the entry is then freed.
 */
void
lksv_lput(lksv_level_list_entry *e)
{
    pthread_spin_lock(&lksv_lsm->level_list_entries_lock);

    e->ref_count--;
    if (e->ref_count == 0) {
        kv_cache_delete_entry(lksv_lsm->lsm_cache, e->cache[LEVEL_LIST_ENTRY]);
        kv_cache_delete_entry(lksv_lsm->lsm_cache, e->cache[HASH_LIST]);
        kv_cache_delete_entry(lksv_lsm->lsm_cache, e->cache[DATA_SEGMENT_GROUP]);

        free(e->smallest.key);
        free(e->largest.key);

        for (int i = 0; i < e->hash_list_n; i++)
        {
            struct femu_ppa ppa = get_next_write_ppa(e->ppa, i);
            lksv3_mark_page_invalid(&ppa);

            free(e->hash_lists[i].hashes);
        }

        for (int i = 0; i < 512; i++)
        {
            if (!e->value_log_rmap[i])
                continue;

            struct line *l = &lksv_ssd->lm.lines[i];
            g_hash_table_remove(per_line_data(l)->files[e->level], &e->id);
            per_line_data(l)->referenced_by_files--;
        }

        g_hash_table_remove(lksv_lsm->level_list_entries, &e->id);
        free(e);
    }

    pthread_spin_unlock(&lksv_lsm->level_list_entries_lock);
}

/*
 * lget obtains an level list entry, increasing its reference count.
 */
lksv_level_list_entry *
lksv_lget(uint64_t id)
{
    lksv_level_list_entry *e;

    pthread_spin_lock(&lksv_lsm->level_list_entries_lock);

    e = g_hash_table_lookup(lksv_lsm->level_list_entries, &id);
    kv_assert(e);
    e->ref_count++;

    pthread_spin_unlock(&lksv_lsm->level_list_entries_lock);

    return e;
}

/*
 * lnew creates an level list entry.
 */
lksv_level_list_entry *
lksv_lnew(void)
{
    lksv_level_list_entry *e = calloc(1, sizeof(lksv_level_list_entry));

    e->id = qatomic_fetch_inc(&lksv_lsm->next_level_list_entry_id);
    e->ref_count++;

    g_hash_table_insert(lksv_lsm->level_list_entries, &e->id, e);

    return e;
}

void
lksv_user_read_delay(struct femu_ppa *ppa, NvmeRequest *req)
{
    struct nand_cmd srd;
    srd.type = USER_IO;
    srd.cmd = NAND_READ;

    if (req)
    {
        srd.stime = req->etime;
        req->flash_access_count++;
    }
    else
        srd.stime = 0;

    uint64_t sublat = lksv3_ssd_advance_status(ppa, &srd);

    if (req)
        req->etime += sublat;
}

void
lksv_update_compaction_score(void)
{
    double compaction_score;
    int i;

    lksv_lsm->versions.compaction_level = 0;
    lksv_lsm->versions.compaction_score = 0;

    for (i = 0; i < LSM_LEVELN-1; i++)
    {
        compaction_score = (double) lksv_lsm->versions.n_files[i] / (double) lksv_lsm->versions.m_files[i];

        if (compaction_score > lksv_lsm->versions.compaction_score)
        {
            lksv_lsm->versions.compaction_score = compaction_score;
            lksv_lsm->versions.compaction_level = i;
        }
    }
}

lksv_level_list_entry **
lksv_overlaps(int level, kv_key smallest, kv_key largest, int *n)
{
    lksv_level_list_entry **ret;
    lksv_level_list_entry *e;
    int i, start, end;

    start = -1;

    for (i = 0; i < lksv_lsm->versions.n_files[level]; i++)
    {
        e = lksv_lsm->versions.files[level][i];
        
        if (kv_cmp_key(e->largest, smallest) < 0)
            continue;

        if (kv_cmp_key(e->smallest, largest) > 0)
            break;

        if (start == -1)
            start = i;

        end = i;
    }

    if (start == -1)
    {
        *n = 0;
        return NULL;
    }

    *n = end - start + 1;

    ret = malloc((*n) * sizeof(lksv_level_list_entry *));

    for (i = start; i < end + 1; i++)
        ret[i-start] = lksv_lsm->versions.files[level][i];

    return ret;
}

static int
binary_search(int level, kv_key k, NvmeRequest *req)
{
    int end = lksv_lsm->versions.n_files[level];
    int start = 0;
    int mid, res;

    while (start < end)
    {
        mid = (start + end) / 2;

        if (!kv_is_cached(lksv_lsm->lsm_cache,
                          lksv_lsm->versions.files[level][mid]->cache[LEVEL_LIST_ENTRY]))
        {
            // TODO: Change the fake PPA to real one.
            const struct femu_ppa fake_pivot_ppa = { .ppa = 0 };
            struct femu_ppa fake_ppa = get_next_write_ppa(fake_pivot_ppa, mid % PG_N);

            lksv_user_read_delay(&fake_ppa, req);
        }

        res = kv_cmp_key(lksv_lsm->versions.files[level][mid]->largest, k);

        if (res < 0)
            start = mid + 1;
        else
            end = mid;
    }

    return start;
}

static bool
key_may_exist(lksv_level_list_entry *e, uint32_t hash)
{
    if (!kv_is_cached(lksv_lsm->lsm_cache, e->cache[HASH_LIST]))
        return true;

    int i;
    for (i = e->hash_list_n-1; i > 0; i--)
    {
        if (e->hash_lists[i].hashes[0] <= hash)
            break;
    }

    for (int j = 0; j < e->hash_lists[i].n; j++)
    {
        if (hash == e->hash_lists[i].hashes[j])
            return true;
    }

    return false;
}


kv_value *
lksv_get(kv_key k, NvmeRequest *req)
{
    kv_value *v = NULL;
    uint32_t hash = XXH32(k.key, k.len, 0); 

    for (int i = 0; i < LSM_LEVELN; i++)
    {
        if (lksv_lsm->versions.n_files[i] == 0) // check
            continue;

        int idx = binary_search(i, k, req); // find first file whose largest key >= k

        
        if (idx == lksv_lsm->versions.n_files[i])
            continue;

        lksv_level_list_entry *e = lksv_lsm->versions.files[i][idx];

        if (kv_cmp_key(e->smallest, k) > 0)
            continue;

        if (!key_may_exist(e, hash))
            continue;

        v = internal_get(e->id, k, hash, req);

        if (v)
            return v;
    }

    return v;
}


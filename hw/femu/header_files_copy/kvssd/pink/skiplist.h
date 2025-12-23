#ifndef __FEMU_PINK_SKIPLIST_H
#define __FEMU_PINK_SKIPLIST_H

#include "hw/femu/kvssd/skiplist.h"

#define snode_ppa(node) \
    (&((pink_per_snode_data*)(node->private))->ppa)
#define snode_off(node) \
    (&((pink_per_snode_data*)(node->private))->off)

typedef struct pink_per_snode_data {
    struct femu_ppa ppa;
    int             off;
} pink_per_snode_data;

kv_snode *pink_skiplist_insert(kv_skiplist *list, kv_key key, kv_value* value);

#endif

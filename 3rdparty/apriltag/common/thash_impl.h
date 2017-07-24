/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
// beware: The combination of:
//    1) a hash function that uses floating point values
//    2) that is inlined
//    3) and compiled with -Ofast
// can result in inconsistent values being computed. You can force the function
// NOT to be inlined with __attribute__ ((noinline)).
//
// It's also tempting to do:
//    #define TKEYEQUAL(pka, pkb) (!memcmp(pka, pkb, sizeof(my_key_tyep)))
//
// But this will not work as expected if the structure contains
// padding that is not consistently cleared to zero. It appears that
// in C99, copying a struct by value does NOT necessarily copy
// padding, and so it may be difficult to guarantee that padding is
// zero, even when the code otherwise appears sane.
//
// You can use the "performance" method to evaluate how well your hash
// function is doing.  Bad hash functions (obviously) are very bad for
// performance!

#define TNAME sm_points_hash
#define TVALTYPE struct sm_points_record
#define TKEYTYPE zarray_t*

// takes a pointer to the key
#define TKEYHASH(pk) ((uint32_t) (pk->level + (a->rad * 100) + (a->meters_per_pixel*100)))

// takes a pointer to the value
#define TKEYEQUAL(pka, pkb) (!memcmp(pka, pkb, sizeof(struct sm_points_record)))

*/

// when nentries/size is greater than _CRITICAL, we trigger a rehash.
#define THASH_FACTOR_CRITICAL 2

// when rehashing (or allocating with a target capacity), we use this ratio of nentries/size
// should be greater than THASH_FACTOR_CRITICAL
#define THASH_FACTOR_REALLOC 4

#define TRRFN(root, suffix) root ## _ ## suffix
#define TRFN(root, suffix) TRRFN(root, suffix)
#define TFN(suffix) TRFN(TNAME, suffix)

#define TTYPENAME TFN(t)

struct TFN(_entry)
{
    // XXX Better to store the hash value and to reserve a special
    // hash value to mean invalid?
    uint8_t  valid;
    TKEYTYPE key;
    TVALTYPE value;
};

typedef struct TTYPENAME TTYPENAME;
struct TTYPENAME
{
    struct TFN(_entry) *entries;
    int                 nentries;
    int                 size; // always a power of two.
};

// will allocate enough room so that size can grow to 'capacity'
// without rehashing.
static inline TTYPENAME *TFN(create_capacity)(int capacity)
{
    // must be this large to not trigger rehash
    int _nentries = THASH_FACTOR_REALLOC*capacity;
    if (_nentries < 8)
        _nentries = 8;

    // but must also be a power of 2
    int nentries = _nentries;
    if ((nentries & (nentries - 1)) != 0) {
        nentries = 8;
        while (nentries < _nentries)
            nentries *= 2;
    }

    assert((nentries & (nentries-1)) == 0);
    TTYPENAME *hash = calloc(1, sizeof(TTYPENAME));
    hash->nentries = nentries;
    hash->entries = calloc(hash->nentries, sizeof(struct TFN(_entry)));
    return hash;
}

static inline TTYPENAME *TFN(create)()
{
    return TFN(create_capacity)(8);
}

static inline void TFN(destroy)(TTYPENAME *hash)
{
    if (!hash)
        return;

    free(hash->entries);
    free(hash);
}

static inline int TFN(size)(TTYPENAME *hash)
{
    return hash->size;
}

static inline void TFN(clear)(TTYPENAME *hash)
{
    // could just clear the 'valid' flag.
    memset(hash->entries, 0, hash->nentries * sizeof(struct TFN(_entry)));
    hash->size = 0;
}

// examine the performance of the hashing function by looking at the distribution of bucket->size
static inline void TFN(performance)(TTYPENAME *hash)
{
    int runs_sz = 32;
    int runs[runs_sz];
    int cnt = 0;
    int max_run = 0;
    int min_run = hash->size;
    int run1 = 0;
    int run2 = 0;

    memset(runs, 0, sizeof(runs));

    for (int entry_idx = 0; entry_idx < hash->nentries; entry_idx++) {
        if (!hash->entries[entry_idx].valid)
            continue;

        int this_run = 0;
        while (hash->entries[(entry_idx+this_run) & (hash->nentries - 1)].valid)
            this_run++;
        if (this_run < runs_sz)
            runs[this_run]++;
        if (this_run < min_run)
            min_run = this_run;
        if (this_run > max_run)
            max_run = this_run;

        run1 += this_run;
        run2 += this_run*this_run;
        cnt++;
    }

    double Ex1 = 1.0 * run1 / cnt;
    double Ex2 = 1.0 * run2 / cnt;

#define strr(s) #s
#define str(s) strr(s)
    printf("%s: size %8d, nentries: %8d, min %3d, max %3d, mean %6.3f, stddev %6.3f\n",
           str(TNAME),
           hash->size, hash->nentries, min_run, max_run, Ex1, sqrt(Ex2 - Ex1*Ex1));
}

static inline int TFN(get_volatile)(TTYPENAME *hash, TKEYTYPE *key, TVALTYPE **value)
{
    uint32_t code = TKEYHASH(key);
    uint32_t entry_idx = code & (hash->nentries - 1);

    while (hash->entries[entry_idx].valid) {
        if (TKEYEQUAL(key, &hash->entries[entry_idx].key)) {
            *value = &hash->entries[entry_idx].value;
            return 1;
        }

        entry_idx = (entry_idx + 1) & (hash->nentries - 1);
    }

    return 0;
}

static inline int TFN(get)(TTYPENAME *hash, TKEYTYPE *key, TVALTYPE *value)
{
    // XXX see implementation in zhash.c (implement in terms of
    // get_volatile)

    uint32_t code = TKEYHASH(key);
    uint32_t entry_idx = code & (hash->nentries - 1);

    while (hash->entries[entry_idx].valid) {
        if (TKEYEQUAL(key, &hash->entries[entry_idx].key)) {
            *value = hash->entries[entry_idx].value;
            return 1;
        }

        entry_idx = (entry_idx + 1) & (hash->nentries - 1);
    }

    return 0;
}

static inline int TFN(put)(TTYPENAME *hash, TKEYTYPE *key, TVALTYPE *value, TKEYTYPE *oldkey, TVALTYPE *oldvalue)
{
    uint32_t code = TKEYHASH(key);
    uint32_t entry_idx = code & (hash->nentries - 1);

    while (hash->entries[entry_idx].valid) {
        if (TKEYEQUAL(key, &hash->entries[entry_idx].key)) {
            if (oldkey)
                *oldkey   = hash->entries[entry_idx].key;
            if (oldvalue)
                *oldvalue = hash->entries[entry_idx].value;
            hash->entries[entry_idx].key = *key;
            hash->entries[entry_idx].value = *value;
            return 1;
        }

        entry_idx = (entry_idx + 1) & (hash->nentries - 1);
    }

    hash->entries[entry_idx].valid = 1;
    hash->entries[entry_idx].key = *key;
    hash->entries[entry_idx].value = *value;
    hash->size++;

    if (hash->nentries < THASH_FACTOR_CRITICAL*hash->size) {
//        printf("rehash: \n   before: ");
//        TFN(performance)(hash);

        // rehash!
        TTYPENAME *newhash = TFN(create_capacity)(hash->size + 1);

        for (int entry_idx = 0; entry_idx < hash->nentries; entry_idx++) {
            if (hash->entries[entry_idx].valid) {

                if (TFN(put)(newhash, &hash->entries[entry_idx].key, &hash->entries[entry_idx].value, NULL, NULL))
                    assert(0); // shouldn't already be present.
            }
        }

        // play switch-a-roo. We become 'newhash' and free the old one.
        TTYPENAME tmp;
        memcpy(&tmp, hash, sizeof(TTYPENAME));
        memcpy(hash, newhash, sizeof(TTYPENAME));
        memcpy(newhash, &tmp, sizeof(TTYPENAME));
        TFN(destroy)(newhash);

//        printf("   after : ");
//        TFN(performance)(hash);
    }

    return 0;
}

static inline int TFN(remove)(TTYPENAME *hash, TKEYTYPE *key, TKEYTYPE *oldkey, TVALTYPE *oldvalue)
{
    uint32_t code = TKEYHASH(key);
    uint32_t entry_idx = code & (hash->nentries - 1);

    while (hash->entries[entry_idx].valid) {
        if (TKEYEQUAL(key, &hash->entries[entry_idx].key)) {

            if (oldkey)
                *oldkey = hash->entries[entry_idx].key;
            if (oldvalue)
                *oldvalue = hash->entries[entry_idx].value;

            hash->entries[entry_idx].valid = 0;
            hash->size--;

            // re-put following entries
            entry_idx = (entry_idx + 1) & (hash->nentries - 1);
            while (hash->entries[entry_idx].valid) {
                TKEYTYPE key = hash->entries[entry_idx].key;
                TVALTYPE value = hash->entries[entry_idx].value;
                hash->entries[entry_idx].valid = 0;
                hash->size--;

                if (TFN(put)(hash, &key, &value, NULL, NULL)) {
                    assert(0);
                }

                entry_idx = (entry_idx + 1) & (hash->nentries - 1);
            }

            return 1;
        }

        entry_idx = (entry_idx + 1) & (hash->nentries - 1);
    }

    return 0;
}

static inline TTYPENAME *TFN(copy)(TTYPENAME *hash)
{
    TTYPENAME *newhash = TFN(create_capacity)(hash->size);

    for (int entry_idx = 0; entry_idx < hash->nentries; entry_idx++) {
        if (hash->entries[entry_idx].valid) {
            if (TFN(put)(newhash, &hash->entries[entry_idx].key, &hash->entries[entry_idx].value, NULL, NULL))
                assert(0); // shouldn't already be present.
        }
    }

    return newhash;
}

typedef struct TFN(iterator) TFN(iterator_t);
struct TFN(iterator)
{
    TTYPENAME *hash;
    int last_entry; // points to last entry returned by _next
};

static inline void TFN(iterator_init)(TTYPENAME *hash, TFN(iterator_t) *iter)
{
    iter->hash = hash;
    iter->last_entry = -1;
}

static inline int TFN(iterator_next)(TFN(iterator_t) *iter, TKEYTYPE *outkey, TVALTYPE *outval)
{
    TTYPENAME *hash = iter->hash;

    while(1) {
        if (iter->last_entry+1 >= hash->nentries)
            return 0;

        iter->last_entry++;

        if (hash->entries[iter->last_entry].valid) {
            if (outkey)
                *outkey = hash->entries[iter->last_entry].key;
            if (outval)
                *outval = hash->entries[iter->last_entry].value;
            return 1;
        }
    }
}

static inline void TFN(iterator_remove)(TFN(iterator_t) *iter)
{
    TTYPENAME *hash = iter->hash;

    hash->entries[iter->last_entry].valid = 0;

    // have to reinsert any consecutive entries that follow.
    int entry_idx = (iter->last_entry + 1) & (hash->nentries - 1);
    while (hash->entries[entry_idx].valid) {
        TKEYTYPE key = hash->entries[entry_idx].key;
        TVALTYPE value = hash->entries[entry_idx].value;
        hash->entries[entry_idx].valid = 0;
        hash->size--;

        if (TFN(put)(hash, &key, &value, NULL, NULL)) {
            assert(0);
        }

        entry_idx = (entry_idx + 1) & (hash->nentries - 1);
    }

    hash->size--;
}

/**
 * @file WATT.c
 * @brief Write-Aware Timestamp Tracking (WATT) eviction algorithm
 *
 * WATT is a cache eviction algorithm that considers both read and write
 * timestamps to make intelligent replacement decisions for modern hardware
 * with mixed read/write workloads.
 *
 * Key ideas:
 * - Each object tracks two timestamps: read_timestamp (last access) and
 *   write_timestamp (insertion time)
 * - Eviction decision is based on a score combining both timestamps
 * - Objects with low scores (old reads and old writes) are evicted first
 * - Recently written objects that haven't been read yet are protected
 *
 * Reference: "Write-Aware Timestamp Tracking: Effective and Efficient Page
 * Replacement for Modern Hardware"
 */

#include "dataStructure/hashtable/hashtable.h"
#include "libCacheSim/cache.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  cache_obj_t *q_head;
  cache_obj_t *q_tail;
  double alpha;  // weight for read timestamp vs write timestamp
} WATT_params_t;

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************
static void WATT_free(cache_t *cache);
static bool WATT_get(cache_t *cache, const request_t *req);
static cache_obj_t *WATT_find(cache_t *cache, const request_t *req,
                              bool update_cache);
static cache_obj_t *WATT_insert(cache_t *cache, const request_t *req);
static cache_obj_t *WATT_to_evict(cache_t *cache, const request_t *req);
static void WATT_evict(cache_t *cache, const request_t *req);
static bool WATT_remove(cache_t *cache, obj_id_t obj_id);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ****                       init, free, get                         ****
// ***********************************************************************

/**
 * @brief initialize cache
 *
 * @param ccache_params some common cache parameters
 * @param cache_specific_params cache specific parameters, see parse_params
 * function or use -e "print" with the cachesim binary
 */
cache_t *WATT_init(const common_cache_params_t ccache_params,
                   const char *cache_specific_params) {
  cache_t *cache =
      cache_struct_init("WATT", ccache_params, cache_specific_params);
  cache->cache_init = WATT_init;
  cache->cache_free = WATT_free;
  cache->get = WATT_get;
  cache->find = WATT_find;
  cache->insert = WATT_insert;
  cache->evict = WATT_evict;
  cache->remove = WATT_remove;
  cache->to_evict = WATT_to_evict;

  if (ccache_params.consider_obj_metadata) {
    cache->obj_md_size = 2;  // two int64_t timestamps
  } else {
    cache->obj_md_size = 0;
  }

  cache->eviction_params = my_malloc(WATT_params_t);
  memset(cache->eviction_params, 0, sizeof(WATT_params_t));
  WATT_params_t *params = (WATT_params_t *)cache->eviction_params;
  params->q_head = NULL;
  params->q_tail = NULL;
  params->alpha = 0.5;  // default: equal weight to read and write

  // Parse optional parameters
  if (cache_specific_params != NULL) {
    char param_copy[256];
    strncpy(param_copy, cache_specific_params, sizeof(param_copy) - 1);
    param_copy[sizeof(param_copy) - 1] = '\0';

    char *token = strtok(param_copy, ",");
    while (token != NULL) {
      char key[64], value[64];
      if (sscanf(token, " %63[^=]=%63s ", key, value) == 2) {
        if (strcmp(key, "alpha") == 0) {
          params->alpha = atof(value);
          if (params->alpha < 0.0) params->alpha = 0.0;
          if (params->alpha > 1.0) params->alpha = 1.0;
        }
      }
      token = strtok(NULL, ",");
    }
  }

  return cache;
}

/**
 * free resources used by this cache
 *
 * @param cache
 */
static void WATT_free(cache_t *cache) {
  free(cache->eviction_params);
  cache_struct_free(cache);
}

/**
 * @brief this function is the user facing API
 * it performs the following logic
 *
 * ```
 * if obj in cache:
 *    update_metadata
 *    return true
 * else:
 *    if cache does not have enough space:
 *        evict until it has space to insert
 *    insert the object
 *    return false
 * ```
 *
 * @param cache
 * @param req
 * @return true if cache hit, false if cache miss
 */
static bool WATT_get(cache_t *cache, const request_t *req) {
  bool ck_hit = cache_get_base(cache, req);
  return ck_hit;
}

// ***********************************************************************
// ****                                                               ****
// ****       developer facing APIs (used by cache developer)         ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief find an object in the cache
 *
 * @param cache
 * @param req
 * @param update_cache whether to update the cache,
 *  if true, the object is promoted
 *  and if the object is expired, it is removed from the cache
 * @return the object or NULL if not found
 */
static cache_obj_t *WATT_find(cache_t *cache, const request_t *req,
                              bool update_cache) {
  cache_obj_t *cache_obj = cache_find_base(cache, req, update_cache);
  if (cache_obj != NULL && update_cache) {
    // Update read timestamp on access
    cache_obj->WATT.read_timestamp = cache->n_req;
  }

  return cache_obj;
}

/**
 * @brief insert an object into the cache,
 * update the hash table and cache metadata
 * this function assumes the cache has enough space
 * eviction should be
 * performed before calling this function
 *
 * @param cache
 * @param req
 * @return the inserted object
 */
static cache_obj_t *WATT_insert(cache_t *cache, const request_t *req) {
  cache_obj_t *obj = cache_insert_base(cache, req);
  WATT_params_t *params = cache->eviction_params;

  // Add to tail of queue
  append_obj_to_tail(&params->q_head, &params->q_tail, obj);

  // Initialize timestamps
  obj->WATT.read_timestamp = cache->n_req;
  obj->WATT.write_timestamp = cache->n_req;

  return obj;
}

/**
 * @brief compute the eviction score for an object
 *
 * Lower score means the object is a better candidate for eviction.
 * Score combines the age since last read and age since write.
 *
 * @param obj the cache object
 * @param current_time current logical time
 * @param alpha weight for read age (1-alpha for write age)
 * @return the eviction score
 */
static inline double WATT_compute_score(const cache_obj_t *obj,
                                        int64_t current_time, double alpha) {
  int64_t read_age = current_time - obj->WATT.read_timestamp;
  int64_t write_age = current_time - obj->WATT.write_timestamp;

  // Score: weighted combination of read age and write age
  // Higher score = more valuable, should not be evicted
  return alpha * (double)read_age + (1.0 - alpha) * (double)write_age;
}

/**
 * @brief find the object to be evicted
 * this function does not actually evict the object or update metadata
 * not all eviction algorithms support this function
 * because the eviction logic cannot be decoupled from finding eviction
 * candidate, so use assert(false) if you cannot support this function
 *
 * @param cache the cache
 * @return the object to be evicted
 */
static cache_obj_t *WATT_to_evict(cache_t *cache, const request_t *req) {
  WATT_params_t *params = cache->eviction_params;
  cache_obj_t *victim = NULL;
  double min_score = 1e18;

  cache_obj_t *obj = params->q_head;
  while (obj != NULL) {
    double score = WATT_compute_score(obj, cache->n_req, params->alpha);
    if (score < min_score) {
      min_score = score;
      victim = obj;
    }
    obj = obj->queue.next;
  }

  return victim;
}

/**
 * @brief evict an object from the cache
 * it needs to call cache_evict_base before returning
 * which updates some metadata such as n_obj, occupied size, and hash table
 *
 * @param cache
 * @param req not used
 * @param evicted_obj if not NULL, return the evicted object to caller
 */
static void WATT_evict(cache_t *cache, const request_t *req) {
  WATT_params_t *params = cache->eviction_params;

  // Find the object with minimum score
  cache_obj_t *victim = NULL;
  double min_score = 1e18;

  cache_obj_t *obj = params->q_head;
  while (obj != NULL) {
    double score = WATT_compute_score(obj, cache->n_req, params->alpha);
    if (score < min_score) {
      min_score = score;
      victim = obj;
    }
    obj = obj->queue.next;
  }

  DEBUG_ASSERT(victim != NULL);

  // Remove from queue
  remove_obj_from_list(&params->q_head, &params->q_tail, victim);

  // Evict from cache
  cache_evict_base(cache, victim, true);
}

static void WATT_remove_obj(cache_t *cache, cache_obj_t *obj_to_remove) {
  DEBUG_ASSERT(obj_to_remove != NULL);
  WATT_params_t *params = cache->eviction_params;
  remove_obj_from_list(&params->q_head, &params->q_tail, obj_to_remove);
  cache_remove_obj_base(cache, obj_to_remove, true);
}

/**
 * @brief remove an object from the cache
 * this is different from cache_evict because it is used to for user trigger
 * remove, and eviction is used by the cache to make space for new objects
 *
 * it needs to call cache_remove_obj_base before returning
 * which updates some metadata such as n_obj, occupied size, and hash table
 *
 * @param cache
 * @param obj_id
 * @return true if the object is removed, false if the object is not in the
 * cache
 */
static bool WATT_remove(cache_t *cache, obj_id_t obj_id) {
  cache_obj_t *obj = hashtable_find_obj_id(cache->hashtable, obj_id);
  if (obj == NULL) {
    return false;
  }

  WATT_remove_obj(cache, obj);

  return true;
}

static void WATT_verify(cache_t *cache) {
  WATT_params_t *params = cache->eviction_params;
  int64_t n_obj = 0, n_byte = 0;
  cache_obj_t *obj = params->q_head;

  while (obj != NULL) {
    assert(hashtable_find_obj_id(cache->hashtable, obj->obj_id) != NULL);
    n_obj++;
    n_byte += obj->obj_size;
    obj = obj->queue.next;
  }

  assert(n_obj == cache->get_n_obj(cache));
  assert(n_byte == cache->get_occupied_byte(cache));
}

#ifdef __cplusplus
}
#endif

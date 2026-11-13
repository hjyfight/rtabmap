// Display-only removal of historical points contradicted by repeated LiDAR rays.
// Uses OctoMap's exact voxel traversal, not angular bins or whole-map aging.
#include <octomap/OcTree.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace {
using Key = octomap::OcTreeKey;
using Hash = Key::KeyHash;
using Keys = std::unordered_set<Key, Hash>;
struct Evidence { unsigned misses = 0; double stamp = 0; };
struct Filter {
  octomap::OcTree geometry;
  Keys map;
  std::unordered_map<Key, Evidence, Hash> evidence;
  unsigned required;
  double max_range, margin, window, last = -std::numeric_limits<double>::infinity();
  Filter(double resolution, unsigned count, double range, double endpoint_margin, double seconds)
    : geometry(resolution), required(count), max_range(range), margin(endpoint_margin), window(seconds) {}
  bool key(const float *p, Key &k) const {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
      geometry.coordToKeyChecked(p[0], p[1], p[2], k);
  }
};
thread_local std::string error;
}

extern "C" {
const char *dc_error() { return error.c_str(); }
void *dc_create(double resolution, unsigned count, double range, double margin, double seconds) {
  try { return new Filter(resolution, count, range, margin, seconds); }
  catch (const std::exception &e) {error = e.what(); return nullptr;}
}
void dc_destroy(void *ptr) { delete static_cast<Filter *>(ptr); }
void dc_reset(void *ptr) {
  auto &f = *static_cast<Filter *>(ptr);
  f.evidence.clear(); f.last = -std::numeric_limits<double>::infinity();
}
int dc_map(void *ptr, const float *points, size_t count) {
  try {
    auto &f = *static_cast<Filter *>(ptr);
    Keys keys; keys.reserve(count);
    for (size_t i=0; i<count; ++i) {Key key; if (f.key(points+3*i, key)) keys.insert(key);}
    f.map.swap(keys);
    for (auto it=f.evidence.begin(); it!=f.evidence.end();) {
      if (!f.map.count(it->first)) it=f.evidence.erase(it); else ++it;
    }
    return 0;
  } catch (const std::exception &e) {error = e.what(); return -1;}
}
int dc_scan(void *ptr, const float *points, size_t count, const float *origin, double stamp, size_t *stats) {
  try {
    auto &f = *static_cast<Filter *>(ptr);
    std::fill(stats, stats+3, 0);
    if (!std::isfinite(stamp) || stamp <= f.last) return 0; // One vote per distinct acquisition.
    Key origin_key;
    if (!f.key(origin, origin_key)) return 0;
    const octomap::point3d start(origin[0], origin[1], origin[2]);
    // Keep the nearest return per endpoint voxel; protect endpoints and adjacent
    // voxels so surface edges and pose/depth noise do not carve static furniture.
    std::unordered_map<Key, octomap::point3d, Hash> endpoints;
    Keys protected_keys, misses;
    for (size_t i=0; i<count; ++i) {
      Key k;
      if (!f.key(points+3*i, k)) continue;
      octomap::point3d p(points[3*i], points[3*i+1], points[3*i+2]);
      double distance = (p-start).norm();
      if (distance < std::max(.3, f.margin)) continue;
      auto existing=endpoints.find(k);
      if (existing==endpoints.end() || (existing->second-start).norm() > distance) endpoints[k]=p;
      if (distance <= f.max_range + f.margin) {
        for (int x=-1; x<=1; ++x) for (int y=-1; y<=1; ++y) for (int z=-1; z<=1; ++z) {
          int a=int(k[0])+x, b=int(k[1])+y, c=int(k[2])+z;
          if (std::min({a,b,c})>=0 && std::max({a,b,c})<=65535) {
            Key neighbor(a,b,c);
            if (f.map.count(neighbor)) protected_keys.insert(neighbor);
          }
        }
      }
    }
    octomap::KeyRay ray;
    for (const auto &entry : endpoints) {
      auto delta=entry.second-start;
      const double distance=delta.norm();
      const double length=std::min(f.max_range, distance-f.margin);
      if (length<=0) continue;
      const auto end=start+delta*(length/distance);
      if (!f.geometry.computeRayKeys(start,end,ray)) continue;
      ++stats[0];
      for (const auto &key : ray) if (f.map.count(key) && !protected_keys.count(key)) misses.insert(key);
    }
    for (const auto &key : protected_keys) f.evidence.erase(key);
    for (const auto &key : misses) {
      auto &e=f.evidence[key];
      if (e.misses < f.required && stamp-e.stamp > f.window) e.misses=0;
      e.misses=std::min(f.required, e.misses+1); e.stamp=stamp;
    }
    f.last=stamp; stats[1]=misses.size();
    for (const auto &entry : f.evidence) if (entry.second.misses>=f.required) ++stats[2];
    return 1;
  } catch (const std::exception &e) {error = e.what(); return -1;}
}
int dc_mask(void *ptr, const float *points, size_t count, uint8_t *keep) {
  try {
    auto &f=*static_cast<Filter *>(ptr);
    for (size_t i=0; i<count; ++i) {
      Key key;
      keep[i]=0;
      if (f.key(points+3*i,key)) {
        auto e=f.evidence.find(key);
        keep[i]=e==f.evidence.end() || e->second.misses < f.required;
      }
    }
    return 0;
  } catch (const std::exception &e) {error = e.what(); return -1;}
}
}

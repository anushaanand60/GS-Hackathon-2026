// Start of HEAD
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <unordered_map>
#include <cstdio>
#include <json/json.h>

using namespace std;

int main() {
    string input_str((istreambuf_iterator<char>(cin)), istreambuf_iterator<char>());
    Json::Value input_data;
    Json::CharReaderBuilder rb;
    string errs;
    istringstream ss(input_str);
    Json::parseFromStream(rb, ss, &input_data, &errs);

    double mapW = input_data["map_size"][0].asDouble();
    double mapH = input_data["map_size"][1].asDouble();
    double warehouseX = mapW / 2.0, warehouseY = mapH / 2.0;
    Json::Value drones = input_data["drones"];
    Json::Value deliveries = input_data["deliveries"];
    Json::Value no_fly_zones = input_data.get("no_fly_zones", Json::Value(Json::arrayValue));
    Json::Value charging_stations = input_data.get("charging_stations", Json::Value(Json::arrayValue));
// End of HEAD

// Start of BODY
    Json::Value flight_manifest(Json::arrayValue);

    const double BATTERY_CAPACITY = 500.0;
    const double SPEED = 1.0;
    const double EPS = 1e-9;

    auto dist = [](double ax, double ay, double bx, double by) -> double {
        double dx = bx - ax, dy = by - ay;
        return sqrt(dx * dx + dy * dy);
    };

    auto round2 = [](double t) -> double {
        return llround(t * 100.0) / 100.0;
    };

    auto json_num = [&](double v) -> Json::Value {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f", round2(v));
        return Json::Value(atof(buf));
    };

    struct NFZ {
        string shape;
        double cx, cy, radius;
        double x_min, y_min, x_max, y_max;
        double T_start, T_end;
    };

    struct Delivery {
        string id;
        double x, y, weight, deadline;
    };

    struct DroneInfo {
        string id;
        double max_payload;
    };

    struct ChargingStation {
        double x, y;
        int slots;
    };

    struct Step {
        double x, y, t;
        string action;
        vector<string> delivery_ids;
        string delivery_id;
    };

    struct Waypoint {
        double x, y, t;
        string action;
    };

    struct PathResult {
        vector<Waypoint> waypoints;
        double total_time = 0.0;
        double total_distance = 0.0;
        bool valid = false;
    };

    struct PairHashLL {
        size_t operator()(pair<long long,long long> const& p) const noexcept {
            return (size_t)p.first ^ ((size_t)p.second * 2654435761ULL + 0x9e3779b9ULL +
                (((size_t)p.first) << 6) + (((size_t)p.first) >> 2));
        }
    };

    struct DroneState {
        string id;
        double max_payload;
        double current_time = 0.0;
        double current_battery = 500.0;
        double x = 0.0, y = 0.0;
    };

    vector<DroneInfo> drone_list;
    for (const auto& d : drones) {
        DroneInfo di;
        di.id = d["id"].asString();
        di.max_payload = d["max_payload"].asDouble();
        drone_list.push_back(di);
    }

    vector<Delivery> delivery_list;
    for (int i = 0; i < (int)deliveries.size(); ++i) {
        const auto& d = deliveries[i];
        Delivery del;
        del.id = d["id"].asString();
        del.x = d["x"].asDouble();
        del.y = d["y"].asDouble();
        del.weight = d["weight"].asDouble();
        del.deadline = d["deadline"].asDouble();
        delivery_list.push_back(del);
    }
    const int M = (int)delivery_list.size();
    const int N = (int)drone_list.size();
    const bool large_job = M >= 6;

    vector<double> dist_to_warehouse(M);
    for (int i = 0; i < M; ++i) {
        dist_to_warehouse[i] = sqrt(
            (delivery_list[i].x - warehouseX) * (delivery_list[i].x - warehouseX) +
            (delivery_list[i].y - warehouseY) * (delivery_list[i].y - warehouseY));
    }

    vector<NFZ> nfz_list;
    for (const auto& z : no_fly_zones) {
        NFZ nfz;
        nfz.shape = z["shape"].asString();
        nfz.T_start = z.get("T_start", 0.0).asDouble();
        nfz.T_end = z.get("T_end", 1e18).asDouble();
        if (nfz.shape == "circle") {
            nfz.cx = z["center"][0].asDouble();
            nfz.cy = z["center"][1].asDouble();
            nfz.radius = z["radius"].asDouble();
        } else {
            if (z.isMember("corners")) {
                nfz.x_min = z["corners"][0][0].asDouble();
                nfz.y_min = z["corners"][0][1].asDouble();
                nfz.x_max = z["corners"][1][0].asDouble();
                nfz.y_max = z["corners"][1][1].asDouble();
            } else {
                nfz.x_min = z["x_min"].asDouble();
                nfz.y_min = z["y_min"].asDouble();
                nfz.x_max = z["x_max"].asDouble();
                nfz.y_max = z["y_max"].asDouble();
            }
            if (nfz.x_min > nfz.x_max) swap(nfz.x_min, nfz.x_max);
            if (nfz.y_min > nfz.y_max) swap(nfz.y_min, nfz.y_max);
        }
        nfz_list.push_back(nfz);
    }
    for (auto& nfz : nfz_list) {
        if (nfz.shape == "circle") {
            nfz.x_min = nfz.cx - nfz.radius;
            nfz.x_max = nfz.cx + nfz.radius;
            nfz.y_min = nfz.cy - nfz.radius;
            nfz.y_max = nfz.cy + nfz.radius;
        }
    }

    vector<ChargingStation> cs_list;
    for (const auto& c : charging_stations) {
        ChargingStation cs;
        cs.x = c["x"].asDouble();
        cs.y = c["y"].asDouble();
        cs.slots = c.get("slots", 1).asInt();
        cs_list.push_back(cs);
    }

    const bool no_nfz = nfz_list.empty();
    bool no_time_windows_flag = true;
    for (const auto& nfz : nfz_list) {
        if (nfz.T_start > EPS || nfz.T_end < 1e17) { no_time_windows_flag = false; break; }
    }
    const bool no_time_windows = no_time_windows_flag;

    auto geo_key = [](double ax, double ay, double bx, double by) -> long long {
        long long a = llround(ax * 100.0) & 0xFFFFLL;
        long long b = llround(ay * 100.0) & 0xFFFFLL;
        long long c = llround(bx * 100.0) & 0xFFFFLL;
        long long d = llround(by * 100.0) & 0xFFFFLL;
        return (a << 48) | (b << 32) | (c << 16) | d;
    };

    const double GRID_CELL = 25.0;
    int grid_cols = max(1, (int)ceil(mapW / GRID_CELL));
    int grid_rows = max(1, (int)ceil(mapH / GRID_CELL));
    vector<vector<int>> nfz_grid;
    vector<int> nfz_stamp;
    int nfz_stamp_gen = 0;
    unordered_map<pair<long long, long long>, vector<int>, PairHashLL> seg_block_cache;
    seg_block_cache.reserve(16384);
    static const vector<int> EMPTY_BLOCKERS;

    if (!no_nfz) {
        nfz_grid.assign(grid_cols * grid_rows, {});
        nfz_stamp.assign(nfz_list.size(), 0);
        for (int i = 0; i < (int)nfz_list.size(); ++i) {
            const NFZ& z = nfz_list[i];
            int c0 = max(0, (int)(z.x_min / GRID_CELL));
            int c1 = min(grid_cols - 1, (int)(z.x_max / GRID_CELL));
            int r0 = max(0, (int)(z.y_min / GRID_CELL));
            int r1 = min(grid_rows - 1, (int)(z.y_max / GRID_CELL));
            for (int r = r0; r <= r1; ++r)
                for (int c = c0; c <= c1; ++c)
                    nfz_grid[r * grid_cols + c].push_back(i);
        }
    }

    auto segment_hits_nfz = [&](double ax, double ay, double bx, double by,
                                double t_depart, const NFZ& nfz) -> bool {
        double sx0 = min(ax, bx), sx1 = max(ax, bx);
        double sy0 = min(ay, by), sy1 = max(ay, by);
        if (sx1 < nfz.x_min - EPS || sx0 > nfz.x_max + EPS ||
            sy1 < nfz.y_min - EPS || sy0 > nfz.y_max + EPS) {
            return false;
        }
        double dx = bx - ax, dy = by - ay;
        double seg_len = dist(ax, ay, bx, by);

        if (nfz.shape == "circle") {
            double cx = nfz.cx, cy = nfz.cy, r = nfz.radius;
            if (seg_len < EPS) {
                if (dist(ax, ay, cx, cy) < r - EPS) {
                    return t_depart >= nfz.T_start - EPS && t_depart <= nfz.T_end + EPS;
                }
                return false;
            }

            double acx = ax - cx, acy = ay - cy;
            double a_coef = dx * dx + dy * dy;
            double b_coef = 2.0 * (dx * acx + dy * acy);
            double c_coef = acx * acx + acy * acy - r * r;
            double disc = b_coef * b_coef - 4.0 * a_coef * c_coef;

            double s_enter, s_exit;
            if (disc < 0) {
                double s_star = (dx * (cx - ax) + dy * (cy - ay)) / a_coef;
                s_star = max(0.0, min(1.0, s_star));
                double px = ax + s_star * dx, py = ay + s_star * dy;
                if (dist(px, py, cx, cy) >= r - EPS) return false;
                s_enter = 0.0;
                s_exit = 1.0;
            } else {
                double sd = sqrt(max(0.0, disc));
                s_enter = (-b_coef - sd) / (2.0 * a_coef);
                s_exit = (-b_coef + sd) / (2.0 * a_coef);
                if (s_enter > s_exit) swap(s_enter, s_exit);
                s_enter = max(0.0, s_enter);
                s_exit = min(1.0, s_exit);
                if (s_enter > s_exit + EPS) return false;
            }

            double t_enter = t_depart + s_enter * seg_len;
            double t_exit = t_depart + s_exit * seg_len;
            return !(t_exit < nfz.T_start - EPS || t_enter > nfz.T_end + EPS);
        }

        double s_lo = 0.0, s_hi = 1.0;
        auto clip = [&](double v0, double v1, double lo, double hi) {
            if (fabs(v1 - v0) < EPS) {
                if (v0 < lo - EPS || v0 > hi + EPS) { s_lo = 1.0; s_hi = 0.0; }
                return;
            }
            double t0 = (lo - v0) / (v1 - v0);
            double t1 = (hi - v0) / (v1 - v0);
            if (t0 > t1) swap(t0, t1);
            s_lo = max(s_lo, t0);
            s_hi = min(s_hi, t1);
        };
        clip(ax, bx, nfz.x_min, nfz.x_max);
        clip(ay, by, nfz.y_min, nfz.y_max);
        if (s_lo > s_hi + EPS) return false;

        double t_enter = t_depart + s_lo * seg_len;
        double t_exit = t_depart + s_hi * seg_len;
        return !(t_exit < nfz.T_start - EPS || t_enter > nfz.T_end + EPS);
    };

    auto seg_blockers = [&](double ax, double ay, double bx, double by,
                              double t_depart) -> const vector<int>& {
        if (no_nfz) return EMPTY_BLOCKERS;
        pair<long long, long long> key = {geo_key(ax, ay, bx, by),
                                          no_time_windows ? 0LL : llround(t_depart)};
        auto it = seg_block_cache.find(key);
        if (it != seg_block_cache.end()) return it->second;
        vector<int>& ids = seg_block_cache[key];
        double sx0 = min(ax, bx), sx1 = max(ax, bx), sy0 = min(ay, by), sy1 = max(ay, by);
        int c0 = max(0, (int)(sx0 / GRID_CELL)), c1 = min(grid_cols - 1, (int)(sx1 / GRID_CELL));
        int r0 = max(0, (int)(sy0 / GRID_CELL)), r1 = min(grid_rows - 1, (int)(sy1 / GRID_CELL));
        int gen = ++nfz_stamp_gen;
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                for (int id : nfz_grid[r * grid_cols + c]) {
                    if (nfz_stamp[id] == gen) continue;
                    nfz_stamp[id] = gen;
                    if (segment_hits_nfz(ax, ay, bx, by, t_depart, nfz_list[id])) {
                        ids.push_back(id);
                    }
                }
            }
        }
        return ids;
    };

    auto segment_hits_any = [&](double ax, double ay, double bx, double by,
                                 double t_depart) -> bool {
        return !seg_blockers(ax, ay, bx, by, t_depart).empty();
    };

    auto circle_detour_points = [&](double ax, double ay, const NFZ& nfz)
            -> vector<pair<double, double>> {
        vector<pair<double, double>> pts;
        double margin = nfz.radius * 1.08;
        double vx = ax - nfz.cx, vy = ay - nfz.cy;
        double d_ac = sqrt(vx * vx + vy * vy);
        if (d_ac <= nfz.radius + EPS) {
            double len = d_ac > EPS ? d_ac : 1.0;
            double px = -vy / len, py = vx / len;
            pts.push_back({nfz.cx + px * margin, nfz.cy + py * margin});
            pts.push_back({nfz.cx - px * margin, nfz.cy - py * margin});
            return pts;
        }
        double theta = atan2(vy, vx);
        double alpha = acos(min(1.0, nfz.radius / d_ac));
        pts.push_back({nfz.cx + margin * cos(theta + alpha),
                       nfz.cy + margin * sin(theta + alpha)});
        pts.push_back({nfz.cx + margin * cos(theta - alpha),
                       nfz.cy + margin * sin(theta - alpha)});
        return pts;
    };

    auto wait_clear_time = [&](double ax, double ay, double bx, double by,
                                double t0) -> double {
        double t_after = t0;
        for (int iter = 0; iter < 4; ++iter) {
            auto b = seg_blockers(ax, ay, bx, by, t_after);
            if (b.empty()) return t_after;
            double extra = 0.0;
            for (int bi : b) {
                extra = max(extra, max(0.0, nfz_list[bi].T_end - t_after));
            }
            if (extra < EPS) return t_after;
            t_after += extra;
        }
        return t_after;
    };

    auto plan_segment = [&](double ax, double ay, double bx, double by,
                            double t_depart) -> PathResult {
        PathResult bad;
        bad.valid = false;
        double d = dist(ax, ay, bx, by);
        if (!segment_hits_any(ax, ay, bx, by, t_depart)) {
            PathResult direct;
            direct.valid = true;
            direct.total_distance = d;
            direct.total_time = d / SPEED;
            if (d > EPS) direct.waypoints.push_back({bx, by, t_depart + d / SPEED, "WAYPOINT"});
            return direct;
        }
        vector<int> local_blockers = seg_blockers(ax, ay, bx, by, t_depart);
        for (int bi : local_blockers) {
            const NFZ& nfz = nfz_list[bi];
            vector<pair<double, double>> pts;
            if (nfz.shape == "circle") pts = circle_detour_points(ax, ay, nfz);
            else {
                double m = 1.0;
                pts = {{nfz.x_min - m, nfz.y_min - m}, {nfz.x_max + m, nfz.y_max + m}};
            }
            for (const auto& pt : pts) {
                double d1 = dist(ax, ay, pt.first, pt.second);
                if (d1 < EPS || segment_hits_any(ax, ay, pt.first, pt.second, t_depart)) continue;
                double t1 = t_depart + d1 / SPEED, d2 = dist(pt.first, pt.second, bx, by);
                if (d2 < EPS || segment_hits_any(pt.first, pt.second, bx, by, t1)) continue;
                PathResult pr;
                pr.valid = true;
                pr.total_distance = d1 + d2;
                pr.total_time = d1 / SPEED + d2 / SPEED;
                pr.waypoints.push_back({pt.first, pt.second, t1, "WAYPOINT"});
                pr.waypoints.push_back({bx, by, t1 + d2 / SPEED, "WAYPOINT"});
                return pr;
            }
        }
        if (!no_time_windows) {
            double ta = wait_clear_time(ax, ay, bx, by, t_depart);
            if (!segment_hits_any(ax, ay, bx, by, ta)) {
                PathResult pr;
                pr.valid = true;
                pr.total_distance = d;
                pr.total_time = (ta - t_depart) + d / SPEED;
                if (ta > t_depart + EPS) pr.waypoints.push_back({ax, ay, ta, "WAIT"});
                if (d > EPS) pr.waypoints.push_back({bx, by, ta + d / SPEED, "WAYPOINT"});
                return pr;
            }
        }
        return bad;
    };

    struct SpPath {
        int n = -1;
        double mx = 0.0, my = 0.0;
    };
    unordered_map<long long, SpPath> sp_cache;
    sp_cache.reserve(16384);
    unordered_map<pair<long long, long long>, pair<double, double>, PairHashLL> leg_pair_cache;
    leg_pair_cache.reserve(16384);

    auto try_cached_path = [&](double ax, double ay, double bx, double by,
                                double t_depart) -> PathResult {
        PathResult bad;
        bad.valid = false;
        auto it = sp_cache.find(geo_key(ax, ay, bx, by));
        if (it == sp_cache.end() || it->second.n < 0) return bad;
        const SpPath& sp = it->second;

        if (sp.n == 0) {
            double ta = t_depart;
            if (segment_hits_any(ax, ay, bx, by, t_depart)) {
                if (no_time_windows) return bad;
                ta = wait_clear_time(ax, ay, bx, by, t_depart);
                if (segment_hits_any(ax, ay, bx, by, ta)) return bad;
            }
            double d = dist(ax, ay, bx, by);
            PathResult pr;
            pr.valid = true;
            pr.total_distance = d;
            pr.total_time = (ta - t_depart) + d / SPEED;
            if (ta > t_depart + EPS) pr.waypoints.push_back({ax, ay, ta, "WAIT"});
            if (d > EPS) pr.waypoints.push_back({bx, by, ta + d / SPEED, "WAYPOINT"});
            return pr;
        }

        double mx = sp.mx, my = sp.my;
        double d1 = dist(ax, ay, mx, my);
        if (d1 < EPS || segment_hits_any(ax, ay, mx, my, t_depart)) return bad;
        double t1 = t_depart + d1 / SPEED;
        double d2 = dist(mx, my, bx, by);
        if (d2 < EPS || segment_hits_any(mx, my, bx, by, t1)) return bad;
        PathResult pr;
        pr.valid = true;
        pr.total_distance = d1 + d2;
        pr.total_time = d1 / SPEED + d2 / SPEED;
        pr.waypoints.push_back({mx, my, t1, "WAYPOINT"});
        pr.waypoints.push_back({bx, by, t1 + d2 / SPEED, "WAYPOINT"});
        return pr;
    };

    auto store_spatial = [&](const PathResult& pr, double ax, double ay,
                              double bx, double by) {
        if (!pr.valid) return;
        SpPath sp;
        vector<Waypoint> mv;
        for (const auto& w : pr.waypoints) {
            if (w.action != "WAIT") mv.push_back(w);
        }
        if (mv.size() <= 1) {
            sp.n = 0;
        } else {
            sp.n = 1;
            sp.mx = mv[0].x;
            sp.my = mv[0].y;
        }
        sp_cache[geo_key(ax, ay, bx, by)] = sp;
    };

    auto query_segment = [&](double ax, double ay, double bx, double by,
                              double t_depart) -> PathResult {
        if (no_nfz) {
            PathResult direct;
            double d = dist(ax, ay, bx, by);
            direct.valid = true;
            direct.total_distance = d;
            direct.total_time = d / SPEED;
            if (d > EPS) {
                direct.waypoints.push_back({bx, by, t_depart + d / SPEED, "WAYPOINT"});
            }
            return direct;
        }
        PathResult cached = try_cached_path(ax, ay, bx, by, t_depart);
        if (cached.valid) return cached;
        PathResult result = plan_segment(ax, ay, bx, by, t_depart);
        if (result.valid) store_spatial(result, ax, ay, bx, by);
        return result;
    };

    auto energy_for_leg = [](double dist_val, double payload) -> double {
        return dist_val * (1.0 + payload);
    };

    auto leg_pair_fast = [&](double ax, double ay, double bx, double by,
                              double t) -> pair<double, double> {
        double d = dist(ax, ay, bx, by);
        if (no_nfz) return {d, d / SPEED};
        pair<long long, long long> lk = {geo_key(ax, ay, bx, by),
                                         no_time_windows ? 0LL : llround(t)};
        auto lit = leg_pair_cache.find(lk);
        if (lit != leg_pair_cache.end()) return lit->second;
        pair<double, double> result;
        if (!segment_hits_any(ax, ay, bx, by, t)) {
            result = {d, d / SPEED};
        } else if (!no_time_windows) {
            double t_after = wait_clear_time(ax, ay, bx, by, t);
            if (!segment_hits_any(ax, ay, bx, by, t_after)) {
                result = {d, (t_after - t) + d / SPEED};
            } else {
                PathResult seg = query_segment(ax, ay, bx, by, t);
                result = seg.valid ? make_pair(seg.total_distance, seg.total_time)
                                   : make_pair(d, d / SPEED);
            }
        } else {
            PathResult seg = query_segment(ax, ay, bx, by, t);
            result = seg.valid ? make_pair(seg.total_distance, seg.total_time)
                               : make_pair(d, d / SPEED);
        }
        leg_pair_cache[lk] = result;
        return result;
    };

    auto leg_distance_fast = [&](double ax, double ay, double bx, double by, double t) -> double {
        return leg_pair_fast(ax, ay, bx, by, t).first;
    };
    auto leg_time_fast = [&](double ax, double ay, double bx, double by, double t) -> double {
        return leg_pair_fast(ax, ay, bx, by, t).second;
    };

    vector<vector<pair<double, double>>> cs_bookings(cs_list.size());

    auto cs_overlaps = [&](double a0, double a1, double b0, double b1) -> bool {
        return !(a1 <= b0 + EPS || b1 <= a0 + EPS);
    };

    auto cs_slots_free = [&](const vector<vector<pair<double, double>>>& bookings,
                              int cs_idx, double t0, double t1) -> bool {
        if (cs_idx < 0) return false;
        int used = 0;
        for (const auto& w : bookings[cs_idx]) {
            if (cs_overlaps(w.first, w.second, t0, t1)) used++;
        }
        return used < cs_list[cs_idx].slots;
    };

    auto reserve_cs_slot = [&](vector<vector<pair<double, double>>>& bookings,
                                int cs_idx, double t0, double t1) {
        if (cs_idx >= 0) bookings[cs_idx].push_back({t0, t1});
    };

    auto earliest_charge_start = [&](const vector<vector<pair<double, double>>>& bookings,
                                      int cs_idx, double arrival_t,
                                      double charge_duration) -> double {
        if (cs_idx < 0) return 1e18;
        double t = arrival_t;
        for (int iter = 0; iter < 10; ++iter) {
            if (cs_slots_free(bookings, cs_idx, t, t + charge_duration)) return t;
            double next = 1e18;
            for (const auto& w : bookings[cs_idx]) {
                if (cs_overlaps(w.first, w.second, t, t + charge_duration))
                    next = min(next, w.second);
            }
            if (next >= 1e17 - 1) return 1e18;
            t = next;
        }
        return 1e18;
    };

    auto nearest_cs_idx = [&](double x, double y) -> int {
        int best = -1;
        double best_d = 1e18;
        for (int i = 0; i < (int)cs_list.size(); ++i) {
            double d = dist(x, y, cs_list[i].x, cs_list[i].y);
            if (d < best_d - EPS) {
                best_d = d;
                best = i;
            }
        }
        return best;
    };

    auto energy_from_pos = [&](double px, double py, double pt, double payload,
                                const vector<int>& batch, int start_k) -> double {
        double e = 0.0;
        double x = px, y = py, t = pt, w = payload;
        for (int k = start_k; k < (int)batch.size(); ++k) {
            const Delivery& del = delivery_list[batch[k]];
            auto lp = leg_pair_fast(x, y, del.x, del.y, t);
            e += energy_for_leg(lp.first, w);
            t += lp.second;
            w -= del.weight;
            x = del.x; y = del.y;
        }
        e += energy_for_leg(leg_pair_fast(x, y, warehouseX, warehouseY, t).first, 0.0);
        return e;
    };

    auto simulate_trip_core = [&](double start_x, double start_y, double start_time,
                                   double start_battery, const vector<int>& order,
                                   vector<vector<pair<double, double>>>& bookings,
                                   bool commit_cs) -> pair<bool, double> {
        double bat = start_battery;
        double cx = start_x, cy = start_y, ct = start_time;
        double payload = 0.0;
        for (int idx : order) payload += delivery_list[idx].weight;

        for (int k = 0; k < (int)order.size(); ++k) {
            int idx = order[k];
            const Delivery& del = delivery_list[idx];

            auto lp = leg_pair_fast(cx, cy, del.x, del.y, ct);
            double leg_d = lp.first;
            double leg_t = lp.second;
            double leg_e = energy_for_leg(leg_d, payload);

            double max_rem_d = 0.0;
            for (int k2 = k; k2 < (int)order.size(); ++k2) {
                max_rem_d += dist_to_warehouse[order[k2]] * 2.0;
            }
            if (bat + EPS >= leg_e + energy_for_leg(max_rem_d, payload)) {
                bat -= leg_e;
                ct += leg_t;
                if (ct > del.deadline + EPS) return {false, ct};
                payload -= del.weight;
                cx = del.x;
                cy = del.y;
                continue;
            }

            double arrive_t = ct + leg_t;
            double rest_e = energy_from_pos(del.x, del.y, arrive_t,
                                            payload - del.weight, order, k + 1);

            int cs_attempts = 0;
            while (bat + EPS < leg_e + rest_e && !cs_list.empty()) {
                if (++cs_attempts > 3) return {false, ct};
                double charge_time_est = max(1.0, ceil(max(0.0, leg_e + rest_e - bat) / 2.0));
                int cs_idx = nearest_cs_idx(cx, cy);
                if (cs_idx < 0) return {false, ct};
                auto lp_cs = leg_pair_fast(cx, cy, cs_list[cs_idx].x, cs_list[cs_idx].y, ct);
                double charge_start = earliest_charge_start(bookings, cs_idx,
                    ct + lp_cs.second, charge_time_est);
                if (charge_start >= 1e17) return {false, ct};
                double to_cs_e = energy_for_leg(lp_cs.first, payload);
                if (bat + EPS < to_cs_e) return {false, ct};
                bat -= to_cs_e;
                ct += lp_cs.second;
                cx = cs_list[cs_idx].x;
                cy = cs_list[cs_idx].y;

                auto lp2 = leg_pair_fast(cx, cy, del.x, del.y, ct);
                leg_d = lp2.first;
                leg_t = lp2.second;
                leg_e = energy_for_leg(leg_d, payload);
                arrive_t = ct + lp2.second;
                rest_e = energy_from_pos(del.x, del.y, arrive_t,
                                         payload - del.weight, order, k + 1);

                double need = leg_e + rest_e;
                double charge_time = max(1.0, ceil(max(0.0, need - bat) / 2.0));
                charge_start = earliest_charge_start(bookings, cs_idx, ct, charge_time);
                if (charge_start >= 1e17) return {false, ct};
                double charge_end = charge_start + charge_time;
                double bat_after = min(BATTERY_CAPACITY, bat + 2.0 * charge_time);
                if (bat_after + EPS < need) return {false, ct};
                if (commit_cs) reserve_cs_slot(bookings, cs_idx, charge_start, charge_end);
                bat = bat_after;
                ct = charge_end;
            }

            if (bat + EPS < leg_e) return {false, ct};
            bat -= leg_e;
            ct += leg_t;
            if (ct > del.deadline + EPS) return {false, ct};
            payload -= del.weight;
            cx = del.x; cy = del.y;
        }

        auto lp_ret = leg_pair_fast(cx, cy, warehouseX, warehouseY, ct);
        double ret_e = energy_for_leg(lp_ret.first, 0.0);
        if (bat - ret_e < -EPS) return {false, ct};
        ct += lp_ret.second;
        return {true, ct};
    };

    vector<vector<pair<double, double>>> empty_bookings(cs_list.size());
    auto can_complete_fast = [&](const DroneState& ds,
                                  const vector<int>& batch) -> bool {
        if (batch.empty()) return true;
        double payload = 0.0;
        for (int idx : batch) payload += delivery_list[idx].weight;
        if (payload > ds.max_payload + EPS) return false;
        return simulate_trip_core(ds.x, ds.y, ds.current_time, ds.current_battery,
                                  batch, M > 500 ? cs_bookings : empty_bookings, false).first;
    };

    vector<DroneState> drone_states;
    for (const auto& d : drone_list) {
        DroneState ds;
        ds.id = d.id;
        ds.max_payload = d.max_payload;
        ds.x = warehouseX;
        ds.y = warehouseY;
        ds.current_battery = BATTERY_CAPACITY;
        drone_states.push_back(ds);
    }

    vector<char> assigned(M, 0);

    auto order_batch = [&](vector<int>& batch, double start_x, double start_y, double start_t) {
        vector<int> ordered, rem = batch;
        double cx = start_x, cy = start_y, ct = start_t;
        while (!rem.empty()) {
            int best = -1;
            double best_travel = 1e18;
            for (int idx : rem) {
                const Delivery& del = delivery_list[idx];
                double travel = no_nfz
                    ? dist(cx, cy, del.x, del.y) / SPEED
                    : leg_time_fast(cx, cy, del.x, del.y, ct);
                if (ct + travel > del.deadline + EPS) continue;
                if (best == -1 || travel < best_travel - EPS ||
                    (fabs(travel - best_travel) < EPS &&
                     delivery_list[idx].weight > delivery_list[best].weight + EPS)) {
                    best = idx;
                    best_travel = travel;
                }
            }
            if (best == -1) {
                double best_slack = 1e18;
                for (int idx : rem) {
                    const Delivery& del = delivery_list[idx];
                    double travel = no_nfz
                        ? dist(cx, cy, del.x, del.y) / SPEED
                        : leg_time_fast(cx, cy, del.x, del.y, ct);
                    double slack = del.deadline - (ct + travel);
                    if (best == -1 || slack < best_slack - EPS) {
                        best = idx;
                        best_slack = slack;
                    }
                }
            }
            ordered.push_back(best);
            ct += no_nfz
                ? dist(cx, cy, delivery_list[best].x, delivery_list[best].y) / SPEED
                : leg_time_fast(cx, cy, delivery_list[best].x, delivery_list[best].y, ct);
            cx = delivery_list[best].x;
            cy = delivery_list[best].y;
            rem.erase(remove(rem.begin(), rem.end(), best), rem.end());
        }
        batch = ordered;
    };

    auto try_build_batch = [&](const DroneState& ds, const vector<int>& pool,
                                int must_include = -1) -> vector<int> {
        vector<int> candidates;
        for (int idx : pool) {
            const Delivery& del = delivery_list[idx];
            double travel = leg_time_fast(ds.x, ds.y, del.x, del.y, ds.current_time);
            if (ds.current_time + travel <= del.deadline + EPS)
                candidates.push_back(idx);
        }
        if (candidates.empty()) return {};

        sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            return delivery_list[a].deadline < delivery_list[b].deadline;
        });

        int max_batch = M <= 5 ? 5 : M <= 20 ? 4 : M <= 50 ? 3 : 2;
        vector<int> batch;
        double wsum = 0.0;
        for (int idx : candidates) {
            if ((int)batch.size() >= max_batch) break;
            if (wsum + delivery_list[idx].weight > ds.max_payload + EPS) continue;
            vector<int> trial = batch;
            trial.push_back(idx);
            order_batch(trial, ds.x, ds.y, ds.current_time);
            if (can_complete_fast(ds, trial)) {
                batch = trial;
                wsum += delivery_list[idx].weight;
            }
        }

        if ((int)batch.size() < 2 && (int)candidates.size() >= 2) {
            int top = min(4, (int)candidates.size());
            for (int i = 0; i < top; ++i) {
                for (int j = i + 1; j < top; ++j) {
                    double tw = delivery_list[candidates[i]].weight +
                                delivery_list[candidates[j]].weight;
                    if (tw > ds.max_payload + EPS) continue;
                    vector<int> trial = {candidates[i], candidates[j]};
                    order_batch(trial, ds.x, ds.y, ds.current_time);
                    if (can_complete_fast(ds, trial)) { batch = trial; wsum = tw; }
                }
            }
        }

        if (must_include >= 0) {
            bool has = false;
            for (int x : batch) if (x == must_include) has = true;
            if (!has) {
                if (can_complete_fast(ds, {must_include})) batch = {must_include};
                else return {};
            }
        }

        if (batch.empty()) {
            for (int idx : candidates)
                if (can_complete_fast(ds, {idx})) return {idx};
            return {};
        }
        order_batch(batch, ds.x, ds.y, ds.current_time);
        return batch;
    };

    auto append_segment_steps = [&](vector<Step>& steps,
                                     double& cx, double& cy, double& ct,
                                     double tx, double ty,
                                     bool omit_dest = false,
                                     const PathResult* cached = nullptr) -> double {
        PathResult seg = cached ? *cached : query_segment(cx, cy, tx, ty, ct);
        if (!seg.valid) return -1.0;

        double total_d = 0.0;
        double prev_x = round2(cx), prev_y = round2(cy);
        double last_t = round2(ct);
        double dest_x = round2(tx), dest_y = round2(ty);

        for (size_t i = 0; i < seg.waypoints.size(); ++i) {
            const auto& wp = seg.waypoints[i];
            bool is_last = (i + 1 == seg.waypoints.size());
            double wx = round2(wp.x), wy = round2(wp.y);

            if (wp.action == "WAIT") {
                double wt = round2(max(wp.t, last_t));
                if (fabs(wx - prev_x) < EPS && fabs(wy - prev_y) < EPS &&
                    wt > last_t + EPS) {
                    steps.push_back({wx, wy, wt, "WAIT", {}, ""});
                    last_t = wt;
                }
                ct = wt;
                prev_x = wx;
                prev_y = wy;
                continue;
            }

            double leg_d = dist(prev_x, prev_y, wx, wy);
            if (leg_d > EPS) {
                double need_depart = wp.t - leg_d / SPEED;
                if (need_depart > last_t + EPS) {
                    steps.push_back({prev_x, prev_y, round2(need_depart), "WAIT", {}, ""});
                    last_t = round2(need_depart);
                }
                total_d += leg_d;
                last_t = round2(last_t + leg_d / SPEED);
            } else if (wp.t > last_t + EPS) {
                steps.push_back({wx, wy, round2(wp.t), "WAIT", {}, ""});
                last_t = round2(wp.t);
            }

            if (omit_dest && is_last &&
                fabs(wx - dest_x) < EPS && fabs(wy - dest_y) < EPS) {
                prev_x = wx;
                prev_y = wy;
                ct = last_t;
                continue;
            }

            steps.push_back({wx, wy, last_t, wp.action, {}, ""});
            prev_x = wx;
            prev_y = wy;
            ct = last_t;
        }
        cx = dest_x;
        cy = dest_y;
        return total_d;
    };

    auto departure_time = [&](double sx, double sy, const PathResult& seg) -> double {
        if (!seg.valid || seg.waypoints.empty()) return 0.0;
        for (const auto& wp : seg.waypoints) {
            if (wp.action == "WAIT") continue;
            double d = dist(sx, sy, wp.x, wp.y);
            if (d > EPS) return wp.t - d / SPEED;
            sx = wp.x; sy = wp.y;
        }
        return 0.0;
    };

    auto build_trip = [&](const vector<int>& batch, double trip_start_t, double max_pl,
                           vector<vector<pair<double, double>>>& bookings,
                           double* trip_end_out = nullptr) -> vector<Step> {
        double payload = 0.0;
        for (int idx : batch) payload += delivery_list[idx].weight;
        if (payload > max_pl + EPS) return {};
        vector<Step> steps;
        double cx = warehouseX, cy = warehouseY;
        double ct = trip_start_t;
        double bat = BATTERY_CAPACITY;
        bool solo = batch.size() == 1;
        vector<string> pickup_ids;
        for (int idx : batch) pickup_ids.push_back(delivery_list[idx].id);

        double pickup_t = round2(trip_start_t);
        PathResult seg0;
        bool have_seg0 = false;
        if (!batch.empty()) {
            const Delivery& fd = delivery_list[batch[0]];
            seg0 = query_segment(warehouseX, warehouseY, fd.x, fd.y, trip_start_t);
            have_seg0 = seg0.valid;
            if (have_seg0) {
                double dt = departure_time(warehouseX, warehouseY, seg0);
                if (dt > EPS) pickup_t = round2(max(trip_start_t, dt));
            }
        }
        if (trip_start_t > EPS && pickup_t > round2(trip_start_t) + EPS) {
            steps.push_back({round2(warehouseX), round2(warehouseY), pickup_t, "WAIT", {}, ""});
        }
        steps.push_back({round2(warehouseX), round2(warehouseY), pickup_t, "PICKUP", pickup_ids, ""});
        ct = pickup_t;
        cx = warehouseX;
        cy = warehouseY;

        for (int k = 0; k < (int)batch.size(); ++k) {
            int idx = batch[k];
            const Delivery& del = delivery_list[idx];

            PathResult seg_to_del;
            if (k == 0 && have_seg0 &&
                fabs(cx - warehouseX) < EPS && fabs(cy - warehouseY) < EPS &&
                fabs(ct - pickup_t) < EPS && fabs(pickup_t - trip_start_t) < EPS) {
                seg_to_del = seg0;
            } else {
                seg_to_del = query_segment(cx, cy, del.x, del.y, ct);
            }
            if (!seg_to_del.valid) return {};
            double leg_d = seg_to_del.total_distance;
            double leg_e = energy_for_leg(leg_d, payload);
            double arrive_t = ct + seg_to_del.total_time;
            double rest_e = solo
                ? energy_for_leg(leg_distance_fast(del.x, del.y, warehouseX, warehouseY, arrive_t), 0.0)
                : energy_from_pos(del.x, del.y, arrive_t, payload - del.weight, batch, k + 1);

            while (bat + EPS < leg_e + rest_e && !cs_list.empty()) {
                int cs_idx = nearest_cs_idx(cx, cy);
                if (cs_idx < 0) return {};

                double cs_x = cs_list[cs_idx].x, cs_y = cs_list[cs_idx].y;
                double to_cs_d = leg_distance_fast(cx, cy, cs_x, cs_y, ct);
                double to_cs_e = energy_for_leg(to_cs_d, payload);
                if (bat + EPS < to_cs_e) return {};

                PathResult seg_cs = query_segment(cx, cy, cs_x, cs_y, ct);
                if (!seg_cs.valid) return {};
                double to_cs = append_segment_steps(steps, cx, cy, ct, cs_x, cs_y, true, &seg_cs);
                if (to_cs < 0) return {};
                if (bat + EPS < energy_for_leg(to_cs, payload)) return {};
                bat -= energy_for_leg(to_cs, payload);
                cx = cs_x; cy = cs_y;

                seg_to_del = query_segment(cx, cy, del.x, del.y, ct);
                if (!seg_to_del.valid) return {};
                leg_d = seg_to_del.total_distance;
                leg_e = energy_for_leg(leg_d, payload);
                arrive_t = ct + seg_to_del.total_time;
                rest_e = solo
                    ? energy_for_leg(leg_distance_fast(del.x, del.y, warehouseX, warehouseY, arrive_t), 0.0)
                    : energy_from_pos(del.x, del.y, arrive_t, payload - del.weight, batch, k + 1);

                double need = leg_e + rest_e;
                double deficit = max(0.0, need - bat);
                double charge_time = max(1.0, ceil(deficit / 2.0));
                double charge_start = earliest_charge_start(bookings, cs_idx, ct, charge_time);
                if (charge_start >= 1e17) return {};
                double charge_end = round2(charge_start + charge_time);
                charge_start = round2(charge_start);
                double bat_after = min(BATTERY_CAPACITY, bat + 2.0 * charge_time);
                if (bat_after + EPS < need) return {};

                reserve_cs_slot(bookings, cs_idx, charge_start, charge_end);
                bat = bat_after;
                if (charge_start > ct + EPS) {
                    steps.push_back({round2(cs_x), round2(cs_y), charge_start, "WAIT", {}, ""});
                }
                steps.push_back({round2(cs_x), round2(cs_y), charge_start, "CHARGE", {}, ""});
                steps.push_back({round2(cs_x), round2(cs_y), charge_end, "CHARGE_COMPLETE", {}, ""});
                ct = charge_end;
            }

            if (bat + EPS < leg_e) return {};

            double to_del = append_segment_steps(steps, cx, cy, ct, del.x, del.y, true, &seg_to_del);
            if (to_del < 0) return {};
            double to_del_e = energy_for_leg(to_del, payload);
            if (bat + EPS < to_del_e) return {};
            bat -= to_del_e;
            steps.push_back({round2(del.x), round2(del.y), round2(ct), "DELIVER", {}, del.id});
            cx = del.x; cy = del.y;
            payload -= del.weight;
        }

        double ret_d = leg_distance_fast(cx, cy, warehouseX, warehouseY, ct);
        double ret_e = energy_for_leg(ret_d, 0.0);
        if (bat + EPS < ret_e) return {};
        PathResult seg_ret = query_segment(cx, cy, warehouseX, warehouseY, ct);
        if (!seg_ret.valid) return {};
        double to_wh = append_segment_steps(steps, cx, cy, ct, warehouseX, warehouseY, true, &seg_ret);
        if (to_wh < 0) return {};
        if (bat + EPS < energy_for_leg(to_wh, 0.0)) return {};
        bat -= energy_for_leg(to_wh, 0.0);
        steps.push_back({round2(warehouseX), round2(warehouseY), round2(ct), "RETURN", {}, ""});
        if (trip_end_out) *trip_end_out = round2(ct);
        return steps;
    };

    vector<vector<Step>> drone_paths(N);

    double max_payload_any = 0.0;
    for (const auto& d : drone_list) max_payload_any = max(max_payload_any, d.max_payload);

    vector<int> remaining;
    remaining.reserve(M);
    for (int i = 0; i < (int)delivery_list.size(); ++i) {
        if (delivery_list[i].weight <= max_payload_any + EPS)
            remaining.push_back(i);
    }
    sort(remaining.begin(), remaining.end(), [&](int a, int b) {
        return delivery_list[a].deadline < delivery_list[b].deadline;
    });

    vector<int> drone_order(N);
    for (int i = 0; i < N; ++i) drone_order[i] = i;
    auto try_solo_assign = [&](int idx) -> bool {
        for (int di : drone_order) {
            DroneState& ds = drone_states[di];
            if (delivery_list[idx].weight > ds.max_payload + EPS) continue;
            if (!can_complete_fast(ds, {idx})) continue;
            double trip_end = 0.0;
            vector<Step> trip_steps = build_trip({idx}, ds.current_time, ds.max_payload,
                                                 cs_bookings, &trip_end);
            if (trip_steps.empty()) continue;
            assigned[idx] = 1;
            drone_paths[di].insert(drone_paths[di].end(),
                make_move_iterator(trip_steps.begin()), make_move_iterator(trip_steps.end()));
            ds.x = warehouseX; ds.y = warehouseY;
            ds.current_time = trip_end;
            ds.current_battery = BATTERY_CAPACITY;
            return true;
        }
        return false;
    };
    auto pop_remaining = [&](int idx) {
        size_t wp = 0;
        for (size_t ri = 0; ri < remaining.size(); ++ri)
            if (remaining[ri] != idx) remaining[wp++] = remaining[ri];
        remaining.resize(wp);
    };
    auto rescue_drop = [&](int drop) {
        if (!try_solo_assign(drop)) remaining.erase(remaining.begin());
        else pop_remaining(drop);
    };
    while (!remaining.empty()) {
        if (large_job) {
            sort(drone_order.begin(), drone_order.end(), [&](int a, int b) {
                return drone_states[a].current_time < drone_states[b].current_time;
            });

            int assigned_cnt = 0;
            size_t scan_cap = remaining.size();
            if (M > 500 && scan_cap > 64) scan_cap = (size_t)max((int)(4 * N + 10), 32);
            for (int di : drone_order) {
                DroneState& ds = drone_states[di];
                int best_solo = -1;
                for (size_t ri = 0; ri < scan_cap; ++ri) {
                    int idx = remaining[ri];
                    if (assigned[idx]) continue;
                    const Delivery& del = delivery_list[idx];
                    if (del.weight > ds.max_payload + EPS) continue;
                    if (M > 500) {
                        double tr = leg_time_fast(ds.x, ds.y, del.x, del.y, ds.current_time);
                        if (ds.current_time + tr > del.deadline + EPS) continue;
                    } else {
                        if (ds.current_time + dist_to_warehouse[idx] / SPEED > del.deadline + EPS) continue;
                        if (!can_complete_fast(ds, {idx})) continue;
                    }
                    best_solo = idx;
                    break;
                }
                if (best_solo < 0) continue;

                vector<int> best_batch = {best_solo};
                if (M <= 500) {
                    double bw = delivery_list[best_solo].weight;
                    for (size_t ri2 = 0; ri2 < scan_cap; ++ri2) {
                        int idx2 = remaining[ri2];
                        if (idx2 == best_solo || assigned[idx2]) continue;
                        if (bw + delivery_list[idx2].weight > ds.max_payload + EPS) continue;
                        vector<int> trial = {best_solo, idx2};
                        order_batch(trial, ds.x, ds.y, ds.current_time);
                        if (can_complete_fast(ds, trial)) { best_batch = trial; break; }
                    }
                }

                double trip_end = 0.0;
                vector<Step> trip_steps = build_trip(best_batch, ds.current_time, ds.max_payload,
                                                     cs_bookings, &trip_end);
                if (trip_steps.empty() && best_batch.size() > 1) {
                    best_batch = {best_solo};
                    trip_steps = build_trip(best_batch, ds.current_time, ds.max_payload,
                                            cs_bookings, &trip_end);
                }
                if (trip_steps.empty()) {
                    for (int idx : best_batch) try_solo_assign(idx);
                    continue;
                }

                for (int idx : best_batch) assigned[idx] = 1;
                assigned_cnt += (int)best_batch.size();
                drone_paths[di].insert(drone_paths[di].end(),
                    make_move_iterator(trip_steps.begin()), make_move_iterator(trip_steps.end()));
                ds.x = warehouseX; ds.y = warehouseY;
                ds.current_time = trip_end;
                ds.current_battery = BATTERY_CAPACITY;
            }

            if (assigned_cnt == 0) { rescue_drop(remaining[0]); continue; }
            size_t write_pos = 0;
            for (size_t ri = 0; ri < remaining.size(); ++ri) {
                if (!assigned[remaining[ri]]) remaining[write_pos++] = remaining[ri];
            }
            remaining.resize(write_pos);
            continue;
        }

        sort(drone_order.begin(), drone_order.end(), [&](int a, int b) {
            return drone_states[a].current_time < drone_states[b].current_time;
        });

        int round_cnt = 0;
        char fd = 1;
        for (int di : drone_order) {
            vector<int> pool;
            for (int idx : remaining) {
                if (!assigned[idx]) pool.push_back(idx);
            }
            if (pool.empty()) continue;

            sort(pool.begin(), pool.end(), [&](int a, int b) {
                return delivery_list[a].deadline < delivery_list[b].deadline;
            });
            int urgent = fd ? pool[0] : -1;
            fd = 0;
            DroneState& ds = drone_states[di];
            vector<int> batch = try_build_batch(ds, pool, urgent);
            if (batch.empty()) continue;
            double trip_end = 0.0;
            vector<Step> trip_steps = build_trip(batch, ds.current_time, ds.max_payload,
                                                 cs_bookings, &trip_end);
            if (trip_steps.empty()) {
                for (int x : batch) try_solo_assign(x);
                continue;
            }
            for (int x : batch) assigned[x] = 1;
            round_cnt++;
            drone_paths[di].insert(drone_paths[di].end(),
                make_move_iterator(trip_steps.begin()),
                make_move_iterator(trip_steps.end()));
            ds.x = warehouseX;
            ds.y = warehouseY;
            ds.current_time = trip_end;
            ds.current_battery = BATTERY_CAPACITY;
        }

        if (round_cnt == 0) { rescue_drop(remaining[0]); continue; }

        size_t write_pos = 0;
        for (size_t ri = 0; ri < remaining.size(); ++ri) {
            if (!assigned[remaining[ri]]) remaining[write_pos++] = remaining[ri];
        }
        remaining.resize(write_pos);
    }

    for (int idx = 0; idx < M; ++idx)
        if (!assigned[idx]) try_solo_assign(idx);

    for (int di = 0; di < N; ++di) {
        if (drone_paths[di].empty()) continue;

        Json::Value entry;
        entry["drone_id"] = drone_list[di].id;
        Json::Value path(Json::arrayValue);

        const vector<Step>& steps = drone_paths[di];
        long long t_cent = 0;
        double prev_x = 0.0, prev_y = 0.0;
        for (size_t i = 0; i < steps.size(); ++i) {
            const Step& s = steps[i];
            double x = round2(s.x), y = round2(s.y);
            long long stored_cent = llround(round2(s.t) * 100.0);
            if (i == 0) {
                t_cent = llround(round2(s.t) * 100.0);
            } else if (s.action == "CHARGE" || s.action == "CHARGE_COMPLETE" ||
                       s.action == "WAIT") {
                t_cent = max(t_cent, stored_cent);
            } else {
                double d = dist(prev_x, prev_y, x, y);
                if (d > EPS) {
                    t_cent += llround(d * 100.0 / SPEED);
                } else {
                    t_cent = max(t_cent, stored_cent);
                }
            }
            prev_x = x;
            prev_y = y;

            Json::Value step;
            step["x"] = json_num(x);
            step["y"] = json_num(y);
            step["t"] = json_num(t_cent / 100.0);
            step["action"] = s.action;
            if (s.action == "PICKUP") {
                Json::Value ids(Json::arrayValue);
                for (const string& id : s.delivery_ids) ids.append(id);
                step["delivery_ids"] = ids;
            }
            if (s.action == "DELIVER") {
                step["delivery_id"] = s.delivery_id;
            }
            path.append(step);
        }
        entry["path"] = path;
        flight_manifest.append(entry);
    }

// End of BODY

// Start of TAIL
    Json::Value output;
    output["flight_manifest"] = flight_manifest;
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";
    cout << Json::writeString(wb, output) << endl;
    return 0;
}
// End of TAIL

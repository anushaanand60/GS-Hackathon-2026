#include <bits/stdc++.h>
using namespace std;

struct User {
    string name;
    int budget;
    int energy;
    set<string> tags;
    bool active = true;
};

struct Activity {
    int id;
    string name;
    int cost;
    int duration;
    int energy;
    string tag;
};

struct Input {
    int N, D, H;
    vector<User> users;
    map<int, Activity> activities;     // ordered by id
    vector<string> events;             // verbatim event lines
};

struct DayResult {
    vector<int> ids;
    int cost;
    int sat;
};

static Input readInput() {
    Input in;
    cin >> in.N >> in.D >> in.H;
    in.users.resize(in.N);
    for (int i = 0; i < in.N; i++) {
        int k;
        cin >> in.users[i].name >> in.users[i].budget >> in.users[i].energy >> k;
        for (int j = 0; j < k; j++) {
            string t;
            cin >> t;
            in.users[i].tags.insert(t);
        }
        in.users[i].active = true;
    }
    int A;
    cin >> A;
    for (int i = 0; i < A; i++) {
        Activity a;
        cin >> a.id >> a.name >> a.cost >> a.duration >> a.energy >> a.tag;
        in.activities[a.id] = a;
    }
    int E;
    cin >> E;
    cin.ignore();
    for (int i = 0; i < E; i++) {
        string line;
        getline(cin, line);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        in.events.push_back(line);
    }
    return in;
}

/** Format a single day line exactly per spec. Use REST if ids is empty. */
static string formatDay(int day, vector<int> ids, int cost, int sat) {
    if (ids.empty()) {
        return "Day " + to_string(day) + ": REST | cost=0 satisfaction=0";
    }
    sort(ids.begin(), ids.end());
    string s = "Day " + to_string(day) + ": ";
    for (size_t i = 0; i < ids.size(); i++) {
        if (i) s += ' ';
        s += to_string(ids[i]);
    }
    s += " | cost=" + to_string(cost) + " satisfaction=" + to_string(sat);
    return s;
}

// =========================================================================
// YOUR CODE GOES HERE.
//
// Implement solve(const Input& in) and return the FULL output string
// (including the trailing newline) that the judge will diff against
// the expected output.
//
// Helpers available from the Head section:
//   - formatDay(day, ids, cost, sat)  -> properly formatted "Day X: ..." line
//   - in.users        : vector<User>           ({name, budget, energy, tags, active})
//   - in.activities   : map<int,Activity>      ({id, name, cost, duration, energy, tag})
//   - in.events       : vector<string>         verbatim event lines, e.g.
//                       "DROP 2 Bob", "WEATHER 3 NATURE",
//                       "FATIGUE 2 Alice 5", "BUDGET 4 Alice 20"
// =========================================================================
struct Partial {
    int cost;
    int energy;
    int dur;
    int sat;
    vector<int> ids;
};

static bool dominatesPartial(const Partial& a, const Partial& b) {
    if (a.cost > b.cost || a.energy > b.energy || a.dur > b.dur) return false;
    if (a.sat > b.sat) return true;
    if (a.sat < b.sat) return false;
    if (a.cost < b.cost) return true;
    if (a.cost > b.cost) return false;
    return a.ids < b.ids;
}

static vector<Partial> prunePartials(vector<Partial> v) {
    vector<Partial> kept;
    kept.reserve(v.size());
    for (Partial& p : v) {
        bool dominated = false;
        for (const Partial& q : kept) {
            if (dominatesPartial(q, p)) {
                dominated = true;
                break;
            }
        }
        if (dominated) continue;
        kept.erase(
            remove_if(kept.begin(), kept.end(), [&](const Partial& q) { return dominatesPartial(p, q); }),
            kept.end());
        kept.push_back(move(p));
    }
    return kept;
}

static vector<Partial> enumerateHalf(
    const vector<const Activity*>& eligible,
    const vector<int>& act_sat,
    int start,
    int len) {
    vector<Partial> res;
    res.reserve(1u << len);
    for (int mask = 0; mask < (1 << len); mask++) {
        Partial p{0, 0, 0, 0, {}};
        for (int i = 0; i < len; i++) {
            if (!((mask >> i) & 1)) continue;
            const Activity* a = eligible[start + i];
            p.cost += a->cost;
            p.energy += a->energy;
            p.dur += a->duration;
            p.sat += act_sat[start + i];
            p.ids.push_back(a->id);
        }
        sort(p.ids.begin(), p.ids.end());
        res.push_back(move(p));
    }
    return res;
}

static DayResult bestDay(
    int day,
    const vector<User>& users,
    const map<int, Activity>& acts,
    const set<int>& used_ids,
    const map<int, set<string>>& weather_block,
    int H) {
    vector<const User*> active;
    for (const auto& u : users) {
        if (u.active) active.push_back(&u);
    }
    if (active.empty()) return {{}, 0, 0};

    int budget_lim = INT_MAX;
    int energy_lim = INT_MAX;
    for (const User* u : active) {
        budget_lim = min(budget_lim, u->budget);
        energy_lim = min(energy_lim, u->energy);
    }

    vector<const Activity*> eligible;
    for (const auto& [id, a] : acts) {
        if (used_ids.count(id)) continue;
        auto wit = weather_block.find(day);
        if (wit != weather_block.end() && wit->second.count(a.tag)) continue;
        eligible.push_back(&a);
    }

    const int n = (int)eligible.size();
    vector<int> act_sat(n);
    for (int i = 0; i < n; i++) {
        for (const User* u : active) {
            if (u->tags.count(eligible[i]->tag)) act_sat[i]++;
        }
    }

    DayResult best = {{}, 0, 0};

    auto isBetter = [](int sat, int tc, const vector<int>& ids, const DayResult& cur) {
        if (sat > cur.sat) return true;
        if (sat < cur.sat) return false;
        if (tc < cur.cost) return true;
        if (tc > cur.cost) return false;
        return ids < cur.ids;
    };

    auto consider = [&](int tc, int te, int td, int sat, const vector<int>& ids) {
        if (tc > budget_lim || te > energy_lim || td > H) return;
        if (isBetter(sat, tc, ids, best)) {
            best.ids = ids;
            best.cost = tc;
            best.sat = sat;
        }
    };

    auto considerMove = [&](int tc, int te, int td, int sat, vector<int>&& ids) {
        if (tc > budget_lim || te > energy_lim || td > H) return;
        if (isBetter(sat, tc, ids, best)) {
            best = {move(ids), tc, sat};
        }
    };

    if (n <= 14) {
        vector<int> order(n);
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int i, int j) {
            return eligible[i]->cost < eligible[j]->cost;
        });
        vector<const Activity*> sorted_eligible(n);
        vector<int> sorted_sat(n);
        for (int i = 0; i < n; i++) {
            sorted_eligible[i] = eligible[order[i]];
            sorted_sat[i] = act_sat[order[i]];
        }
        eligible = move(sorted_eligible);
        act_sat = move(sorted_sat);

        for (int mask = 0; mask < (1 << n); mask++) {
            int tc = 0, te = 0, td = 0, sat = 0;
            vector<int> ids;
            bool feasible = true;

            for (int i = 0; i < n; i++) {
                if (!((mask >> i) & 1)) continue;
                const Activity* a = eligible[i];
                tc += a->cost;
                if (tc > budget_lim) {
                    feasible = false;
                    break;
                }
                te += a->energy;
                if (te > energy_lim) {
                    feasible = false;
                    break;
                }
                td += a->duration;
                if (td > H) {
                    feasible = false;
                    break;
                }
                sat += act_sat[i];
                ids.push_back(a->id);
            }
            if (!feasible) continue;

            sort(ids.begin(), ids.end());
            consider(tc, te, td, sat, ids);
        }
        return best;
    }

    const int n1 = n / 2;
    vector<Partial> left = enumerateHalf(eligible, act_sat, 0, n1);
    vector<Partial> right = enumerateHalf(eligible, act_sat, n1, n - n1);

    for (const Partial& p : left) {
        consider(p.cost, p.energy, p.dur, p.sat, p.ids);
    }
    for (const Partial& p : right) {
        if (!p.ids.empty()) {
            consider(p.cost, p.energy, p.dur, p.sat, p.ids);
        }
    }

    left = prunePartials(move(left));
    sort(left.begin(), left.end(), [](const Partial& a, const Partial& b) {
        return a.cost < b.cost;
    });

    vector<int> merged;
    merged.reserve(n);

    for (const Partial& r : right) {
        if (r.ids.empty()) continue;
        for (const Partial& l : left) {
            if (l.ids.empty()) continue;
            const int tc = l.cost + r.cost;
            if (tc > budget_lim) break;
            const int te = l.energy + r.energy;
            if (te > energy_lim) continue;
            const int td = l.dur + r.dur;
            if (td > H) continue;
            merged.clear();
            merged.insert(merged.end(), l.ids.begin(), l.ids.end());
            merged.insert(merged.end(), r.ids.begin(), r.ids.end());
            considerMove(tc, te, td, l.sat + r.sat, move(merged));
        }
    }
    return best;
}

static int parseEventDay(const string& evline) {
    istringstream ss(evline);
    string etype;
    ss >> etype;
    int event_day = 0;
    if (etype == "WEATHER") {
        string tag;
        ss >> event_day >> tag;
    } else if (etype == "DROP") {
        string name;
        ss >> event_day >> name;
    } else if (etype == "FATIGUE") {
        string name;
        int ne;
        ss >> event_day >> name >> ne;
    } else if (etype == "BUDGET") {
        string name;
        int nb;
        ss >> event_day >> name >> nb;
    }
    return event_day;
}

static void applyEvent(const string& evline, vector<User>& users, map<int, set<string>>& weather_block) {
    istringstream ss(evline);
    string etype;
    ss >> etype;

    if (etype == "WEATHER") {
        int event_day;
        string tag;
        ss >> event_day >> tag;
        weather_block[event_day].insert(tag);
    } else if (etype == "DROP") {
        int event_day;
        string name;
        ss >> event_day >> name;
        for (auto& u : users) {
            if (u.name == name) u.active = false;
        }
    } else if (etype == "FATIGUE") {
        int event_day;
        string name;
        int ne;
        ss >> event_day >> name >> ne;
        for (auto& u : users) {
            if (u.name == name) u.energy = ne;
        }
    } else if (etype == "BUDGET") {
        int event_day;
        string name;
        int nb;
        ss >> event_day >> name >> nb;
        for (auto& u : users) {
            if (u.name == name) u.budget = nb;
        }
    }
}



static string solve(Input in) {
    set<int> used_ids;
    map<int, set<string>> weather_block;
    map<int, DayResult> day_results;
    string out;
    out += "=== PLAN ===\n";

    // TODO 1: build the initial D-day plan.
    //   for (int day = 1; day <= in.D; day++) {
    //       out += formatDay(day, chosenIds, totalCost, satisfaction) + "\n";
    //   }

    // TODO 2: process events in order. For each event i (1-indexed):
    //   - append "=== EVENT i: <event line verbatim> ===\n"
    //   - mutate state (DROP / WEATHER / FATIGUE / BUDGET)
    //   - re-plan days [eventDay .. D], preserving days [1 .. eventDay-1]
    for (int d = 1; d <= in.D; d++) {
        day_results[d] = bestDay(d, in.users, in.activities, used_ids, weather_block, in.H);
        for (int id : day_results[d].ids) used_ids.insert(id);
        out += formatDay(d, day_results[d].ids, day_results[d].cost, day_results[d].sat) + "\n";
    }

    for (int ei = 0; ei < (int)in.events.size(); ei++) {
        const string& evline = in.events[ei];
        out += "=== EVENT " + to_string(ei + 1) + ": " + evline + " ===\n";

        applyEvent(evline, in.users, weather_block);
        int event_day = parseEventDay(evline);

        used_ids.clear();
        for (int d = 1; d < event_day; d++) {
            for (int id : day_results[d].ids) used_ids.insert(id);
        }

        for (int d = event_day; d <= in.D; d++) {
            day_results[d] = bestDay(d, in.users, in.activities, used_ids, weather_block, in.H);
            for (int id : day_results[d].ids) used_ids.insert(id);
            out += formatDay(d,day_results[d].ids,day_results[d].cost, day_results[d].sat)+ "\n";
        }
    }
    return out;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    Input in = readInput();
    cout << solve(in);
    return 0;
}

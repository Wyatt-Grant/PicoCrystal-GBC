// Host-side structural validator for the Second Story campaign. Compiles the
// pure level data (assets.hpp, no SDK) and checks each level is *structurally*
// solvable: objectives reachable (doors assumed openable, teleporters linked),
// teleporter pairs complete, and every door channel has a matching actuator.
//
//   g++ -std=c++17 -I.. tools/validate.cpp -o /tmp/validate && /tmp/validate
//
// This does NOT prove the ghost-timing puzzle is solvable (that needs play), but
// it catches walled-off loot/exits, unpaired warps, and dangling channels.

#include "assets.hpp"
#include <cstdio>
#include <queue>
#include <cstring>

static int fails = 0;
#define CHECK(cond, ...) do { if(!(cond)) { printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while(0)

static bool walkable(uint8_t t) { return t != T_WALL; }               // doors openable
static int dchan(uint8_t t){ return (t>=20&&t<=23)?t-20:-1; }
static int pchan(uint8_t t){ return (t>=10&&t<=13)?t-10:-1; }
static int schan(uint8_t t){ return (t>=30&&t<=33)?t-30:-1; }
static int kchan(uint8_t t){ return (t>=40&&t<=43)?t-40:-1; }
static int tid(uint8_t t){ return (t>=60&&t<=63)?t-60:-1; }

int main() {
  printf("Validating %d levels\n", NUM_LEVELS);
  for (int n = 0; n < NUM_LEVELS; n++) {
    const Level &L = LEVELS[n];
    printf("[%2d] %s (%d crew)\n", n+1, L.title, L.crew);

    // Locate features.
    int spawnC=-1,spawnR=-1,safeC=-1,safeR=-1,lootC=-1,lootR=-1,exitC=-1,exitR=-1,termC=-1,termR=-1;
    int teleN[MAX_TELE]={0}; uint8_t teleP[MAX_TELE][2][2];
    bool hasPlate[NUM_CHAN]={}, hasSwitch[NUM_CHAN]={}, hasKey[NUM_CHAN]={}, hasDoor[NUM_CHAN]={};
    for (int r=0;r<GRID;r++) for (int c=0;c<GRID;c++) {
      uint8_t t=L.map[r][c];
      if(t==T_SPAWN){spawnC=c;spawnR=r;} else if(t==T_SAFE){safeC=c;safeR=r;}
      else if(t==T_LOOT){lootC=c;lootR=r;} else if(t==T_EXIT){exitC=c;exitR=r;}
      else if(t==T_TERMINAL){termC=c;termR=r;}
      if(pchan(t)>=0)hasPlate[pchan(t)]=true;
      if(schan(t)>=0)hasSwitch[schan(t)]=true;
      if(kchan(t)>=0)hasKey[kchan(t)]=true;
      if(dchan(t)>=0)hasDoor[dchan(t)]=true;
      int id=tid(t); if(id>=0&&teleN[id]<2){teleP[id][teleN[id]][0]=c;teleP[id][teleN[id]][1]=r;teleN[id]++;}
    }
    CHECK(spawnC>=0, "no spawn");
    CHECK(exitC>=0,  "no exit");
    CHECK(lootC>=0,  "no loot");
    CHECK(safeC>=0,  "no safe");

    // Names/briefs arity (can't size arrays, but null-check within crew).
    for (int i=0;i<L.crew;i++){ CHECK(L.names[i]&&L.names[i][0], "crew %d has no name", i);
                                CHECK(L.briefs[i]&&L.briefs[i][0], "crew %d has no brief", i); }

    // Channel actuator wiring.
    for (int ch=0;ch<NUM_CHAN;ch++){
      ChanMode m=(ChanMode)L.chanMode[ch];
      if(hasDoor[ch]){
        if(m==OR_PLATE||m==AND_PLATE) CHECK(hasPlate[ch], "door ch%d needs a plate", ch);
        if(m==MODE_SWITCH||m==TIMED_SWITCH) CHECK(hasSwitch[ch], "door ch%d needs a switch", ch);
        if(m==MODE_KEY) CHECK(hasKey[ch], "door ch%d needs a key", ch);
      }
    }
    // Teleporters must be paired.
    for (int id=0;id<MAX_TELE;id++) CHECK(teleN[id]==0||teleN[id]==2, "tele id%d has %d pads (need 0 or 2)", id, teleN[id]);
    // Lasers must reference a channel with an actuator (switch/plate) so they can drop.
    for (int i=0;i<L.laserCount;i++){ int ch=L.lasers[i].channel;
      CHECK(hasSwitch[ch]||hasPlate[ch]||hasKey[ch], "laser %d on ch%d has no way to disable", i, ch); }

    // Flood fill from spawn; doors passable, teleporters linked.
    bool seen[GRID][GRID]={}; std::queue<std::pair<int,int>> q;
    q.push({spawnC,spawnR}); seen[spawnR][spawnC]=true;
    auto push=[&](int c,int r){ if(c>=0&&r>=0&&c<GRID&&r<GRID&&!seen[r][c]&&walkable(L.map[r][c])){seen[r][c]=true;q.push({c,r});} };
    while(!q.empty()){ auto[c,r]=q.front();q.pop();
      push(c+1,r);push(c-1,r);push(c,r+1);push(c,r-1);
      int id=tid(L.map[r][c]); if(id>=0&&teleN[id]==2){ int e=(teleP[id][0][0]==c&&teleP[id][0][1]==r)?1:0; push(teleP[id][e][0],teleP[id][e][1]); }
    }
    auto reach=[&](int c,int r){ return c>=0&&r>=0&&seen[r][c]; };
    auto reachAdj=[&](int c,int r){ return reach(c+1,r)||reach(c-1,r)||reach(c,r+1)||reach(c,r-1)||reach(c,r); };

    if(lootC>=0) CHECK(reach(lootC,lootR), "loot tile unreachable");
    if(exitC>=0) CHECK(reach(exitC,exitR), "exit unreachable");
    if(safeC>=0) CHECK(reachAdj(safeC,safeR), "safe not reachable to crack");
    if(termC>=0) CHECK(reachAdj(termC,termR), "terminal not reachable to hack");
    for (int r=0;r<GRID;r++) for (int c=0;c<GRID;c++){ uint8_t t=L.map[r][c];
      if(pchan(t)>=0||schan(t)>=0||kchan(t)>=0) CHECK(reach(c,r), "actuator at %d,%d unreachable", c,r);
      if(tid(t)>=0) CHECK(reach(c,r), "teleporter pad at %d,%d unreachable", c,r);
    }
  }
  printf(fails? "\n%d CHECK(S) FAILED\n" : "\nAll structural checks passed.\n", fails);
  return fails?1:0;
}

/*
  Schelling's Model of Segregation — Interactive Simulation
  - Two agent types (A/B), 0 = empty
  - Neighborhood: Moore radius 1 (8 neighbors)
  - Satisfaction if fraction of similar neighbors >= tolerance
  - Unsatisfied agents move to random empty cells each step (parallel moves)
*/

(function() {
  // DOM elements
  const els = {
    gridSize: document.getElementById('gridSize'),
    gridSizeValue: document.getElementById('gridSizeValue'),
    gridSizeValue2: document.getElementById('gridSizeValue2'),
    density: document.getElementById('density'),
    densityValue: document.getElementById('densityValue'),
    ratioA: document.getElementById('ratioA'),
    ratioAValue: document.getElementById('ratioAValue'),
    tolerance: document.getElementById('tolerance'),
    toleranceValue: document.getElementById('toleranceValue'),
    delay: document.getElementById('delay'),
    delayValue: document.getElementById('delayValue'),
    maxIterations: document.getElementById('maxIterations'),
    wrapEdges: document.getElementById('wrapEdges'),
    showUnsatisfied: document.getElementById('showUnsatisfied'),
    initializeBtn: document.getElementById('initializeBtn'),
    stepBtn: document.getElementById('stepBtn'),
    runBtn: document.getElementById('runBtn'),
    runBtnLabel: document.querySelector('#runBtn .label'),
    simTitle: document.getElementById('simTitle'),
    canvas: document.getElementById('gridCanvas'),
    statStep: document.getElementById('statStep'),
    statAgents: document.getElementById('statAgents'),
    statUnsatisfied: document.getElementById('statUnsatisfied'),
    statSatisfied: document.getElementById('statSatisfied'),
    statMoved: document.getElementById('statMoved'),
    // Tabs and per-sim controls
    tabSchelling: document.getElementById('tabSchelling'),
    tabLife: document.getElementById('tabLife'),
    tabBoids: document.getElementById('tabBoids'),
    tabFireflies: document.getElementById('tabFireflies'),
    schellingPanel: document.getElementById('schellingPanel'),
    lifePanel: document.getElementById('lifePanel'),
    boidsPanel: document.getElementById('boidsPanel'),
    firefliesPanel: document.getElementById('firefliesPanel'),
    lifePattern: document.getElementById('lifePattern'),
    unsatRow: document.getElementById('unsatRow'),
    schellingRules: document.getElementById('schellingRules'),
    lifeRules: document.getElementById('lifeRules'),
    boidsRules: document.getElementById('boidsRules'),
    schellingLegend: document.getElementById('schellingLegend'),
    lifeLegend: document.getElementById('lifeLegend'),
    boidsLegend: document.getElementById('boidsLegend'),
    firefliesLegend: document.getElementById('firefliesLegend'),
    // Boids controls
    boidsCount: document.getElementById('boidsCount'),
    boidsCountValue: document.getElementById('boidsCountValue'),
    perception: document.getElementById('perception'),
    perceptionValue: document.getElementById('perceptionValue'),
    separationDist: document.getElementById('separationDist'),
    separationDistValue: document.getElementById('separationDistValue'),
    maxSpeed: document.getElementById('maxSpeed'),
    maxSpeedValue: document.getElementById('maxSpeedValue'),
    maxForce: document.getElementById('maxForce'),
    maxForceValue: document.getElementById('maxForceValue'),
    weightSep: document.getElementById('weightSep'),
    weightSepValue: document.getElementById('weightSepValue'),
    weightAlign: document.getElementById('weightAlign'),
    weightAlignValue: document.getElementById('weightAlignValue'),
    weightCoh: document.getElementById('weightCoh'),
    weightCohValue: document.getElementById('weightCohValue'),
    // Schelling edit tools
    schellingEditEnabled: document.getElementById('schellingEditEnabled'),
    schellingClearBtn: document.getElementById('schellingClearBtn'),
    // Life edit tools
    lifeEditEnabled: document.getElementById('lifeEditEnabled'),
    lifeClearBtn: document.getElementById('lifeClearBtn'),
    boidsTrails: document.getElementById('boidsTrails'),
    trailPersistence: document.getElementById('trailPersistence'),
    trailPersistenceValue: document.getElementById('trailPersistenceValue'),
    mouseForce: document.getElementById('mouseForce'),
    mouseForceValue: document.getElementById('mouseForceValue'),
    wAvoid: document.getElementById('wAvoid'),
    wAvoidValue: document.getElementById('wAvoidValue'),
    // Fireflies controls
    ffRadius: document.getElementById('ffRadius'),
    ffRadiusValue: document.getElementById('ffRadiusValue'),
    ffCoupling: document.getElementById('ffCoupling'),
    ffCouplingValue: document.getElementById('ffCouplingValue'),
    ffPeriod: document.getElementById('ffPeriod'),
    ffPeriodValue: document.getElementById('ffPeriodValue'),
    ffJitter: document.getElementById('ffJitter'),
    ffJitterValue: document.getElementById('ffJitterValue'),
  };

  // Utility: RNG and shuffle
  function shuffle(array) {
    for (let i = array.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [array[i], array[j]] = [array[j], array[i]];
    }
  }

  // Mouse tracking for Boids
  let mousePos = { x: 0, y: 0, inside: false };

  // Drag-to-paint state
  let dragging = false;
  let didDrag = false;
  let paintedSet = new Set();
  let paintTarget = null; // target state chosen on first press

  function applyEditAtCell(cellX, cellY) {
    if (!model) return false;
    if (currentSim === 'schelling') {
      if (!(els.schellingEditEnabled && els.schellingEditEnabled.checked)) return false;
      const idx = model.index(cellX, cellY);
      const v = model.grid[idx];
      const nv = (dragging && paintTarget !== null) ? paintTarget : (v === 0 ? 1 : (v === 1 ? 2 : 0));
      if (nv !== v) { model.grid[idx] = nv; return true; }
      return false;
    } else if (currentSim === 'life') {
      const idx = model.index(cellX, cellY);
      const v = model.grid[idx];
      const nv = (dragging && paintTarget !== null) ? paintTarget : (v ? 0 : 1);
      if (nv !== v) { model.grid[idx] = nv; return true; }
      return false;
    }
    return false;
  }


  class BoidsModel {
    constructor(opts) {
      this.wrapEdges = !!opts.wrapEdges;
      this.width = opts.width; this.height = opts.height;
      this.perception = opts.perception;
      this.separationDist = opts.separationDist;
      this.maxSpeed = opts.maxSpeed;
      this.maxForce = opts.maxForce;
      this.wSep = opts.wSep; this.wAlign = opts.wAlign; this.wCoh = opts.wCoh;
      this.wAvoid = opts.wAvoid ?? 1.2;
      this.mouseForce = opts.mouseForce ?? 0;
      this.boids = [];
      this.obstacles = [];
      for (let i = 0; i < opts.count; i++) {
        this.boids.push({
          x: Math.random() * this.width,
          y: Math.random() * this.height,
          vx: (Math.random() * 2 - 1) * this.maxSpeed,
          vy: (Math.random() * 2 - 1) * this.maxSpeed,
        });
      }
      this.stepCount = 0;
    }
    count() { return this.boids.length; }
    addObstacle(x, y, r=24) { this.obstacles.push({ x, y, r }); }
    removeNearestObstacle(x, y) {
      if (!this.obstacles.length) return;
      let best = 0, bestD2 = Infinity;
      for (let i=0;i<this.obstacles.length;i++) {
        const o = this.obstacles[i];
        const dx = o.x - x, dy = o.y - y; const d2 = dx*dx + dy*dy;
        if (d2 < bestD2) { best = i; bestD2 = d2; }
      }
      this.obstacles.splice(best,1);
    }
    step() {
      const b = this.boids;
      const n = b.length;
      for (let i = 0; i < n; i++) {
        const me = b[i];
        let steerSepX = 0, steerSepY = 0;
        let alignX = 0, alignY = 0, alignCount = 0;
        let cohX = 0, cohY = 0, cohCount = 0;
        let avoidX = 0, avoidY = 0;
        for (let j = 0; j < n; j++) {
          if (i === j) continue;
          const other = b[j];
          let dx = other.x - me.x, dy = other.y - me.y;
          if (this.wrapEdges) {
            if (Math.abs(dx) > this.width/2) dx -= Math.sign(dx) * this.width;
            if (Math.abs(dy) > this.height/2) dy -= Math.sign(dy) * this.height;
          }
          const d2 = dx*dx + dy*dy;
          if (d2 <= this.perception*this.perception) {
            alignX += other.vx; alignY += other.vy; alignCount++;
            cohX += other.x; cohY += other.y; cohCount++;
            const d = Math.sqrt(d2) || 1e-6;
            if (d < this.separationDist) {
              const inv = 1 / d;
              steerSepX -= dx * inv; steerSepY -= dy * inv;
            }
          }
        }
        
        if (alignCount > 0) {
          alignX /= alignCount; alignY /= alignCount;
          const mag = Math.hypot(alignX, alignY) || 1;
          alignX = (alignX / mag) * this.maxSpeed - me.vx;
          alignY = (alignY / mag) * this.maxSpeed - me.vy;
        }
        if (cohCount > 0) {
          cohX = (cohX / cohCount) - me.x;
          cohY = (cohY / cohCount) - me.y;
          if (this.wrapEdges) {
            if (Math.abs(cohX) > this.width/2) cohX -= Math.sign(cohX)*this.width;
            if (Math.abs(cohY) > this.height/2) cohY -= Math.sign(cohY)*this.height;
          }
          const mag = Math.hypot(cohX, cohY) || 1;
          cohX = (cohX / mag) * this.maxSpeed - me.vx;
          cohY = (cohY / mag) * this.maxSpeed - me.vy;
        }
        const sepMag = Math.hypot(steerSepX, steerSepY) || 0;
        if (sepMag > 0) { steerSepX = (steerSepX/sepMag)*this.maxSpeed - me.vx; steerSepY = (steerSepY/sepMag)*this.maxSpeed - me.vy; }

        // Mouse attraction/repulsion
        let mAX = 0, mAY = 0;
        if (this.mouseForce && mousePos.inside) {
          let dx = mousePos.x - me.x, dy = mousePos.y - me.y;
          if (this.wrapEdges) {
            if (Math.abs(dx) > this.width/2) dx -= Math.sign(dx) * this.width;
            if (Math.abs(dy) > this.height/2) dy -= Math.sign(dy) * this.height;
          }
          const d = Math.hypot(dx, dy) || 1e-6;
          const dirX = dx / d, dirY = dy / d;
          mAX = dirX * this.mouseForce;
          mAY = dirY * this.mouseForce;
        }

        let ax = this.wSep * steerSepX + this.wAlign * alignX + this.wCoh * cohX + this.wAvoid * avoidX + mAX;
        let ay = this.wSep * steerSepY + this.wAlign * alignY + this.wCoh * cohY + this.wAvoid * avoidY + mAY;
        const aMag = Math.hypot(ax, ay) || 0;
        if (aMag > this.maxForce) { ax = ax / aMag * this.maxForce; ay = ay / aMag * this.maxForce; }

        me.vx += ax; me.vy += ay;
        const vMag = Math.hypot(me.vx, me.vy) || 1;
        if (vMag > this.maxSpeed) { me.vx = me.vx / vMag * this.maxSpeed; me.vy = me.vy / vMag * this.maxSpeed; }
        me.x += me.vx; me.y += me.vy;
        if (this.wrapEdges) {
          if (me.x < 0) me.x += this.width; if (me.x >= this.width) me.x -= this.width;
          if (me.y < 0) me.y += this.height; if (me.y >= this.height) me.y -= this.height;
        } else {
          if (me.x < 0 || me.x > this.width) { me.vx *= -1; me.x = Math.max(0, Math.min(this.width, me.x)); }
          if (me.y < 0 || me.y > this.height) { me.vy *= -1; me.y = Math.max(0, Math.min(this.height, me.y)); }
        }
      }
      this.stepCount++;
      return { changed: true };
    }
  }
  // Fireflies model (grid-based Kuramoto-like coupling)
  class FirefliesModel {
    constructor(size, densityPct, wrapEdges, radius, coupling, basePeriod, jitterPct) {
      this.size = size | 0;
      this.wrapEdges = !!wrapEdges;
      this.grid = new Uint8Array(this.size * this.size); // 0 empty, 1 firefly
      this.theta = new Float32Array(this.size * this.size); // phase [0, 2pi)
      this.omega = new Float32Array(this.size * this.size); // natural freq (rad/step)
      this.flash = new Uint8Array(this.size * this.size); // flash cooldown frames
      this.radius = Math.max(1, radius|0);
      this.coupling = Math.max(0, Math.min(1, coupling));
      this.basePeriod = Math.max(1, basePeriod|0);
      this.jitter = Math.max(0, Math.min(1, (jitterPct||0)/100));
      this.stepCount = 0;
      this.randomize(densityPct);
    }
    index(x,y){return y*this.size+x;}
    inBounds(x,y){return x>=0&&y>=0&&x<this.size&&y<this.size;}
    randomize(densityPct){
      const p = Math.max(0, Math.min(100, densityPct))/100;
      const n = this.size*this.size;
      for (let i=0;i<n;i++) {
        const present = Math.random() < p ? 1 : 0;
        this.grid[i] = present;
        // random initial phase
        this.theta[i] = Math.random()*Math.PI*2;
        // natural period with jitter
        const u = (Math.random()*2 - 1) * this.jitter; // [-jitter, +jitter]
        const Ti = this.basePeriod * (1 + u);
        this.omega[i] = (Math.PI*2) / Math.max(1e-3, Ti);
        this.flash[i] = 0;
      }
    }
    step(){
      const n = this.size;
      const R = this.radius;
      const K = this.coupling;
      let flashed = 0;
      // compute phase increments with local coupling
      const dtheta = new Float32Array(n*n);
      for (let y=0;y<n;y++){
        for (let x=0;x<n;x++){
          const idx = this.index(x,y);
          if (!this.grid[idx]) { dtheta[idx]=0; continue; }
          const th = this.theta[idx];
          let sum = 0; let count = 0;
          for (let dy=-R; dy<=R; dy++){
            for (let dx=-R; dx<=R; dx++){
              if (dx===0 && dy===0) continue;
              let nx = x+dx, ny = y+dy;
              if (this.wrapEdges) { nx=(nx+n)%n; ny=(ny+n)%n; }
              else if (!this.inBounds(nx,ny)) continue;
              const j = this.index(nx,ny);
              if (!this.grid[j]) continue;
              // Kuramoto coupling contribution sin(theta_j - theta_i)
              const diff = this.theta[j] - th;
              sum += Math.sin(diff);
              count++;
            }
          }
          const meanCouple = count>0 ? (sum / count) : 0;
          dtheta[idx] = this.omega[idx] + K * meanCouple;
        }
      }
      // integrate and detect flashes
      for (let i=0;i<n*n;i++){
        if (!this.grid[i]) { if (this.flash[i]>0) this.flash[i]=0; continue; }
        let th = this.theta[i] + dtheta[i];
        if (th >= Math.PI*2) {
          th -= Math.PI*2;
          this.flash[i] = 6; // frames to show flash
          flashed++;
        }
        this.theta[i] = th;
        if (this.flash[i]>0) this.flash[i]--;
      }
      this.stepCount++;
      return { changed: true, flashed };
    }
    countAgents(){ let a=0; for (let i=0;i<this.grid.length;i++) if (this.grid[i]) a++; return a; }
  }
  // Conway's Game of Life model
  class LifeModel {
    constructor(size, densityPct, wrapEdges) {
      this.size = size | 0;
      this.wrapEdges = !!wrapEdges;
      this.grid = new Uint8Array(this.size * this.size); // 0 dead, 1 alive
      this.stepCount = 0;
      this.randomize(densityPct);
    }
    index(x, y) { return y * this.size + x; }
    inBounds(x, y) { return x >= 0 && y >= 0 && x < this.size && y < this.size; }
    randomize(densityPct) {
      const p = Math.max(0, Math.min(100, densityPct)) / 100;
      const n = this.size * this.size;
      for (let i = 0; i < n; i++) this.grid[i] = Math.random() < p ? 1 : 0;
      this.stepCount = 0;
    }
    setAllDead() { this.grid.fill(0); }
    neighborsAlive(x, y) {
      let alive = 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          let nx = x + dx, ny = y + dy;
          if (this.wrapEdges) {
            nx = (nx + this.size) % this.size;
            ny = (ny + this.size) % this.size;
          } else if (!this.inBounds(nx, ny)) {
            continue;
          }
          alive += this.grid[this.index(nx, ny)] ? 1 : 0;
        }
      }
      return alive;
    }
    step() {
      const next = new Uint8Array(this.grid.length);
      let changes = 0;
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          const idx = this.index(x, y);
          const a = this.grid[idx] === 1;
          const n = this.neighborsAlive(x, y);
          let nv = 0;
          if (a) nv = (n === 2 || n === 3) ? 1 : 0;
          else nv = (n === 3) ? 1 : 0;
          next[idx] = nv;
          if (nv !== this.grid[idx]) changes++;
        }
      }
      this.grid = next;
      this.stepCount++;
      return { changed: changes };
    }
    countAlive() {
      let alive = 0;
      for (let i = 0; i < this.grid.length; i++) alive += this.grid[i];
      return alive;
    }
    seedPattern(name) {
      this.setAllDead();
      const s = this.size;
      const cx = Math.floor(s / 2), cy = Math.floor(s / 2);
      const set = (x, y) => { if (x>=0&&y>=0&&x<s&&y<s) this.grid[this.index(x,y)] = 1; };
      if (name === 'glider') {
        [[1,0],[2,1],[0,2],[1,2],[2,2]].forEach(([dx,dy])=>set(cx+dx-1, cy+dy-1));
      } else if (name === 'blinker') {
        [[-1,0],[0,0],[1,0]].forEach(([dx,dy])=>set(cx+dx, cy+dy));
      } else if (name === 'pulsar') {
        const coords = [
          [-6,-4],[-5,-4],[-4,-4],[-2,-4],[-1,-4],[0,-4],[2,-4],[3,-4],[4,-4],
          [-6,4],[-5,4],[-4,4],[-2,4],[-1,4],[0,4],[2,4],[3,4],[4,4],
          [-4,-6],[-4,-5],[-4,-4],[-4,-2],[-4,-1],[-4,0],[4,-6],[4,-5],[4,-4],[4,-2],[4,-1],[4,0],
          [-1,-1],[-1,-2],[-1,-4],[-1,-5],[-1,-6],[1,-1],[1,-2],[1,-4],[1,-5],[1,-6],
          [-1,1],[-1,2],[-1,4],[-1,5],[-1,6],[1,1],[1,2],[1,4],[1,5],[1,6],
          [-4,6],[-4,5],[-4,4],[-4,2],[-4,1],[-4,0],[4,6],[4,5],[4,4],[4,2],[4,1],[4,0]
        ];
        coords.forEach(([dx,dy])=>set(cx+dx, cy+dy));
      } else if (name === 'gosper') {
        const gun = [
          [1,5],[1,6],[2,5],[2,6],
          [13,3],[14,3],[12,4],[16,4],[11,5],[17,5],[11,6],[15,6],[17,6],[18,6],[11,7],[17,7],[12,8],[16,8],[13,9],[14,9],
          [23,1],[24,1],[22,2],[26,2],[21,3],[27,3],[21,4],[25,4],[27,4],[28,4],[21,5],[27,5],[22,6],[26,6],[23,7],[24,7],
          [35,3],[36,3],[35,4],[36,4]
        ];
        gun.forEach(([dx,dy])=>set(dx, dy));
      }
    }
  }
  // Model
  class SchellingModel {
    constructor(size, densityPct, ratioA, tolerancePct, wrapEdges) {
      this.size = size | 0;
      this.wrapEdges = !!wrapEdges;
      this.grid = new Uint8Array(this.size * this.size);
      this.tolerance = Math.min(1, Math.max(0, tolerancePct / 100));
      this.density = Math.min(1, Math.max(0, densityPct / 100));
      this.ratioA = Math.min(1, Math.max(0, ratioA / 100));
      this.stepCount = 0;
      this.randomize();
    }

    index(x, y) { return y * this.size + x; }
    inBounds(x, y) { return x >= 0 && y >= 0 && x < this.size && y < this.size; }

    randomize() {
      const n = this.size * this.size;
      const agentCells = Math.round(n * this.density);
      const aAgents = Math.round(agentCells * this.ratioA);
      const bAgents = agentCells - aAgents;

      const cells = new Array(n).fill(0);
      for (let i = 0; i < aAgents; i++) cells[i] = 1;
      for (let i = aAgents; i < aAgents + bAgents; i++) cells[i] = 2;
      shuffle(cells);
      this.grid = Uint8Array.from(cells);
      this.stepCount = 0;
    }

    neighbors(x, y) {
      let total = 0, same = 0;
      const here = this.grid[this.index(x, y)];
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          let nx = x + dx, ny = y + dy;
          if (this.wrapEdges) {
            nx = (nx + this.size) % this.size;
            ny = (ny + this.size) % this.size;
          } else if (!this.inBounds(nx, ny)) {
            continue;
          }
          const v = this.grid[this.index(nx, ny)];
          if (v !== 0) {
            total++;
            if (v === here) same++;
          }
        }
      }
      return { total, same };
    }

    // Hypothetical neighbor stats if a given value were placed at (x,y)
    neighborStatsIf(x, y, value) {
      let total = 0, same = 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          let nx = x + dx, ny = y + dy;
          if (this.wrapEdges) {
            nx = (nx + this.size) % this.size;
            ny = (ny + this.size) % this.size;
          } else if (!this.inBounds(nx, ny)) {
            continue;
          }
          const v = this.grid[this.index(nx, ny)];
          if (v !== 0) {
            total++;
            if (v === value) same++;
          }
        }
      }
      return { total, same };
    }

    wouldBeSatisfiedAt(x, y, value) {
      if (value === 0) return true;
      const { total, same } = this.neighborStatsIf(x, y, value);
      if (total === 0) return true;
      return (same / total) >= this.tolerance;
    }

    // Like wouldBeSatisfiedAt but accounts for the origin cell becoming empty when moving.
    neighborStatsIfMoving(fromX, fromY, toX, toY, value) {
      let total = 0, same = 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          let nx = toX + dx, ny = toY + dy;
          if (this.wrapEdges) {
            nx = (nx + this.size) % this.size;
            ny = (ny + this.size) % this.size;
          } else if (!this.inBounds(nx, ny)) {
            continue;
          }
          // Skip the origin cell because it becomes empty after moving
          if (nx === fromX && ny === fromY) continue;
          const v = this.grid[this.index(nx, ny)];
          if (v !== 0) {
            total++;
            if (v === value) same++;
          }
        }
      }
      return { total, same };
    }

    wouldBeSatisfiedMoving(fromX, fromY, toX, toY, value) {
      if (value === 0) return true;
      const { total, same } = this.neighborStatsIfMoving(fromX, fromY, toX, toY, value);
      if (total === 0) return true;
      return (same / total) >= this.tolerance;
    }

    isSatisfied(x, y) {
      const v = this.grid[this.index(x, y)];
      if (v === 0) return true;
      const { total, same } = this.neighbors(x, y);
      if (total === 0) return true; // no neighbors -> satisfied
      return (same / total) >= this.tolerance;
    }

    countUnsatisfied() {
      let unsat = 0, agents = 0;
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          const v = this.grid[this.index(x, y)];
          if (v !== 0) {
            agents++;
            if (!this.isSatisfied(x, y)) unsat++;
          }
        }
      }
      return { unsatisfied: unsat, agents };
    }

    step() {
      const empties = [];
      const unsatisfied = [];
      for (let y = 0; y < this.size; y++) {
        for (let x = 0; x < this.size; x++) {
          const idx = this.index(x, y);
          const v = this.grid[idx];
          if (v === 0) {
            empties.push(idx);
          } else if (!this.isSatisfied(x, y)) {
            unsatisfied.push(idx);
          }
        }
      }

      if (unsatisfied.length === 0) {
        this.stepCount++;
        return { moved: 0, unsatisfiedBefore: 0, converged: true };
      }
      if (empties.length === 0) {
        this.stepCount++;
        return { moved: 0, unsatisfiedBefore: unsatisfied.length, converged: true };
      }

      shuffle(unsatisfied);
      shuffle(empties);
      const moves = Math.min(unsatisfied.length, empties.length);
      for (let i = 0; i < moves; i++) {
        const from = unsatisfied[i];
        const to = empties[i];
        const v = this.grid[from];
        this.grid[from] = 0;
        this.grid[to] = v;
      }
      this.stepCount++;
      const { unsatisfied: uns2 } = this.countUnsatisfied();
      return { moved: moves, unsatisfiedBefore: unsatisfied.length, unsatisfiedAfter: uns2, converged: moves === 0 || uns2 === 0 };
    }
  }

  // Renderer
  const ctx = els.canvas.getContext('2d', { alpha: false });
  let model = null;
  let currentSim = 'schelling';
  // Per-simulation defaults/state for Wrap edges
  let simWrap = { schelling: false, life: true, boids: true, fireflies: true };
  // Remember per-sim density (occupancy) preferences; defaults: Schelling 90, Life 50
  let simDensity = { schelling: 90, life: 50, boids: 50, fireflies: 80 };
  // Remember per-sim step delay (ms); default Boids=50, others=100
  let simDelay = { schelling: 100, life: 100, boids: 50, fireflies: 50 };
  let cellSize = 10; // pixels
  let view = { offsetX: 0, offsetY: 0, width: 0, height: 0 };
  let lastMove = null; // { from:{x,y}, to:{x,y}, time:number }

  function fitCanvasToDisplay() {
    const dpr = Math.max(1, Math.floor(window.devicePixelRatio || 1));
    const rect = els.canvas.getBoundingClientRect();
    const w = Math.max(200, Math.floor(rect.width));
    const h = Math.max(200, Math.floor(rect.height));
    els.canvas.width = w * dpr;
    els.canvas.height = h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function drawGrid() {
    if (!model) return;
    const rect = els.canvas.getBoundingClientRect();
    const w = rect.width, h = rect.height;

    // Boids: draw in pixel space and return
    if (currentSim === 'boids') {
      // Trails fade or clear
      const trailsOn = els.boidsTrails && els.boidsTrails.checked;
      if (trailsOn) {
        const persistence = Math.max(0, Math.min(100, parseInt(els.trailPersistence.value || '80', 10))) / 100;
        const fadeAlpha = Math.max(0.02, 1 - persistence);
        ctx.fillStyle = '#0c0f14';
        ctx.globalAlpha = fadeAlpha * 0.35;
        ctx.fillRect(0, 0, w, h);
        ctx.globalAlpha = 1;
      } else {
        ctx.fillStyle = '#0c0f14';
        ctx.fillRect(0, 0, w, h);
      }
      view = { offsetX: 0, offsetY: 0, width: w, height: h };
      // Obstacles
      if (model.obstacles && model.obstacles.length) {
        ctx.strokeStyle = 'rgba(236, 72, 153, 0.85)';
        ctx.fillStyle = 'rgba(236, 72, 153, 0.08)';
        for (const o of model.obstacles) {
          ctx.beginPath(); ctx.arc(o.x, o.y, o.r, 0, Math.PI*2);
          ctx.fill(); ctx.stroke();
        }
      }
      for (const boid of model.boids) {
        const angle = Math.atan2(boid.vy, boid.vx);
        const sizePx = 6 + Math.min(10, Math.hypot(boid.vx, boid.vy));
        ctx.fillStyle = '#f472b6';
        ctx.beginPath();
        ctx.moveTo(boid.x + Math.cos(angle) * sizePx, boid.y + Math.sin(angle) * sizePx);
        ctx.lineTo(boid.x + Math.cos(angle + 2.5) * sizePx * 0.6, boid.y + Math.sin(angle + 2.5) * sizePx * 0.6);
        ctx.lineTo(boid.x + Math.cos(angle - 2.5) * sizePx * 0.6, boid.y + Math.sin(angle - 2.5) * sizePx * 0.6);
        ctx.closePath();
        ctx.fill();
      }
      return;
    }

    // Clear background for grid-based sims
    ctx.fillStyle = '#0c0f14';
    ctx.fillRect(0, 0, w, h);

    const size = model.size;
    const margin = 8;
    const usable = Math.min(w, h) - margin * 2;
    cellSize = Math.max(2, Math.floor(usable / size));
    const gridW = cellSize * size;
    const offsetX = Math.floor((w - gridW) / 2);
    const offsetY = Math.floor((h - gridW) / 2);
    view = { offsetX, offsetY, width: w, height: h };

    // Draw cells
    if (currentSim === 'schelling') {
      for (let y = 0; y < size; y++) {
        for (let x = 0; x < size; x++) {
          const v = model.grid[model.index(x, y)];
          let color;
          if (v === 1) color = '#2d88ff';
          else if (v === 2) color = '#ff4757';
          else color = '#2a2f3d';
          ctx.fillStyle = color;
          ctx.fillRect(offsetX + x * cellSize, offsetY + y * cellSize, cellSize, cellSize);
        }
      }
    } else if (currentSim === 'life') {
      for (let y = 0; y < size; y++) {
        for (let x = 0; x < size; x++) {
          const alive = model.grid[model.index(x, y)] === 1;
          ctx.fillStyle = alive ? '#7ee7c4' : '#1a1f2c';
          ctx.fillRect(offsetX + x * cellSize, offsetY + y * cellSize, cellSize, cellSize);
        }
      }
    } else if (currentSim === 'fireflies') {
      for (let y=0; y<size; y++){
        for (let x=0; x<size; x++){
          const idx = model.index(x,y);
          const present = model.grid[idx] === 1;
          if (!present) { ctx.fillStyle = '#1a1f2c'; }
          else {
            const flashing = model.flash[idx] > 0;
            if (flashing) ctx.fillStyle = '#fde047';
            else {
              // dim yellow based on phase (brighter near flash)
              const phase = model.theta[idx] / (Math.PI*2); // 0..1
              const intensity = 0.25 + 0.5 * (1 - Math.cos(phase * Math.PI*2)) * 0.5;
              const base = 0x47; // yellow-ish low
              const r = Math.min(255, Math.floor(253 * intensity));
              const g = Math.min(255, Math.floor(224 * intensity));
              const b = Math.min(255, Math.floor(71 * intensity));
              ctx.fillStyle = `rgb(${r},${g},${b})`;
            }
          }
          ctx.fillRect(offsetX + x*cellSize, offsetY + y*cellSize, cellSize, cellSize);
        }
      }
    }

    // Grid overlay (subtle)
    if (cellSize >= 6) {
      ctx.strokeStyle = 'rgba(255,255,255,0.05)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i <= size; i++) {
        const px = offsetX + i * cellSize + 0.5;
        ctx.moveTo(px, offsetY + 0.5);
        ctx.lineTo(px, offsetY + size * cellSize + 0.5);
        const py = offsetY + i * cellSize + 0.5;
        ctx.moveTo(offsetX + 0.5, py);
        ctx.lineTo(offsetX + size * cellSize + 0.5, py);
      }
      ctx.stroke();
    }

    // Unsatisfied markers (optional)
    if (currentSim === 'schelling' && els.showUnsatisfied && els.showUnsatisfied.checked) {
      const stroke = 'rgba(255, 209, 102, 0.9)'; // warm yellow
      ctx.lineWidth = Math.max(1, Math.floor(cellSize / 6));
      ctx.strokeStyle = stroke;
      const inset = Math.max(1.5, Math.floor(cellSize * 0.18));
      for (let y = 0; y < size; y++) {
        for (let x = 0; x < size; x++) {
          const v = model.grid[model.index(x, y)];
          if (v === 0) continue;
          if (!model.isSatisfied(x, y)) {
            const px = offsetX + x * cellSize;
            const py = offsetY + y * cellSize;
            // small inner rectangle marker
            ctx.strokeRect(px + inset, py + inset, cellSize - inset * 2, cellSize - inset * 2);
          }
        }
      }
    }

    // Draw last move markers (fade out): elegant curved neon-green arrow with glow
    if (currentSim === 'schelling' && lastMove) {
      const age = Date.now() - lastMove.time;
      const ttl = 2200;
      if (age < ttl) {
        const alpha = Math.max(0, 1 - age / ttl);
        const from = lastMove.from; const to = lastMove.to;
        const sx = offsetX + from.x * cellSize + cellSize/2;
        const sy = offsetY + from.y * cellSize + cellSize/2;
        const tx = offsetX + to.x * cellSize + cellSize/2;
        const ty = offsetY + to.y * cellSize + cellSize/2;
        const dx = tx - sx; const dy = ty - sy;
        const len = Math.hypot(dx, dy) || 1;
        const ux = dx / len; const uy = dy / len;
        const nx = -uy; const ny = ux; // unit normal

        // Sizing, scaled to cell size only (not distance)
        const lineWidthOuter = Math.max(2, Math.floor(cellSize * 0.22));
        const lineWidthInner = Math.max(1, Math.floor(lineWidthOuter * 0.5));
        const circleOuter = Math.max(4, Math.floor(cellSize * 0.35));
        const circleInner = Math.max(2, Math.floor(circleOuter * 0.55));

        // Shaft endpoints: straight line, shortened to avoid overlap with start circle and fixed-size X
        const xHalf = Math.max(5, Math.floor(cellSize * 0.4));
        const shaftStartX = sx + ux * (circleOuter + 1);
        const shaftStartY = sy + uy * (circleOuter + 1);
        const shaftEndX = tx - ux * (xHalf + 2);
        const shaftEndY = ty - uy * (xHalf + 2);

        ctx.save();
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';

        // Outer glow stroke
        ctx.shadowColor = `rgba(0, 255, 170, ${0.7 * alpha})`;
        ctx.shadowBlur = Math.max(8, Math.floor(cellSize * 0.8));
        ctx.strokeStyle = `rgba(0, 230, 150, ${0.55 * alpha})`;
        ctx.lineWidth = lineWidthOuter;
        ctx.beginPath();
        ctx.moveTo(shaftStartX, shaftStartY);
        ctx.lineTo(shaftEndX, shaftEndY);
        ctx.stroke();

        // Inner bright stroke
        ctx.shadowBlur = Math.max(4, Math.floor(cellSize * 0.4));
        ctx.strokeStyle = `rgba(6, 214, 160, ${0.95 * alpha})`;
        ctx.lineWidth = lineWidthInner;
        ctx.beginPath();
        ctx.moveTo(shaftStartX, shaftStartY);
        ctx.lineTo(shaftEndX, shaftEndY);
        ctx.stroke();

        // Start marker: ring + dot with glow
        ctx.fillStyle = `rgba(6, 214, 160, ${0.95 * alpha})`;
        ctx.beginPath(); ctx.arc(sx, sy, circleOuter, 0, Math.PI * 2); ctx.fill();
        ctx.globalCompositeOperation = 'destination-out';
        ctx.beginPath(); ctx.arc(sx, sy, circleOuter - Math.max(2, Math.floor(circleOuter * 0.35)), 0, Math.PI * 2); ctx.fill();
        ctx.globalCompositeOperation = 'source-over';
        ctx.shadowBlur = Math.max(6, Math.floor(cellSize * 0.6));
        ctx.beginPath(); ctx.arc(sx, sy, circleInner, 0, Math.PI * 2); ctx.fill();

        // End marker: glowing 'X' at the destination (fixed orientation and size)
        const outerHead = Math.max(2, Math.floor(cellSize * 0.28));
        const innerHead = Math.max(1, Math.floor(outerHead * 0.6));

        // Outer glow for the X
        ctx.shadowBlur = Math.max(8, Math.floor(cellSize * 0.8));
        ctx.strokeStyle = `rgba(6, 214, 160, ${0.75 * alpha})`;
        ctx.lineWidth = outerHead;
        ctx.beginPath();
        ctx.moveTo(tx - xHalf, ty - xHalf);
        ctx.lineTo(tx + xHalf, ty + xHalf);
        ctx.moveTo(tx - xHalf, ty + xHalf);
        ctx.lineTo(tx + xHalf, ty - xHalf);
        ctx.stroke();

        // Inner crisp X
        ctx.shadowBlur = Math.max(4, Math.floor(cellSize * 0.4));
        ctx.strokeStyle = `rgba(6, 214, 160, ${0.98 * alpha})`;
        ctx.lineWidth = innerHead;
        ctx.beginPath();
        ctx.moveTo(tx - xHalf, ty - xHalf);
        ctx.lineTo(tx + xHalf, ty + xHalf);
        ctx.moveTo(tx - xHalf, ty + xHalf);
        ctx.lineTo(tx + xHalf, ty - xHalf);
        ctx.stroke();

        ctx.restore();
      } else {
        lastMove = null;
      }
    }
  }

  function updateStats(extra = {}) {
    if (!model) return;
    els.statStep.textContent = String(model.stepCount || 0);
    if (currentSim === 'schelling') {
      const { unsatisfied, agents } = model.countUnsatisfied();
      els.statAgents.textContent = String(agents);
      els.statUnsatisfied.textContent = String(unsatisfied);
      const satisfiedPct = agents === 0 ? 100 : Math.round(((agents - unsatisfied) / agents) * 100);
      els.statSatisfied.textContent = satisfiedPct + '%';
    } else if (currentSim === 'life') {
      const alive = model.countAlive();
      els.statAgents.textContent = String(alive);
      els.statUnsatisfied.textContent = '-';
      els.statSatisfied.textContent = '-';
    } else if (currentSim === 'boids') {
      const count = model.count();
      els.statAgents.textContent = String(count);
      els.statUnsatisfied.textContent = '-';
      els.statSatisfied.textContent = '-';
    } else if (currentSim === 'fireflies') {
      const agents = model.countAgents();
      els.statAgents.textContent = String(agents);
      els.statUnsatisfied.textContent = '-';
      els.statSatisfied.textContent = '-';
    }
    if (typeof extra.moved === 'number') {
      els.statMoved.textContent = String(extra.moved);
    } else if (typeof extra.flashed === 'number') {
      els.statMoved.textContent = String(extra.flashed);
    }
  }

  // Controls wiring
  function syncLabels() {
    els.gridSizeValue.textContent = els.gridSize.value;
    els.gridSizeValue2.textContent = els.gridSize.value;
    els.densityValue.textContent = els.density.value;
    els.ratioAValue.textContent = els.ratioA.value;
    els.toleranceValue.textContent = els.tolerance.value;
    els.delayValue.textContent = els.delay.value;
    if (els.ffRadiusValue) els.ffRadiusValue.textContent = els.ffRadius.value;
    if (els.ffCouplingValue) els.ffCouplingValue.textContent = parseFloat(els.ffCoupling.value||'0').toFixed(2);
    if (els.ffPeriodValue) els.ffPeriodValue.textContent = els.ffPeriod.value;
    if (els.ffJitterValue) els.ffJitterValue.textContent = els.ffJitter.value;
    if (els.boidsCountValue) els.boidsCountValue.textContent = els.boidsCount.value;
    if (els.perceptionValue) els.perceptionValue.textContent = els.perception.value;
    if (els.separationDistValue) els.separationDistValue.textContent = els.separationDist.value;
    if (els.maxSpeedValue) els.maxSpeedValue.textContent = els.maxSpeed.value;
    if (els.maxForceValue) els.maxForceValue.textContent = els.maxForce.value;
    if (els.weightSepValue) els.weightSepValue.textContent = els.weightSep.value;
    if (els.weightAlignValue) els.weightAlignValue.textContent = els.weightAlign.value;
    if (els.weightCohValue) els.weightCohValue.textContent = els.weightCoh.value;
    if (els.trailPersistenceValue) els.trailPersistenceValue.textContent = els.trailPersistence.value;
    if (els.mouseForceValue) els.mouseForceValue.textContent = parseFloat(els.mouseForce.value || '0').toFixed(1);
    if (els.wAvoidValue) els.wAvoidValue.textContent = els.wAvoid.value;
  }

  [els.gridSize, els.density, els.ratioA, els.tolerance, els.delay].forEach(inp => {
    inp.addEventListener('input', syncLabels);
  });
  [els.boidsCount, els.perception, els.separationDist, els.maxSpeed, els.maxForce, els.weightSep, els.weightAlign, els.weightCoh, els.trailPersistence, els.mouseForce, els.wAvoid, els.ffRadius, els.ffCoupling, els.ffPeriod, els.ffJitter].forEach(inp => {
    if (!inp) return;
    inp.addEventListener('input', syncLabels);
  });

  function initModelFromControls() {
    const size = parseInt(els.gridSize.value, 10);
    const density = parseInt(els.density.value, 10);
    const wrap = !!els.wrapEdges.checked;
    if (currentSim === 'schelling') {
      const ratioA = parseInt(els.ratioA.value, 10);
      const tolerance = parseInt(els.tolerance.value, 10);
      model = new SchellingModel(size, density, ratioA, tolerance, wrap);
    } else if (currentSim === 'life') {
      model = new LifeModel(size, density, wrap);
      const pattern = els.lifePattern ? els.lifePattern.value : 'random';
      if (pattern && pattern !== 'random') model.seedPattern(pattern);
    } else if (currentSim === 'boids') {
      fitCanvasToDisplay();
      const rect = els.canvas.getBoundingClientRect();
      const opts = {
        count: parseInt(els.boidsCount.value, 10) || 150,
        width: rect.width,
        height: rect.height,
        perception: parseFloat(els.perception.value) || 50,
        separationDist: parseFloat(els.separationDist.value) || 20,
        maxSpeed: parseFloat(els.maxSpeed.value) || 2.5,
        maxForce: parseFloat(els.maxForce.value) || 0.05,
        wSep: parseFloat(els.weightSep.value) || 1.5,
        wAlign: parseFloat(els.weightAlign.value) || 1.0,
        wCoh: parseFloat(els.weightCoh.value) || 1.0,
        wAvoid: parseFloat(els.wAvoid.value) || 1.2,
        mouseForce: parseFloat(els.mouseForce.value) || 0,
        wrapEdges: wrap,
      };
      model = new BoidsModel(opts);
    } else if (currentSim === 'fireflies') {
      const radiusParsed = parseInt(els.ffRadius.value, 10);
      const radius = Number.isFinite(radiusParsed) ? radiusParsed : 3;
      const couplingParsed = parseFloat(els.ffCoupling.value);
      const coupling = Number.isFinite(couplingParsed) ? couplingParsed : 0.2; // allow 0 exactly
      const basePeriodParsed = parseInt(els.ffPeriod.value, 10);
      const basePeriod = Number.isFinite(basePeriodParsed) ? basePeriodParsed : 60;
      const jitterParsed = parseInt(els.ffJitter.value, 10);
      const jitter = Number.isFinite(jitterParsed) ? jitterParsed : 20;
      model = new FirefliesModel(size, density, wrap, radius, coupling, basePeriod, jitter);
    }
    fitCanvasToDisplay();
    drawGrid();
    updateStats({ moved: 0 });
  }

  // Debounced auto-initialize when key sliders change
  let autoInitTimer = null;
  function autoInit() {
    if (running) { running = false; if (els.runBtnLabel) els.runBtnLabel.textContent = 'Run'; els.runBtn.classList.remove('running'); }
    initModelFromControls();
  }
  function debouncedAutoInit() {
    if (autoInitTimer) clearTimeout(autoInitTimer);
    autoInitTimer = setTimeout(autoInit, 150);
  }

  // Auto re-init on structural parameters: grid size, occupancy, type split
  [els.gridSize, els.density, els.ratioA].forEach(inp => {
    inp.addEventListener('input', debouncedAutoInit);
    inp.addEventListener('change', autoInit);
  });
  // Auto re-init for boids controls
  ;[els.boidsCount, els.perception, els.separationDist, els.maxSpeed, els.maxForce, els.weightSep, els.weightAlign, els.weightCoh, els.trailPersistence, els.mouseForce, els.wAvoid, els.ffRadius, els.ffCoupling, els.ffPeriod, els.ffJitter].forEach(inp => {
    if (!inp) return;
    inp.addEventListener('input', debouncedAutoInit);
    inp.addEventListener('change', autoInit);
  });

  let running = false;
  let runPromise = null;

  async function runUntilConvergence() {
    if (!model || running) return;
    running = true;
    if (els.runBtnLabel) els.runBtnLabel.textContent = 'Pause';
    els.runBtn.classList.add('running');
    try {
      const maxSteps = Math.max(1, parseInt(els.maxIterations.value, 10) || 5000);
      const delay = Math.max(0, parseInt(els.delay.value, 10) || 0);
      for (let i = 0; i < maxSteps; i++) {
        if (!running) break;
        const stats = model.step();
        drawGrid();
        updateStats(stats);
        if (stats.converged) break;
        if (delay > 0) await new Promise(r => setTimeout(r, delay));
      }
    } finally {
      running = false;
      if (els.runBtnLabel) els.runBtnLabel.textContent = 'Run';
      els.runBtn.classList.remove('running');
    }
  }

  function stepOnce() {
    if (!model) return;
    const stats = model.step();
    drawGrid();
    updateStats(stats);
  }

  // Manual move: click a cell to move if unsatisfied
  function coordsToCell(clientX, clientY) {
    const rect = els.canvas.getBoundingClientRect();
    const cx = clientX - rect.left;
    const cy = clientY - rect.top;
    const x = Math.floor((cx - view.offsetX) / cellSize);
    const y = Math.floor((cy - view.offsetY) / cellSize);
    if (!model) return null;
    if (x < 0 || y < 0 || x >= model.size || y >= model.size) return null;
    return { x, y };
  }

  function moveOneIfUnsatisfied(x, y) {
    if (!model) return { moved: false };
    if (model.grid[model.index(x, y)] === 0) return { moved: false };
    if (model.isSatisfied(x, y)) return { moved: false, satisfied: true };
    // collect empties
    const empties = [];
    const n = model.size * model.size;
    for (let i = 0; i < n; i++) if (model.grid[i] === 0) empties.push(i);
    if (empties.length === 0) return { moved: false };

    const fromIdx = model.index(x, y);
    const v = model.grid[fromIdx];

    // distance function (Manhattan with wrap if enabled)
    function manhattanToroidal(x1, y1, x2, y2, size, wrap) {
      let dx = Math.abs(x2 - x1);
      let dy = Math.abs(y2 - y1);
      if (wrap) {
        dx = Math.min(dx, size - dx);
        dy = Math.min(dy, size - dy);
      }
      return dx + dy;
    }

    // find nearest empty that would satisfy
    let best = null;
    for (const idx of empties) {
      const tx = idx % model.size;
      const ty = Math.floor(idx / model.size);
      if (model.wouldBeSatisfiedMoving(x, y, tx, ty, v)) {
        const dist = manhattanToroidal(x, y, tx, ty, model.size, model.wrapEdges);
        if (!best || dist < best.dist) {
          best = { idx, x: tx, y: ty, dist };
        }
      }
    }

    let toIdx;
    if (best) {
      toIdx = best.idx;
    } else {
      // fallback: random empty
      toIdx = empties[Math.floor(Math.random() * empties.length)];
    }

    model.grid[fromIdx] = 0;
    model.grid[toIdx] = v;
    const to = { x: toIdx % model.size, y: Math.floor(toIdx / model.size) };
    return { moved: true, from: { x, y }, to };
  }

  // Event listeners
  els.initializeBtn.addEventListener('click', (e) => {
    e.preventDefault();
    if (running) {
      running = false;
      if (els.runBtnLabel) els.runBtnLabel.textContent = 'Run';
      if (els.runBtn) els.runBtn.classList.remove('running');
    }
    initModelFromControls();
  });
  els.stepBtn.addEventListener('click', (e) => {
    e.preventDefault();
    stepOnce();
  });
  els.runBtn.addEventListener('click', async (e) => {
    e.preventDefault();
    if (running) { running = false; if (els.runBtnLabel) els.runBtnLabel.textContent = 'Run'; els.runBtn.classList.remove('running'); return; }
    await runUntilConvergence();
  });
  // Removed Fit View button; canvas auto-fits on init and resize

  if (els.showUnsatisfied) {
    els.showUnsatisfied.addEventListener('change', () => {
      drawGrid();
    });
  }

  // Simulation tabs and pattern
  function setSim(sim) {
    // Helper: update URL's sim query param without reloading
    function updateURLSim(val) {
      try {
        const url = new URL(window.location.href);
        url.searchParams.set('sim', val);
        history.replaceState(null, '', url.toString());
      } catch (_) {}
    }
    // If switching to the same sim and no model yet (e.g., first load), ensure init
    if (sim === currentSim) {
      if (!model) {
        // Ensure classes/labels are in a consistent state then init
        document.body.classList.toggle('sim-schelling', sim === 'schelling');
        document.body.classList.toggle('sim-life', sim === 'life');
        document.body.classList.toggle('sim-boids', sim === 'boids');
        if (typeof updateSimTitle === 'function') updateSimTitle();
        autoInit();
      }
      // Even if already current, reflect in URL (e.g., user clicks active tab)
      updateURLSim(sim);
      return;
    }
    // Store current density and wrap under currentSim
    if (els.density) {
      const curVal = parseInt(els.density.value, 10);
      if (!Number.isNaN(curVal)) simDensity[currentSim] = curVal;
    }
    if (els.wrapEdges) {
      simWrap[currentSim] = !!els.wrapEdges.checked;
    }
    // Store current delay
    if (els.delay) {
      const curDelay = parseInt(els.delay.value, 10);
      if (!Number.isNaN(curDelay)) simDelay[currentSim] = curDelay;
    }
    currentSim = sim;
    // Update body class for robust CSS-based toggling
    document.body.classList.toggle('sim-schelling', sim === 'schelling');
    document.body.classList.toggle('sim-life', sim === 'life');
    document.body.classList.toggle('sim-boids', sim === 'boids');
    document.body.classList.toggle('sim-fireflies', sim === 'fireflies');
    // Apply target sim default/last density
    if (els.density) {
      const next = simDensity[sim] ?? (sim === 'life' ? 50 : 90);
      els.density.value = String(next);
      if (els.densityValue) els.densityValue.textContent = String(next);
    }
    // Apply target sim default/last wrap
    if (els.wrapEdges) {
      const nextWrap = simWrap[sim];
      els.wrapEdges.checked = (typeof nextWrap === 'boolean') ? nextWrap : (sim === 'life');
    }
    // Apply target sim default/last delay
    if (els.delay) {
      const nextDelay = simDelay[sim] ?? 100;
      els.delay.value = String(nextDelay);
      if (els.delayValue) els.delayValue.textContent = String(nextDelay);
    }
    if (els.tabSchelling && els.tabLife) {
      els.tabSchelling.classList.toggle('active', sim === 'schelling');
      els.tabLife.classList.toggle('active', sim === 'life');
    }
    if (els.tabBoids) {
      els.tabBoids.classList.toggle('active', sim === 'boids');
    }
    if (els.tabFireflies) {
      els.tabFireflies.classList.toggle('active', sim === 'fireflies');
    }
    if (els.schellingPanel) els.schellingPanel.hidden = sim !== 'schelling';
    if (els.lifePanel) els.lifePanel.hidden = sim !== 'life';
    if (els.boidsPanel) els.boidsPanel.hidden = sim !== 'boids';
    if (els.firefliesPanel) els.firefliesPanel.hidden = sim !== 'fireflies';
    if (els.unsatRow) els.unsatRow.style.display = (sim === 'schelling') ? '' : 'none';
    // Toggle common grid controls visibility (not used by Boids)
    const gridSizeGroup = document.getElementById('gridSizeGroup');
    const densityGroup = document.getElementById('densityGroup');
    const showGridControls = (sim !== 'boids');
    if (gridSizeGroup) gridSizeGroup.style.display = showGridControls ? '' : 'none';
    if (densityGroup) densityGroup.style.display = showGridControls ? '' : 'none';
    if (els.schellingRules) els.schellingRules.hidden = sim !== 'schelling';
    if (els.lifeRules) els.lifeRules.hidden = sim !== 'life';
    if (els.boidsRules) els.boidsRules.hidden = sim !== 'boids';
    const fr = document.getElementById('firefliesRules'); if (fr) fr.hidden = sim !== 'fireflies';
    if (els.schellingLegend) els.schellingLegend.hidden = sim !== 'schelling';
    if (els.lifeLegend) els.lifeLegend.hidden = sim !== 'life';
    if (els.boidsLegend) els.boidsLegend.hidden = sim !== 'boids';
    if (els.firefliesLegend) els.firefliesLegend.hidden = sim !== 'fireflies';
    // Toggle Advanced per-sim blocks
    const sa = document.getElementById('schellingAdvanced');
    const la = document.getElementById('lifeAdvanced');
    const ba = document.getElementById('boidsAdvanced');
    const fa = document.getElementById('firefliesAdvanced');
    if (sa) sa.hidden = sim !== 'schelling';
    if (la) la.hidden = sim !== 'life';
    if (ba) ba.hidden = sim !== 'boids';
    if (fa) fa.hidden = sim !== 'fireflies';
    // Update the per-simulation title link
    updateSimTitle();
    // Update rules Wikipedia link
    const rulesWiki = document.getElementById('rulesWiki');
    if (rulesWiki) {
      if (sim === 'schelling') rulesWiki.href = 'https://en.wikipedia.org/wiki/Schelling%27s_model_of_segregation';
      else if (sim === 'life') rulesWiki.href = 'https://en.wikipedia.org/wiki/Conway%27s_Game_of_Life';
      else if (sim === 'boids') rulesWiki.href = 'https://en.wikipedia.org/wiki/Boids';
      else if (sim === 'fireflies') rulesWiki.href = 'https://en.wikipedia.org/wiki/Firefly_synchronization';
    }
    autoInit();
    // Reflect new selection in URL
    updateURLSim(sim);
  }
  if (els.tabSchelling && els.tabLife) {
    els.tabSchelling.addEventListener('click', ()=>setSim('schelling'));
    els.tabLife.addEventListener('click', ()=>setSim('life'));
  }
  if (els.tabBoids) {
    els.tabBoids.addEventListener('click', ()=>setSim('boids'));
  }
  if (els.tabFireflies) {
    els.tabFireflies.addEventListener('click', ()=>setSim('fireflies'));
  }
  if (els.lifePattern) {
    els.lifePattern.addEventListener('change', () => {
      if (currentSim === 'life') initModelFromControls();
    });
  }
  // Keyboard shortcuts for Edit tools
  document.addEventListener('keydown', (e) => {
    // Ignore when typing in inputs/selects/textarea
    const tag = (e.target && e.target.tagName) || '';
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    const k = (e.key || '').toLowerCase();
    if (currentSim === 'schelling') {
      if (!els.schellingEditEnabled) return;
      if (k === 'a' || k === 'b' || k === 'e') { els.schellingEditEnabled.checked = true; e.preventDefault(); }
    } else if (currentSim === 'life') {
      if (!els.lifeEditEnabled) return;
      if (k === 'a' || k === 'd' || k === 'e') { els.lifeEditEnabled.checked = true; e.preventDefault(); }
    }
  });
  if (els.lifeClearBtn) {
    els.lifeClearBtn.addEventListener('click', (e) => {
      e.preventDefault();
      if (currentSim !== 'life' || !model || !model.grid) return;
      model.grid.fill(0);
      drawGrid();
      updateStats({});
    });
  }

  if (els.schellingClearBtn) {
    els.schellingClearBtn.addEventListener('click', (e) => {
      e.preventDefault();
      if (currentSim !== 'schelling' || !model || !model.grid) return;
      model.grid.fill(0);
      drawGrid();
      updateStats({});
    });
  }

  // Sim title link updater
  function updateSimTitle() {
    if (!els.simTitle) return;
    els.simTitle.classList.remove('schelling','life','boids','fireflies');
    if (currentSim === 'schelling') {
      els.simTitle.textContent = "Schelling's Model of Segregation";
      els.simTitle.href = 'https://en.wikipedia.org/wiki/Schelling%27s_model_of_segregation';
      els.simTitle.classList.add('schelling');
    } else if (currentSim === 'life') {
      els.simTitle.textContent = "Conway's Game of Life";
      els.simTitle.href = 'https://en.wikipedia.org/wiki/Conway%27s_Game_of_Life';
      els.simTitle.classList.add('life');
    } else if (currentSim === 'boids') {
      els.simTitle.textContent = "Reynolds' Boids";
      els.simTitle.href = 'https://en.wikipedia.org/wiki/Boids';
      els.simTitle.classList.add('boids');
    } else if (currentSim === 'fireflies') {
      els.simTitle.textContent = "Fireflies Synchronization";
      els.simTitle.href = 'https://en.wikipedia.org/wiki/Firefly_synchronization';
      els.simTitle.classList.add('fireflies');
    }
  }
  // End sim switching


  els.canvas.addEventListener('click', (e) => {
    if (didDrag) { didDrag = false; const b=document.getElementById('paintBadge'); if (b) b.hidden=true; return; }
    const cell = coordsToCell(e.clientX, e.clientY);
    if (currentSim === 'schelling') {
      if (!cell) return;
      if (els.schellingEditEnabled && els.schellingEditEnabled.checked) {
        const idx = model.index(cell.x, cell.y);
        const v = model.grid[idx];
        model.grid[idx] = v === 0 ? 1 : (v === 1 ? 2 : 0);
        drawGrid();
        updateStats({});
        const b=document.getElementById('paintBadge'); if (b) b.hidden=true;
      } else {
        const res = moveOneIfUnsatisfied(cell.x, cell.y);
        if (res && res.moved) {
          lastMove = { from: res.from, to: res.to, time: Date.now() };
          drawGrid();
          updateStats({ moved: 1 });
        }
        const b=document.getElementById('paintBadge'); if (b) b.hidden=true;
      }
    } else if (currentSim === 'life') {
      if (!cell) return;
      const idx = model.index(cell.x, cell.y);
      // Toggle always in Life
      model.grid[idx] = model.grid[idx] ? 0 : 1;
      drawGrid();
      updateStats({});
      const b=document.getElementById('paintBadge'); if (b) b.hidden=true;
    } else if (currentSim === 'boids') {
      const rect = els.canvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      if (e.shiftKey) model.addObstacle(x, y, 24);
      else if (e.altKey) model.removeNearestObstacle(x, y);
      else model.boids.push({ x, y, vx: (Math.random()*2-1)*model.maxSpeed, vy: (Math.random()*2-1)*model.maxSpeed });
      drawGrid();
    }
  });
  // Mouse position for boids
  els.canvas.addEventListener('mousemove', (e) => {
    if (dragging && (currentSim === 'schelling' || currentSim === 'life')) {
      const cell = coordsToCell(e.clientX, e.clientY);
      if (cell) {
        const idx = (currentSim === 'schelling') ? model.index(cell.x, cell.y) : model.index(cell.x, cell.y);
        if (!paintedSet.has(idx)) {
          if (applyEditAtCell(cell.x, cell.y)) {
            paintedSet.add(idx); didDrag = true; drawGrid(); updateStats({});
          }
        }
      }
    }

    const rect = els.canvas.getBoundingClientRect();
    mousePos.x = e.clientX - rect.left;
    mousePos.y = e.clientY - rect.top;
    mousePos.inside = true;
    const badge=document.getElementById('paintBadge');
    if (badge && dragging && paintTarget!==null) { badge.style.left=mousePos.x+'px'; badge.style.top=mousePos.y+'px'; }
  });
  els.canvas.addEventListener('mouseleave', () => { mousePos.inside = false; dragging=false; paintTarget=null; paintedSet.clear(); const b=document.getElementById('paintBadge'); if(b) b.hidden=true; });

  els.canvas.addEventListener('mousedown', (e) => {
    if (currentSim === 'schelling' && els.schellingEditEnabled && els.schellingEditEnabled.checked) {
      dragging = true; didDrag = false; paintedSet.clear();
      const cell = coordsToCell(e.clientX, e.clientY);
      if (cell) {
        const idx = model.index(cell.x, cell.y);
        const v = model.grid[idx]; paintTarget = (currentSim==='schelling') ? (v===0?1:(v===1?2:0)) : (v?0:1); const badge=document.getElementById('paintBadge'); if (badge) { badge.textContent = currentSim==='schelling' ? (paintTarget===1?'A':(paintTarget===2?'B':'Empty')) : (paintTarget===1?'Alive':'Dead'); badge.className = 'paint-badge ' + (currentSim==='schelling' ? (paintTarget===1?'paint-a':(paintTarget===2?'paint-b':'paint-empty')) : (paintTarget===1?'paint-alive':'paint-dead')); badge.hidden=false; } if (!paintedSet.has(idx) && applyEditAtCell(cell.x, cell.y)) {
          paintedSet.add(idx); drawGrid(); updateStats({});
        }
      }
    } else if (currentSim === 'life') {
      dragging = true; didDrag = false; paintedSet.clear();
      const cell = coordsToCell(e.clientX, e.clientY);
      if (cell) {
        const idx = model.index(cell.x, cell.y);
        const v = model.grid[idx]; paintTarget = (v?0:1); const badge=document.getElementById('paintBadge'); if (badge) { badge.textContent = (paintTarget===1?'Alive':'Dead'); badge.hidden=false; } if (!paintedSet.has(idx) && applyEditAtCell(cell.x, cell.y)) {
          paintedSet.add(idx); drawGrid(); updateStats({});
        }
      }
    } else { dragging=false; paintedSet.clear(); }
  });
  window.addEventListener('mouseup', () => { if (dragging) { dragging=false; paintTarget=null; const b=document.getElementById('paintBadge'); if(b) b.hidden=true; if (paintedSet.size>0) didDrag=true; paintedSet.clear(); } });


  window.addEventListener('resize', () => {
    fitCanvasToDisplay();
    drawGrid();
  });

  // Boot
  syncLabels();
  // Initialize selected simulation from URL (?sim=schelling|life|boids)
  (function(){
    let initial = 'schelling';
    try {
      const url = new URL(window.location.href);
      const s = (url.searchParams.get('sim') || '').toLowerCase();
      if (s === 'schelling' || s === 'life' || s === 'boids' || s === 'fireflies') initial = s;
    } catch(_) {}
    setSim(initial);
  })();
  // If initial sim equals default and setSim returned early, ensure a model exists
  if (!model) autoInit();
  fitCanvasToDisplay();
  // Ensure sim title reflects current selection
  if (typeof updateSimTitle === 'function') updateSimTitle();
  drawGrid();
})();

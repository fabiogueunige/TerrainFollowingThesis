# State Machine Improvements Documentation

## Overview
Sistema di gestione migliorato per la perdita di sensori e recovery automatico, con timeout, grace period e diagnostica avanzata.

---

## Flusso di Esecuzione (main_6DOF_3D.m)

### Loop Principale (ogni iterazione k):
```
1. state_machine(state(k-1), cmd, k)
   └─> Determina next_state basato su cmd flags (dal k-1)

2. goal_def(state(k), rob_rot, x_est, k)
   └─> Definisce goal (velocità, angoli) per lo stato corrente

3. input_control(...) → PID computation
4. dynamic_model(...) → Robot dynamics
5. AHRS_measurement(...) → Orientation update
6. DVL_measurement(...) → Position update
7. terrain_generator(...) → Dynamic terrain
8. SBES_measurement(...) → Sensor measurements
   └─> Aggiorna cmd.contact(1:4) e cmd.sensor_fail

9. EKF prediction & update
10. goal_controller(cmd, x_est, rob_rot, goal, N, state(k), k, d_dim)
    └─> Aggiorna cmd con nuovi flags diagnostici per k+1:
        - cmd.pitch_sensors_lost
        - cmd.roll_sensors_lost
        - cmd.diagonal_sensors_lost
        - cmd.sensor_fail_persistent
        - cmd.recovery_timeout
        - cmd.recovery_progress
```

### Ordine Temporale:
- **k**: `state_machine` usa `cmd` flags da iterazione precedente (k-1)
- **k**: `SBES_measurement` aggiorna `cmd.contact` e `cmd.sensor_fail`
- **k**: `goal_controller` calcola nuovi flags diagnostici
- **k+1**: `state_machine` usa i nuovi flags per decidere transizioni

---

## Nuovi Campi Command (cmd)

### Inizializzazione (main_6DOF_3D.m, linee ~41-56):
```matlab
cmd.pitch_sensors_lost = false;     % Sensori 1-2 entrambi persi
cmd.roll_sensors_lost = false;      % Sensori 3-4 entrambi persi
cmd.diagonal_sensors_lost = false;  % Sensori diagonali persi (1-4 o 2-3)
cmd.sensor_fail_persistent = false; % Perdita persistita oltre grace period
cmd.recovery_timeout = false;       % Manovra recovery in timeout
cmd.recovery_progress = false;      % Recovery sta migliorando
```

### Aggiornamento (goal_controller.m):
- **pitch_sensors_lost**: `~cmd.contact(1) && ~cmd.contact(2)`
- **roll_sensors_lost**: `~cmd.contact(3) && ~cmd.contact(4)`
- **diagonal_sensors_lost**: `(~cmd.contact(1) && ~cmd.contact(4)) || (~cmd.contact(2) && ~cmd.contact(3))`
- **sensor_fail_persistent**: `true` dopo 500 steps (~0.5s) di failure
- **recovery_timeout**: `true` dopo 5000 steps (~5s) in stato recovery
- **recovery_progress**: `true` se `sensor_fail` diminuisce durante recovery

---

## Stati Macchina

### Stati Esistenti (migliorati):
1. **Idle** → TargetAltitude
2. **TargetAltitude** → ContactSearch
3. **ContactSearch** → Following / MovePitch / MoveRoll / RecoveryAltitude / Reset
4. **MovePitch** → ContactSearch / RecoveryAltitude (timeout)
5. **MoveRoll** → ContactSearch / RecoveryAltitude (timeout)
6. **Following** → MovePitch / MoveRoll / RecoveryAltitude / Reset
7. **Emergency** → TargetAltitude
8. **EndSimulation** → EndSimulation

### Nuovo Stato:
9. **RecoveryAltitude** → ContactSearch / Reset (timeout)
   - Aggiusta altitudine (-1m) per tentare recovery
   - Movimento ultra-lento
   - Usato per: perdite diagonali, timeout di MovePitch/MoveRoll

---

## Transizioni Chiave

### ContactSearch → Recovery:
```
sensor_fail == 0 → Following
sensor_fail == 4 → Reset
sensor_fail >= 2:
  ├─ pitch_sensors_lost → MovePitch
  ├─ roll_sensors_lost → MoveRoll
  ├─ diagonal_sensors_lost → RecoveryAltitude
  └─ else → ContactSearch (stay)
```

### Following → Recovery:
```
sensor_fail >= 2 && sensor_fail_persistent:
  ├─ pitch_sensors_lost → MovePitch
  ├─ roll_sensors_lost → MoveRoll
  ├─ diagonal_sensors_lost → RecoveryAltitude
  └─ else → Following (transient)
```

### Recovery → Escalation:
```
MovePitch/MoveRoll:
  ├─ recovery_timeout → RecoveryAltitude
  ├─ both sensors recovered → ContactSearch
  └─ else → stay in state

RecoveryAltitude:
  ├─ recovery_timeout → Reset
  ├─ sensor_fail == 0 → ContactSearch
  └─ else → stay in state
```

---

## Parametri Configurabili

### goal_controller.m:
```matlab
recovery_timeout_steps = 5000;  % Timeout recovery (5s @ 1ms)
grace_period_steps = 500;       % Grace period transient loss (0.5s)
```

### goal_def.m:
```matlab
% Velocità
u_star_recovery = 0.02;         % Surge in recovery (m/s)
v_star_recovery = -0.05;        % Sway in recovery (m/s)

% Aggiustamenti angolari
ang_to_cut_initial = pi/12;     % 15° per small errors
ang_to_cut_progressive = pi/8;  % 22.5° per large errors

% Altitudine
altitude_offset = 1.0;          % Adjustment in RecoveryAltitude (m)
```

---

## Pattern di Failure Gestiti

### 1. Same-Axis Loss (pitch o roll):
- **Sensori 1-2 persi** → `MovePitch`
  - Goal: pitch adjustment graduale
  - Velocità ridotta: `u_star_recovery = 0.02 m/s`
  - Timeout 5s → `RecoveryAltitude`

- **Sensori 3-4 persi** → `MoveRoll`
  - Goal: roll adjustment graduale
  - Velocità ridotta: `u_star_recovery = 0.02 m/s`
  - Timeout 5s → `RecoveryAltitude`

### 2. Diagonal Loss:
- **Sensori 1-4 o 2-3 persi** → `RecoveryAltitude`
  - Goal: riduzione altitudine target (-1m)
  - Movimento ultra-lento
  - Timeout 5s → `Reset`

### 3. Total Loss:
- **Tutti 4 sensori persi** → `Reset`
  - Torna ad altitudine target
  - Azzera orientazione
  - Riprova ContactSearch

### 4. Transient Loss:
- **Perdita < 500 steps** → Ignorata (grace period)
- **Perdita singola** → Tollerata (continua in stato corrente)

---

## Test Scenarios Consigliati

### Scenario 1: Pitch Sensors Loss
```
Condizioni: robot segue terreno, sensori 1-2 perdono contatto
Aspettativa:
  1. Grace period 0.5s
  2. Transizione Following → MovePitch
  3. Pitch adjustment graduale (+15° o +22.5°)
  4. Recovery entro 5s → ContactSearch
  5. Se fallisce → RecoveryAltitude → Reset
```

### Scenario 2: Roll Sensors Loss
```
Condizioni: robot segue terreno, sensori 3-4 perdono contatto
Aspettativa:
  1. Grace period 0.5s
  2. Transizione Following → MoveRoll
  3. Roll adjustment graduale
  4. Recovery entro 5s → ContactSearch
```

### Scenario 3: Diagonal Loss
```
Condizioni: sensori 1-4 persi (diagonal pattern)
Aspettativa:
  1. Transizione diretta → RecoveryAltitude
  2. Riduzione altitudine target (-1m)
  3. Recovery entro 5s → ContactSearch
  4. Se fallisce → Reset
```

### Scenario 4: Transient Glitch
```
Condizioni: perdita sensori < 0.5s
Aspettativa:
  1. Grace period attivo
  2. Nessuna transizione stato
  3. Continua Following senza interruzione
```

---

## Variabili Persistent (goal_controller.m)

Per tracciare stato recovery attraverso iterazioni:

```matlab
persistent recovery_start_step;        % Step iniziale recovery
persistent last_sensor_fail_count;    % Contatore failure precedente
persistent sensor_fail_grace_counter; % Contatore grace period
```

**IMPORTANTE**: Le variabili persistent mantengono stato tra chiamate, quindi:
- Reset automatico quando si esce da stati recovery
- Tracciamento accurato di timeout e progress
- No reset esplicito tra iterazioni

---

## Compatibilità

### File Modificati:
1. ✅ `main_6DOF_3D.m` - Aggiornato con nuovi campi cmd
2. ✅ `state_machine.m` - Refactored con nuovo stato e logica
3. ✅ `goal_def.m` - Migliorati goal per recovery states
4. ✅ `goal_controller.m` - Aggiunta diagnostica e timeout logic

### File Non Modificati (compatibili):
- `SBES_measurement.m` - Già aggiorna `cmd.contact` e `cmd.sensor_fail`
- `input_control.m` - Usa `goal` struct (no modifiche necessarie)
- `dynamic_model.m` - Usa `u` velocities (no modifiche necessarie)
- Tutte le altre funzioni EKF, sensori, terreno

---

## Vantaggi Implementati

1. **Robustezza**: Grace period evita oscillazioni da glitch temporanei
2. **Sicurezza**: Timeout garantiscono escape da recovery falliti
3. **Adattività**: Incrementi angolari graduali based on error magnitude
4. **Completezza**: Tutti i pattern di failure coperti (same-axis, diagonal, total)
5. **Traceability**: Logging dettagliato di ogni transizione con motivo e step
6. **Fallback**: Escalation gerarchica (MovePitch/Roll → RecoveryAltitude → Reset)

---

## Note di Debug

### Abilitare Logging Dettagliato:
Ogni transizione già stampa motivo e step. Per debug ulteriore, aggiungere in `goal_controller.m`:

```matlab
fprintf('Step %d: sensor_fail=%d, persistent=%d, pitch_lost=%d, roll_lost=%d\n', ...
        step, command.sensor_fail, command.sensor_fail_persistent, ...
        command.pitch_sensors_lost, command.roll_sensors_lost);
```

### Monitorare Variabili Persistent:
Aggiungere dopo calcolo flags in `goal_controller.m`:

```matlab
fprintf('Recovery tracking: start=%d, grace_counter=%d, timeout=%d\n', ...
        recovery_start_step, sensor_fail_grace_counter, command.recovery_timeout);
```

---

## Conclusioni

Il sistema migliorato fornisce:
- **Recovery automatico** da perdite sensori con strategia multi-livello
- **Prevenzione loop infiniti** via timeout e fallback gerarchico
- **Tolleranza transient** via grace period
- **Diagnostica avanzata** per identificare pattern di failure
- **Manovre graduate** con velocità e incrementi angolari adattivi

Tutte le modifiche sono **backward compatible** con il resto del codice esistente.

# MODIFICHE AI GRAFICI E ANALISI DATI - 01 Dicembre 2025

## Sommario delle Modifiche

Sono state implementate le seguenti modifiche al sistema di visualizzazione e analisi dati:

### 1. **Grafico Traiettoria 3D - NED Convention** ✓
- **File modificato**: `visualization/plot_results.m`
- **Modifica**: L'asse Z è ora invertito per rispettare la convenzione NED (North-East-Down)
- **Dettagli**: 
  - Usa `-prob(3,:)` invece di `prob(3,:)`
  - Etichette aggiornate: "X (North)", "Y (East)", "Z (Down) - NED"
  - `set(gca, 'ZDir', 'reverse')` per visualizzazione corretta
- **Beneficio**: Visualizzazione più chiara e conforme agli standard aeronautici/sottomarini

### 2. **Robot Ingrandito nella Visualizzazione** ✓
- **File modificato**: `visualization/m_visualization.m`
- **Modifica**: Dimensioni del robot aumentate del 50%
- **Dettagli**:
  - `Lx = 1.2m` (era 0.8m) - lunghezza
  - `Wy = 0.6m` (era 0.4m) - larghezza
  - `Hz = 0.3m` (era 0.2m) - altezza
- **Beneficio**: Robot più visibile nei grafici 3D rispetto al terreno

### 3. **Script per Rigenerare Grafici** ✓
- **Nuovo file**: `visualization/replay_plots.m`
- **Funzionalità**: Carica dati salvati e rigenera TUTTI i grafici
- **Uso**:
  ```matlab
  cd visualization
  replay_plots('run_20251201_105813')  % Specifica run
  replay_plots()                        % Selezione interattiva
  ```
- **Output**: Tutti i grafici di plot_results + statistiche performance

### 4. **Verifica Dati per Analisi Tesi** ✓
- **Nuovo file**: `data_management/data_analysis_checklist.m`
- **Funzionalità**: Verifica completezza dati per analisi performance tesi
- **Uso**:
  ```matlab
  cd data_management
  data_analysis_checklist('run_20251201_105813')
  ```
- **Output**: Report completo su disponibilità dati e metriche calcolabili

---

## Come Utilizzare le Nuove Funzionalità

### Scenario 1: Rigenerare Tutti i Grafici da Dati Salvati

```matlab
% 1. Vai nella directory visualization
cd visualization

% 2. Esegui replay_plots
replay_plots('run_20251201_105813')

% Oppure per selezione interattiva
replay_plots()
```

**Output atteso**:
- Tutti i grafici degli stati (altitude, alpha, beta)
- Grafici angoli robot (roll, pitch, yaw)
- Grafici input di controllo (u, v, w, p, q, r)
- Analisi normali (parallelismo, z-sign)
- Traiettoria 3D con NED convention
- Statistiche di performance sulla console

---

### Scenario 2: Verificare Dati Disponibili per Tesi

```matlab
% 1. Vai nella directory data_management
cd data_management

% 2. Esegui checklist
data_analysis_checklist('run_20251201_105813')
```

**Output atteso**:
- Lista completa dei dati disponibili (51 fields)
- Metriche calcolabili per ogni categoria:
  - Tracking performance
  - EKF performance
  - Sensor performance
  - Control performance
  - Trajectory analysis
- Preview metriche base (MAE, RMSE, etc.)

---

### Scenario 3: Analisi Completa per la Tesi

```matlab
% 1. Esegui simulazione e salva dati
main_6DOF_3D  % Rispondere Y per salvare

% 2. Verifica dati salvati
cd data_management
data_analysis_checklist('run_YYYYMMDD_HHMMSS')

% 3. Rigenera grafici per report
cd ../visualization
replay_plots('run_YYYYMMDD_HHMMSS')

% 4. Analisi statistica multi-run (se disponibile)
cd ../data_management
stats = analyze_statistics()
```

---

## Dati Salvati per Analisi Performance

### ✓ CATEGORIE COMPLETE

#### 1. **Tracking Performance**
- Altitudine di riferimento (`h_ref`)
- Stati veri (`x_true`: h, α, β)
- Stati stimati (`x_est`)
- Vettore temporale (`time`)

**Metriche calcolabili**:
- MAE, RMSE, Max Error (altitudine e angoli)
- Tempo di settling
- Overshoot/Undershoot
- Steady-state error

#### 2. **Extended Kalman Filter**
- Stati predetti (`x_pred`)
- Covarianza finale (`P_final`)
- Innovazione (`ni`)
- Covarianza innovazione (`S`)
- Process noise (`Q`)

**Metriche calcolabili**:
- Convergenza covarianza (trace P)
- NEES (Normalized Estimation Error Squared)
- Consistency check (whiteness test)
- Autocorrelazione residui

#### 3. **Sensori (SBES, AHRS)**
- Misure SBES (`z_meas`, `z_pred`)
- Normali terreno (`n_mes`, `n_est`, `n_pre`)
- Angoli robot (`rob_rot`, `clean_rot`)
- Covariance rumore (`R`, `R_tp`, `R_a`)

**Metriche calcolabili**:
- Errore misure vs predizioni
- Parallelismo normali (angoli tra vettori)
- Impatto rumore AHRS
- Statistiche sensor failure

#### 4. **Controllo PID**
- Velocità robot (`u`: [u,v,w,p,q,r])
- Accelerazioni (`u_dot`)
- Uscite PID (`pid`)
- Errori (proporzionale, integrale)
- Guadagni (`Kp`, `Ki`, `Kd`, `Kt`)

**Metriche calcolabili**:
- Sforzo di controllo (control effort)
- Energia consumata
- Smoothness (jerk analysis)
- Rise/settling time

#### 5. **Traiettoria e Orientamento**
- Posizione robot (`prob`: [x,y,z])
- Rotazioni robot (`wRr`)
- Rotazioni terreno (`wRt`)
- Stati macchina (`state`)

**Metriche calcolabili**:
- Distanza percorsa
- Velocità media/max
- Path smoothness
- Tempo per stato

---

## Struttura File Salvati

```
results/
  run_YYYYMMDD_HHMMSS/
    ├── ekf_states.mat          # Stati EKF (true, estimated, predicted)
    ├── ekf_covariance.mat      # Covarianza e innovazione
    ├── sensor_data.mat         # SBES, AHRS, normali
    ├── control_data.mat        # PID, velocità, errori
    ├── trajectory.mat          # Posizione e rotazioni
    ├── parameters.mat          # Parametri simulazione
    └── metadata.txt            # Info run (human-readable)
```

---

## Script Disponibili

| Script | Posizione | Funzione |
|--------|-----------|----------|
| `replay_plots.m` | `visualization/` | Rigenera tutti i grafici da dati salvati |
| `data_analysis_checklist.m` | `data_management/` | Verifica completezza dati per tesi |
| `analyze_statistics.m` | `data_management/` | Analisi statistica multi-run |
| `collect_simulation_data.m` | `data_management/` | Raccoglie dati simulazione |
| `save_simulation_data.m` | `data_management/` | Salva dati in formato strutturato |
| `load_simulation_data.m` | `data_management/` | Carica dati salvati |
| `test_modifications.m` | `.` | Test automatico modifiche |

---

## Test Eseguiti ✓

Tutti i test sono stati eseguiti con successo:

1. ✓ Caricamento dati salvati (51 fields)
2. ✓ Dimensioni robot ingrandite (1.2×0.6×0.3m)
3. ✓ Asse Z invertito con NED convention
4. ✓ Generazione grafico 3D con nuova convenzione
5. ✓ Script `replay_plots.m` funzionante
6. ✓ Script `data_analysis_checklist.m` funzionante
7. ✓ Tutti i dati necessari per tesi presenti

**Per eseguire i test**: `matlab -batch "test_modifications; exit;"`

---

## Metriche Raccomandate per la Tesi

### Performance di Tracking
1. **Altitude Tracking**: MAE, RMSE, max error, settling time
2. **Angle Tracking**: α e β errors, convergenza

### Prestazioni EKF
3. **Convergenza**: trace(P) nel tempo
4. **Consistenza**: NEES, innovation whiteness test
5. **Accuratezza**: prediction error vs estimation error

### Qualità Sensori
6. **SBES**: errore misure, detection rate
7. **Normali**: parallelismo (est vs mes vs robot), angoli
8. **AHRS**: impatto rumore su tracking

### Prestazioni Controllo
9. **Control Effort**: energia totale, smoothness
10. **Stabilità**: rise time, overshoot, settling
11. **Robustezza**: performance con rumore e fallimenti

---

## Note Importanti

- **NED Convention**: Ora tutti i grafici 3D usano la convenzione North-East-Down (standard aeronautico/sottomarino)
- **Robot Ingrandito**: Meglio visibile nei grafici rispetto al terreno (1.5× dimensioni originali)
- **Dati Completi**: 51 campi salvati, sufficienti per analisi completa tesi
- **Replay Grafici**: Puoi rigenerare tutti i grafici senza rieseguire la simulazione

---

## Contatto e Supporto

Per domande o problemi con le nuove funzionalità:
1. Verifica la presenza dei dati: `data_analysis_checklist()`
2. Controlla i log di errore in MATLAB
3. Esegui test: `test_modifications`

---

**Data creazione**: 01 Dicembre 2025  
**Versione**: 1.0  
**Ultima modifica**: 01 Dicembre 2025

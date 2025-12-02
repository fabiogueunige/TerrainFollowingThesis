# Performance Metrics Implementation

## Overview

Implementazione completa delle metriche di performance secondo **RESULTS_GUIDE_2.pdf** (Sezione 2.4).

## 📊 Nuove Funzioni Aggiunte

### 1. `compute_performance_metrics.m`

**Scopo:** Calcola tutte le metriche di performance richieste dalla guida.

**Input:** Struttura `sim_data` da `load_simulation_data()`

**Output:** Struttura `metrics` contenente:

- **altitude_tracking** - RMS altitude error
- **angle_tracking** - Mean angle errors (|φ-α|, |θ-β|)
- **sensor_failure_rate** - % tempo con sensor_fail > 0
- **state_transitions** - Numero transizioni per minuto
- **control_effort** - Magnitudine media degli output PID
- **innovation_max** - Massima magnitudine dell'innovazione ||ν||∞
- **normal_parallelism** - Angolo tra n_est e n_mes (Eq 2.6)
- **robot_alignment** - Angolo tra z_robot e n_est (Eq 2.7)

**Esempio:**

```matlab
sim_data = load_simulation_data('run_20251201_120426');
metrics = compute_performance_metrics(sim_data);
```

### 2. `display_performance_metrics.m`

**Scopo:** Visualizza le metriche in formato leggibile e strutturato.

**Input:** 
- `metrics` - Struttura da `compute_performance_metrics`
- `verbose` - (Opzionale) true per output dettagliato, false per sommario

**Output:** Visualizzazione formattata su console con:
- Box formattati per ogni categoria
- Interpretazione automatica dei risultati (✓, ⚠, ✗)
- Overall performance score (A-F)

**Esempio:**

```matlab
% Modalità completa
display_performance_metrics(metrics, true);

% Solo sommario
display_performance_metrics(metrics, false);
```

### 3. `example_performance_analysis.m`

**Scopo:** Script dimostrativo per analisi delle performance.

**Dimostra:**
1. Analisi singola run
2. Batch analysis su multiple run
3. Quick performance check

**Utilizzo:**

```matlab
example_performance_analysis
```

### 4. Integrazione in `replay_plots.m`

**Modifiche:** Aggiunta chiamata automatica a `compute_performance_metrics` e `display_performance_metrics` quando si riproducono i grafici.

**Utilizzo (invariato):**

```matlab
replay_plots('run_20251201_120426')
```

Ora include automaticamente le metriche complete alla fine!

## 📈 Metriche Implementate vs RESULTS_GUIDE

### ✅ Completamente Implementate

| Metrica | Riferimento Guida | Implementazione |
|---------|-------------------|-----------------|
| RMS altitude error | Pagina 4, §2.4.3 | `metrics.altitude_tracking` |
| Mean angle error | Pagina 4, §2.4.3 | `metrics.angle_tracking.*` |
| Sensor failure rate | Pagina 4, §2.4.3 | `metrics.sensor_failure_rate` |
| State transitions/min | Pagina 4, §2.4.3 | `metrics.state_transitions.per_minute` |
| Control effort | Pagina 4, §2.4.3 | `metrics.control_effort.*` |
| Maximum innovation | Pagina 13, §2.6.3 | `metrics.innovation_max` |
| Normal parallelism | Pagina 3, Eq 2.6 | `metrics.normal_parallelism.*` |
| Robot-terrain alignment | Pagina 3, Eq 2.7 | `metrics.robot_alignment.*` |

### 📊 Metriche Aggiuntive (Bonus)

| Metrica | Descrizione |
|---------|-------------|
| `altitude_errors.*` | MAE, RMSE, max, std |
| `alpha_errors.*` | MAE, RMSE, max per alpha |
| `beta_errors.*` | MAE, RMSE, max per beta |
| `state_occupancy.*` | % tempo in ogni stato |
| `control_effort` (per-axis) | RMS per u,v,w,p,q,r |
| `innovation_mean/std` | Statistiche complete innovazione |
| `normal_parallelism.below_5deg` | % sotto soglia 5° |

## 🎯 Interpretazione Automatica

La funzione `display_performance_metrics` fornisce valutazione automatica:

### Altitude Tracking
- ✓ Excellent: < 0.3 m
- ✓ Good: < 0.5 m
- ⚠ Acceptable: < 1.0 m
- ✗ Poor: ≥ 1.0 m

### Sensor Failure Rate
- ✓ Excellent: < 5%
- ✓ Good: < 10%
- ⚠ Acceptable: < 20%
- ✗ High: ≥ 20%

### Mission Stability (Transitions/min)
- ✓ Stable: < 2/min
- ⚠ Moderate: 2-5/min
- ⚠ High: ≥ 5/min

### Normal Parallelism
- ✓ Excellent: < 5°
- ⚠ Good: < 10°
- ⚠ Check: ≥ 10°

### Robot-Terrain Alignment
- ✓ Well aligned: < 10°
- ⚠ Moderate: < 20°
- ⚠ Poor: ≥ 20°

### Angle Tracking
- ✓ Excellent: < 3°
- ✓ Good: < 5°
- ⚠ Acceptable: < 10°
- ✗ Poor: ≥ 10°

## 📝 Workflow Consigliato

### 1. Analisi Singola Run

```matlab
% Carica dati
sim_data = load_simulation_data();

% Calcola metriche
metrics = compute_performance_metrics(sim_data);

% Visualizza risultati completi
display_performance_metrics(metrics, true);

% Salva metriche
save('my_metrics.mat', 'metrics');
```

### 2. Analisi Batch Multiple Run

```matlab
% Usa script di esempio
example_performance_analysis

% Oppure manualmente
results_dir = 'results';
runs = dir(fullfile(results_dir, 'run_*'));

batch_metrics = cell(length(runs), 1);
for i = 1:length(runs)
    data = load_simulation_data(runs(i).name);
    batch_metrics{i} = compute_performance_metrics(data);
end

% Statistiche aggregate
alt_errors = cellfun(@(m) m.altitude_tracking, batch_metrics);
fprintf('Mean altitude RMS: %.4f ± %.4f m\n', mean(alt_errors), std(alt_errors));
```

### 3. Replay con Metriche Automatiche

```matlab
% Ora replay_plots include automaticamente le metriche!
replay_plots('run_20251201_120426')
```

## 🔧 Requisiti Dati

Per calcolare tutte le metriche, `sim_data` deve contenere:

**Essenziali:**
- `x_true`, `x_est`, `h_ref` - Stati EKF
- `u` - Input di controllo
- `t` - Vettore tempo

**Opzionali (ma raccomandati):**
- `sensor_fail` - Indicatore fallimenti sensori
- `state` - Storia state machine
- `ni` - Innovazione EKF
- `n_est`, `n_mes` - Normali stimate/misurate
- `wRr`, `wRr_noisy` - Matrici rotazione robot

Se un campo manca, la funzione emette un warning e continua con le metriche disponibili.

## 🆕 Compatibilità

- ✅ Compatibile con `load_simulation_data` esistente
- ✅ Nessuna modifica richiesta al codice di simulazione
- ✅ Funziona con dati salvati in precedenza
- ✅ Integrazione trasparente in `replay_plots`

## 📚 Riferimenti

- **RESULTS_GUIDE_2.pdf** - Sezioni 2.4.2, 2.4.3, 2.4.4
  - §2.4.2.4: Normal Vector Analysis (Eq 2.6, 2.7)
  - §2.4.3: Batch Analysis and Statistical Evaluation
  - §2.6.3: System-Level Validation Scenarios

## 🎓 Per la Tesi

Queste metriche coprono **tutti i requisiti** della guida per:
- ✅ State Tracking Plots
- ✅ Robot Angle Plots
- ✅ Control Input Time Histories
- ✅ Normal Vector Analysis
- ✅ Performance Metrics
- ✅ Batch Analysis
- ✅ Statistical Evaluation

Puoi utilizzare:
1. `display_performance_metrics` per tabelle nei risultati
2. `batch_results.summary` per statistiche aggregate
3. Metriche salvate in `.mat` per processing ulteriore

# Sistema di Salvataggio Dati - Guida Rapida

## 🎯 Cosa è stato implementato

Ho creato un sistema completo per salvare e analizzare i dati delle simulazioni. Ecco cosa puoi fare:

### ✅ Funzionalità Implementate

1. **Salvataggio Automatico**: Alla fine di ogni simulazione, ti viene chiesto se vuoi salvare i dati
2. **Organizzazione Automatica**: Ogni run viene salvata in una cartella separata con timestamp
3. **Dati Completi**: Salva TUTTI i dati importanti della simulazione
4. **Analisi Statistica**: Funzioni per confrontare multiple simulazioni
5. **Report Automatici**: Genera grafici e tabelle comparative

---

## 📊 Dati Salvati (Automaticamente)

### Categorie Principali

**1. Stati EKF** (`ekf_states.mat`)
- `x_true` - Stati veri del terreno [h, alpha, beta]
- `x_est` - Stati stimati dall'EKF
- `x_pred` - Stati predetti
- `h_ref` - Riferimento altitudine

**2. Covarianza EKF** (`ekf_covariance.mat`)
- `ni` - Innovation (errore misura)
- `S` - Covarianza innovation
- `P_final` - Covarianza finale
- `P0` - Covarianza iniziale

**3. Dati Sensori** (`sensor_data.mat`)
- `z_meas` - Misure SBES (4 sensori)
- `z_pred` - Misure predette
- `n_mes`, `n_est`, `n_pre` - Normali (misurate, stimate, predette)
- `rob_rot` - Angoli robot (con rumore)
- `clean_rot` - Angoli robot (puliti)

**4. Dati Controllo** (`control_data.mat`)
- `pid` - Output PID
- `u` - Velocità robot [u,v,w,p,q,r]
- `u_dot` - Accelerazioni
- `goal` - Setpoint desiderati
- Errori del controllore

**5. Traiettoria** (`trajectory.mat`)
- `prob` - Posizione robot
- `wRr`, `wRt` - Matrici di rotazione
- `state` - Stati della macchina a stati

**6. Parametri** (`parameters.mat`)
- Parametri simulazione (Ts, Tf, N)
- Parametri EKF (Q, R, P0)
- Guadagni controllore (Kp, Ki, Kd, Kt)
- Condizioni iniziali
- **Parametri generazione terreno** (max_planes, step_length, angle_range, rate_of_change, delta_limit, pp_init_w, n0)

**7. Metadata** (`metadata.txt`)
- File di testo leggibile con sommario della simulazione
- Include anche parametri del terreno

---

## 🚀 Come Usare

### 1. Eseguire Simulazione e Salvare

```matlab
% Esegui la simulazione normalmente
main_6DOF_3D

% Alla fine ti verrà chiesto:
% Save data? (Y/N): Y
% 
% Choose option (1 or 2): 1  (timestamp automatico)
%                        2  (nome personalizzato)
```

**Esempio con Nome Personalizzato:**
```
Save data? (Y/N): Y
Choose option (1 or 2): 2
Enter custom run name: test_alta_quota
```
Verrà salvato in: `results/run_test_alta_quota/`

### 2. Caricare Dati Salvati

```matlab
% Carica run specifica
sim_data = load_simulation_data('run_20251023_143022');

% Mostra lista e scegli
sim_data = load_simulation_data('');

% Accedi ai dati
errore_altitudine = sim_data.h_ref - sim_data.x_est(1,:);
velocita_surge = sim_data.u(1,:);

% Parametri del terreno
buffer_terreno = sim_data.max_planes;        % 300
distanza_piani = sim_data.step_length;       % 4 m
angoli_limite = sim_data.angle_range;        % [-π/3, π/3]
variazione_terreno = sim_data.rate_of_change; % 3  
max_cambiamento_angolo = sim_data.delta_limit % π/4
```

### 3. Analisi Statistica

```matlab
% Analizza tutte le simulazioni salvate
stats = analyze_statistics();

% Output automatico:
% - Mean altitude error: 0.1234 ± 0.0567 m
% - Control effort: Surge=0.234 m/s
% - Ecc...

% Accedi ai risultati
errore_medio = stats.altitude_tracking.mean_error;
sforzo_controllo = stats.control_effort.surge_mean;
```

### 4. Analisi Batch con Grafici

```matlab
% Script esempio per analisi completa
example_batch_analysis

% Genera automaticamente:
% - analysis_results/altitude_error_comparison.png
% - analysis_results/control_effort_comparison.png
% - analysis_results/timeseries_comparison.png
% - analysis_results/statistics_summary.csv
% - analysis_results/analysis_report.txt
```

---

## 📁 Struttura Cartelle

Dopo aver salvato alcune simulazioni:

```
matlab_3D/
├── results/                          # CREATA AUTOMATICAMENTE
│   ├── run_20251023_143022/          # Timestamp automatico
│   │   ├── ekf_states.mat
│   │   ├── ekf_covariance.mat
│   │   ├── sensor_data.mat
│   │   ├── control_data.mat
│   │   ├── trajectory.mat
│   │   ├── parameters.mat
│   │   └── metadata.txt
│   ├── run_20251023_150545/
│   │   └── ...
│   └── run_test_alta_quota/          # Nome personalizzato
│       └── ...
└── data_management/
    ├── save_simulation_data.m        # ✓ Creato
    ├── collect_simulation_data.m     # ✓ Creato
    ├── load_simulation_data.m        # ✓ Creato
    ├── analyze_statistics.m          # ✓ Creato
    ├── example_batch_analysis.m      # ✓ Creato
    └── README.md                     # ✓ Creato
```

---

## 💡 Esempi d'Uso

### Esempio 1: Confronta Due Configurazioni

```matlab
% Run 1: Controller A
% (modifica guadagni Kp, Ki, Kd)
main_6DOF_3D
% Salva come: "run_controller_A"

% Run 2: Controller B
% (modifica guadagni diversi)
main_6DOF_3D
% Salva come: "run_controller_B"

% Confronta
stats = analyze_statistics({'run_controller_A', 'run_controller_B'});
```

### Esempio 2: Confronta Diversi Terreni

```matlab
% Run 1: Terreno piatto (angle_range piccolo)
% Modifica: angle_range = [-pi/6, pi/6]
main_6DOF_3D
% Salva come: "run_terrain_flat"

% Run 2: Terreno ripido (angle_range grande)
% Modifica: angle_range = [-pi/3, pi/3]
main_6DOF_3D
% Salva come: "run_terrain_steep"

% Confronta performance
stats = analyze_statistics({'run_terrain_flat', 'run_terrain_steep'});
```

### Esempio 3: Monte Carlo (10 Simulazioni)

```matlab
% Esegui 10 volte con rumore diverso
for i = 1:10
    main_6DOF_3D
    % Salva con timestamp automatico (opzione 1)
end

% Analizza tutto
stats = analyze_statistics();
% Otterrai: media ± deviazione standard su 10 run
```

### Esempio 4: Analisi Custom

```matlab
% Carica dati
sim1 = load_simulation_data('run1');
sim2 = load_simulation_data('run2');

% Plot personalizzato
figure;
plot(sim1.time, sim1.x_est(1,:), 'b');
hold on;
plot(sim2.time, sim2.x_est(1,:), 'r');
legend('Run 1', 'Run 2');
xlabel('Tempo [s]');
ylabel('Altitudine [m]');
title('Confronto Tracking Altitudine');
```

---

## 📈 Metriche Calcolate Automaticamente

L'analisi statistica calcola:

### Tracking Altitudine
- Errore medio assoluto
- Errore RMS
- Errore massimo
- Deviazione standard

### Tracking Angoli
- Errore alpha (roll terreno)
- Errore beta (pitch terreno)

### Sforzo di Controllo
- RMS velocità surge
- RMS velocità heave
- Statistiche su tutti i 6 DOF

### Innovation EKF
- Norma media innovation
- Norma massima innovation
- (utile per check consistenza EKF)

---

## 🎨 Grafici Generati da `example_batch_analysis`

1. **Altitude Error Comparison**
   - Bar chart errori per run
   - Istogramma distribuzione
   - Box plot statistiche

2. **Control Effort Comparison**
   - Sforzo controllo surge per run
   - Sforzo controllo heave per run

3. **Time Series Comparison**
   - Errore altitudine nel tempo (3 run)
   - Segnale controllo surge
   - Segnale controllo heave

4. **Statistics Summary (CSV)**
   - Tabella esportabile in Excel
   - Tutte le metriche per run

5. **Analysis Report (TXT)**
   - Report testuale leggibile
   - Sommario aggregato
   - Risultati per singola run

---

## 🔧 Personalizzazione

### Aggiungere Metriche Custom

Modifica `analyze_statistics.m`, funzione `compute_run_metrics`:

```matlab
% Esempio: aggiungi tempo di settling
settling_idx = find(abs(h_error) < 0.1, 1, 'first');
run_stats.altitude.settling_time = sim_data.time(settling_idx);
```

### Salvare Dati Aggiuntivi

Modifica `collect_simulation_data.m`:

```matlab
% Aggiungi alla fine
sim_data.mia_variabile = mia_variabile;
```

Modifica `save_simulation_data.m` per salvarla:

```matlab
% Nella sezione appropriata
ekf_states.mia_variabile = sim_data.mia_variabile;
```

---

## ⚠️ Note Importanti

1. **Spazio Disco**: Ogni run occupa ~10-50 MB (dipende da N)
2. **Backup**: Salva la cartella `results/` per archiviazione
3. **Nomi**: Usa nomi descrittivi per run importanti
4. **Metadata**: Controlla sempre `metadata.txt` per parametri run
5. **Compatibilità**: File `.mat` salvati in formato v7 (compatibile tutte versioni MATLAB)

---

## 📚 File di Documentazione

- `data_management/README.md` - Documentazione completa (inglese)
- `data_management/GUIDA_RAPIDA_IT.md` - Questo file (italiano)
- `README.md` (root) - Aggiornato con sezione Data Management

---

## ✅ Checklist Workflow

**Workflow Consigliato:**

1. ☑ Modifica parametri simulazione (se necessario)
2. ☑ Esegui `main_6DOF_3D`
3. ☑ Alla fine: scegli **Y** per salvare
4. ☑ Scegli nome personalizzato o timestamp
5. ☑ Ripeti per diverse configurazioni
6. ☑ Esegui `analyze_statistics()` per confronto
7. ☑ (Opzionale) Esegui `example_batch_analysis` per report completo
8. ☑ Controlla `analysis_results/` per grafici e CSV

---

## 🆘 Risoluzione Problemi

**Problema:** "No results directory found"
- **Soluzione:** Esegui almeno una simulazione e salva i dati

**Problema:** "Run directory not found"
- **Soluzione:** Controlla nome run, usa `load_simulation_data('')` per lista

**Problema:** Memoria insufficiente
- **Soluzione:** Analizza run in batch più piccoli

**Problema:** Dati sovrascritti
- **Soluzione:** Usa sempre nomi diversi o timestamp automatici

---

## 📧 Supporto

Per domande o problemi:
1. Controlla `data_management/README.md`
2. Verifica esempi in `example_batch_analysis.m`
3. Controlla metadata.txt delle run salvate

---

**Creato:** 23 Ottobre 2025  
**Versione:** 1.0  
**Autore:** Sistema di Data Management per TF_6DOF

---

## 🎉 Funzionalità Chiave

✅ Salvataggio automatico con prompt  
✅ Cartelle separate per ogni run  
✅ Timestamp automatici  
✅ Nomi personalizzati  
✅ Tutti i dati importanti salvati (inclusi parametri terreno)  
✅ Caricamento facile  
✅ Analisi statistica automatica  
✅ Report e grafici  
✅ Export CSV  
✅ Documentazione completa  

**Pronto all'uso! Buone analisi! 🚀**

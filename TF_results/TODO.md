# MasterThesis TODO

## CONTROLLO

- Controlla cosa diverge e rivedi controllori -> aggiungi zeri e controlla nuovi poli dove diverge
- Lavora su instabilità del sistema -> scopri quali siano quelli che danno maggiori problemi e quali funzionano relativamente bene
- Aggiungi il cambio di valore dei guadagni in tempo reale ogni 3 secondi (mantienili comunque limitati) -> permesso dalla struttura delta del PID

### SIDE QUEST: (PERSO LE FOTO)

- Sistema controllori PI (surge e sway) come richiesto da professore e provare nuovi tipi consigliati -> Pi e p2 meglio aggiungere lo zero ottenuto dal denominatore e svolgere il luogo delle radici. Però consiglia di fare quello suggerito e dato dalle foto per sistemare divergenza nel controllo.

## MODELLO

- Check delle equazioni delle restoring (g o tau_r) del blueRov2
- Riporta dinamica a non lineare
- Metti rumore a modello dinamica nei test fino a (30%) (software)

## SOFTWARE

- Aggiungi EKF della posizione
- Sistema strutture / classi e metti insieme segnale pulito e rumoroso -> rendere più chiaro
- Transizione quando il goal cambia (quale e come)?
- Atuuare yaw per dare un path?
- Coefficiente roughness nelle superfici?

### SIDE QUEST

Plottare con video lo spostamento nel tempo del robot

## STONEFISH

- Aggiungi simulazione blueRov2 che copia il terreno (più semplice possibile)

## RISULTATI

- Aggiungere controllo correlazione e innovazione per KF
- Mettere covarianza e fare tanti test

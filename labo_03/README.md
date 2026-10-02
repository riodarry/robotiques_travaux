# Labo 03 - Integration progressive

## Etape 1 : demarrage, FSM et DEL

Cette version partielle integre une partie d'un prototype deja prepare.
L'historique documente les integrations, corrections et verifications reelles;
il ne reconstitue pas les dates de creation du prototype.

DA : 2409626; deux derniers chiffres : 26; variation paire.

- Deux etats : `SETUP_STATE` puis `READY_STATE`.
- Attente de 3000 ms avec `millis()`, sans `delay()`.
- Animation progressive des DEL 1 a 7 pendant cette attente.
- Moteurs maintenus a l'arret; le parcours n'est pas encore integre.
- Traces d'entree dans les etats et journal serie toutes les 250 ms.
- `DEBUG_ENABLED` permet de desactiver les traces.

## Sources du professeur

- [Modele FSM avec firstTime](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/autres/modele_fsm/modele_fsm.ino) : repartition par `switch` et code d'entree dans chaque etat.
- [Temporisations sans delay](https://nbourre.github.io/1sx_cours_notes/1SX_cours_01b_rappels_no_delay/01_delai_sans_delay/) : difference de temps avec `millis()`.
- [DEL et moteurs](https://nbourre.github.io/1sx_cours_notes/1SX_cours_04_dels_moteurs/) : anneau RGB, broche 44 et commandes PWM.

## Verifications

La compilation et les essais physiques sont des controles differents.
Le 2 octobre 2026 : compilation reussie pour `arduino:avr:mega`,
avec Arduino AVR Boards 1.8.6 : 7772 octets de programme et 1338 octets de RAM.
Les avertissements observes viennent de MakeBlock et du coeur Arduino.
Tests isoles : 29 assertions reussies (attente, transition, DEL, traces
periodiques et absence de commande moteur non nulle).
Cette version n'a pas ete televersee ni observee sur le robot.

Lors d'un essai manuel autorise : verifier l'animation pendant 3 secondes,
le passage a `READY_STATE`, la DEL 12 verte et l'absence de mouvement.

## Parties suivantes

1. Encodeurs, gyroscope, segment 1 et progression des DEL.
2. Premier pivot gauche de 90 degres et protection en cas de blocage.
3. Segment 2, sonar et arret a 30 cm avec validation des mesures.
4. Deuxieme pivot droite de 90 degres et segment 3.
5. Pivot final de 180 degres et DEL 8 a 12 clignotant a 10 Hz.
6. Corrections constatees, calibration manuelle et verifications de remise.

Les parties non integrees ne sont pas considerees comme terminees.

# Labo 03

DA : 2409626, derniers chiffres 26 : gauche puis droite (pivots a venir).

## Pour l'instant

Demarrage et segment 1 du prototype : attente 3 s, encodeurs, gyroscope et DEL 1 a 7.
DEL 12 verte, puis arret sans pivot. `SEGMENT_1_DISTANCE_CM` : 100 cm provisoires, a remplacer par X.
Les traces serie montrent l'etat, la distance et l'angle toutes les 250 ms.

Compilation Mega AVR 1.8.6 et tests isoles reussis. Distance reelle NON TESTEE.
Suite : premier pivot gauche, puis sonar et autres segments.

## Cours utilises

- [FSM avec firstTime](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/autres/modele_fsm/modele_fsm.ino)
- [Delais et DEL](https://nbourre.github.io/1sx_cours_notes/1SX_cours_01b_rappels_no_delay/01_delai_sans_delay/#d%C3%A9lai-sans-delay), [anneau RGB](https://nbourre.github.io/1sx_cours_notes/1SX_cours_04_dels_moteurs/#lumi%C3%A8re-avant-tout)
- [Gyroscope et correction PD](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_straight/ranger_straight.ino), [conversion distance/rotation](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_avance_precis/ranger_avance_precis.ino)

Adaptations : progression, limite PD, passage +/-180 et arret de securite a 20 s.

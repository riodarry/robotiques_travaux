# Labo 03

DA : 2409626, derniers chiffres 26 : premier pivot gauche, prochain pivot droite.

## Pour l'instant
Attente 3 s, segment 1, pivot gauche puis segment 2 du prototype; arret avant le second pivot.
Sonar PORT_10 lu toutes les 100 ms, avec attente d'echo limitee a 25 ms.
Arret a 30 cm confirme par trois lectures dans 28-32 cm; trop proche = erreur, pas reussite.
DEL 1 a 7 progressives, 12 puis 11 vertes. Traces serie toutes les 250 ms.
`SEGMENT_1_DISTANCE_CM` : 100 cm provisoires, a remplacer par X.
Compilation Mega AVR 1.8.6 et tests isoles reussis. Distance, sens et angle reels NON TESTES.
Suite : pivot droite et segment 3. Aucun televersement automatique.

## Cours utilises
- [FSM avec firstTime](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/autres/modele_fsm/modele_fsm.ino)
- [Delais et DEL](https://nbourre.github.io/1sx_cours_notes/1SX_cours_01b_rappels_no_delay/01_delai_sans_delay/#d%C3%A9lai-sans-delay), [anneau RGB](https://nbourre.github.io/1sx_cours_notes/1SX_cours_04_dels_moteurs/#lumi%C3%A8re-avant-tout)
- [Gyroscope et correction PD](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_straight/ranger_straight.ino), [conversion distance/rotation](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_avance_precis/ranger_avance_precis.ino)
- [Pivot avec les encodeurs : spin()](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_tourne_precis/ranger_tourne_precis.ino)
- [Sonar : impulsion et lecture](https://nbourre.github.io/1sx_cours_notes/1SX_cours_02_ultrason_hum_temp/#exemple-de-code)
Adaptations : limites de temps/PD, lecture bornee, rejet des mesures et confirmation d'arret.

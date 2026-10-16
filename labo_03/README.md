Labo 03

DA : 2409626, derniers chiffres 26 : premier pivot gauche, deuxieme pivot droite.

Pour l'instant :
Parcours principal du prototype integre : trois segments, pivots 90/90/180 degres.
Sonar PORT_10 lu toutes les 100 ms, avec attente d'echo limitee a 25 ms.
Arret a 30 cm confirme par trois lectures dans 28-32 cm; trop proche = erreur, pas reussite.
DEL 1 a 7 progressives; temoins 12/11/10 cumules, puis 8 a 12 vertes, 50 ms ON/50 ms OFF (10 Hz).
Segments 1 et 3 : 100 cm provisoires, a remplacer par X. Arret si un mouvement prend trop de temps.
Le code compile pour Mega AVR 1.8.6; 44 essais sur ordinateur passent. Mesures reelles a verifier.
Restent les essais physiques, le formulaire Git et le ZIP sur Lea. Aucun envoi automatique au robot.

Cours utilises :
- [FSM avec firstTime](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/autres/modele_fsm/modele_fsm.ino)
- [Delais et DEL](https://nbourre.github.io/1sx_cours_notes/1SX_cours_01b_rappels_no_delay/01_delai_sans_delay/#d%C3%A9lai-sans-delay), [anneau RGB](https://nbourre.github.io/1sx_cours_notes/1SX_cours_04_dels_moteurs/#lumi%C3%A8re-avant-tout)
- [Gyroscope et correction PD](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_straight/ranger_straight.ino), [conversion distance/rotation](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_avance_precis/ranger_avance_precis.ino)
- [Pivot avec les encodeurs : spin()](https://github.com/nbourre/1SX_robotique/blob/0941ad14f74b80703fee6ec92e0c7fbae76611d9/cours_08_encodeurs/ranger_tourne_precis/ranger_tourne_precis.ino)
- [Sonar : impulsion et lecture](https://nbourre.github.io/1sx_cours_notes/1SX_cours_02_ultrason_hum_temp/#exemple-de-code)
Adaptations : limites de temps/PD, lecture bornee, rejet des mesures et confirmation d'arret.

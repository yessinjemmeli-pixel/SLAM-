# SLAM - Pôle projet S7 (P19)

Robot omnidirectionnel autonome : lidar RPLidar C1, Raspberry Pi, Arduino Romeo BLE, ROS2 Jazzy.
Ce dépôt contient le code du robot et trois outils développés ce semestre :
cartographie (grille d'occupation), cartographie sur PC, navigation par apprentissage par renforcement.

## Contenu

| Dossier / fichier | Rôle |
|---|---|
| `Navigation-main/robot_ws/src/robot/` | Nœuds ROS2 du robot (lidar, localisation, asservissement, DWA, console) |
| `Navigation-main/robot_ws/src/robot/robot/occupancy_grid.py` | Calcul de la grille d'occupation (numpy pur) |
| `Navigation-main/robot_ws/src/visualization/` | Fenêtres de visualisation (lidar, position, **carte**) |
| `robot-scripts/lidar-test/visualize_lidar.py` | Affichage simple du lidar sur PC |
| `robot-scripts/lidar-test/live_grid.py` | Cartographie en direct sur PC avec le lidar en USB |
| `robot-scripts/start_robot.ps1` / `.bat` | Connexion SSH au robot (lance ROS2 et la console) |
| `rl-nav/rl_nav.py`, `rl-nav/policy.npz` | Navigation départ -> arrivée par RL (TD), politique déjà entraînée |

## Installation sur le PC (Windows, une seule fois)

```
pip install numpy matplotlib pyserial rplidar-roboticia pytest
```

Pour le lidar : installer le pilote CP210x (dossier `robot-scripts/lidar-test/CP210x_Windows_Drivers`),
brancher le lidar, puis relever son port dans le Gestionnaire de périphériques > Ports (COM & LPT), par exemple `COM5`.

## 1. Voir le lidar en direct (PC, sans robot)

```
cd robot-scripts\lidar-test
python visualize_lidar.py
```

Adapter `PORT_NAME` en haut du fichier si le port n'est pas `COM5`. Fermer la fenêtre pour arrêter.

## 2. Cartographier avec le lidar seul (PC, sans robot ni ROS)

```
cd robot-scripts\lidar-test
python live_grid.py --port COM5      # lidar réel
python live_grid.py --sim            # test sans lidar : pièce simulée
python live_grid.py --selftest       # vérification automatique (erreur de position)
```

Dans la fenêtre : touche **S** = sauvegarder la carte (PNG + NPZ dans `maps/`), touche **R** = repartir de zéro.
La position est estimée uniquement avec le lidar : déplacer le lidar **lentement**, dans un lieu avec des murs et des
formes variées (pas un couloir droit ni une grande salle vide). Si le titre affiche « POSITION PERDUE », revenir vers
une zone déjà cartographiée. Le score dans le titre mesure la qualité du recalage (au-dessus de ~1,5 : bon ; sous 0,3 : scan rejeté).

## 3. Cartographie sur le robot (ROS2)

Sur le Pi (SSH), après chaque `git pull` / changement de code :

```
cd ~/Navigation/robot_ws
colcon build --packages-select robot visualization
source install/setup.bash
ros2 launch robot robot_launch.py
```

Deuxième terminal (console de commande) :

```
cd ~/Navigation/robot_ws && source install/setup.bash
ros2 run robot console
```

Commandes utiles de la console : `savemap` (sauvegarde la carte en PNG + NPZ dans `~/robot_maps` sur le Pi),
`resetpos` (remet la position et la carte à zéro), `help`.

Sur une machine avec écran, sur le même réseau que le Pi (même `ROS_DOMAIN_ID`) :

```
cd ~/Navigation/robot_ws && source install/setup.bash
ros2 run visualization map
```

Vérifier que la carte est publiée : `ros2 topic hz /robot/map` (environ 1 Hz).
Si le nœud `localization` n'est pas lancé par `robot_launch.py`, le démarrer à part : `ros2 run robot localization`.
Ne pas lancer la fenêtre de carte sur le Pi : le faire sur le PC.

Les réglages de la grille (résolution, taille, vitesse de rotation maximale) sont dans la section
« Grille d'occupation » de `Navigation-main/robot_ws/src/robot/robot/robot_config.py`.
La carte est celle de la session en cours : elle repart de zéro à chaque lancement.

## 4. Navigation par apprentissage par renforcement (simulation)

```
cd rl-nav
python rl_nav.py run --start -2.5 2 --goal 2.5 -2 --obstacle 0 0 0.5 --obstacle 1.2 -1.2 0.4
python rl_nav.py eval                      # taux de réussite sur 300 scènes aléatoires
python rl_nav.py train --updates 4000      # réentraîne (environ 4 min), écrit policy.npz
python rl_nav.py compare --updates 3000    # compare TD(0), TD(lambda), Monte Carlo
python rl_nav.py selftest                  # vérifie les gradients et le lidar simulé
```

`--start X Y`, `--goal X Y`, `--obstacle X Y RAYON` (répétable), arène de 6 x 6 m. Ajouter `--png fichier.png --no-gui`
pour enregistrer l'image au lieu d'ouvrir la fenêtre. `--lam` règle TD(lambda) : 0 = TD(0), 1 = proche de Monte Carlo.
Ce programme tourne uniquement en simulation : il n'est pas encore branché sur ROS.

## Tests

```
cd Navigation-main\robot_ws\src\robot
python -m pytest test\test_occupancy_grid.py -q      # attendu : 8 passed
```

## Git : travail en équipe

```
git checkout -b ma-branche          # créer sa branche
git add <fichiers>                  # ajouter des fichiers précis
git commit -m "message"
git push -u origin ma-branche       # premier push d'une branche
git pull                            # récupérer le travail des autres
```

Ne pas committer les dossiers générés (`maps/`, `build/`, `install/`, `log/`, `__pycache__/`) ni de fichiers volumineux.

## Limites connues

- La grille n'est pas encore utilisée par l'évitement d'obstacles (DWA) : il ne voit que le lidar courant.
- Pas de fermeture de boucle : sur un long trajet, la carte dérive avec l'ICP.
- Pas de correction du scan pendant les rotations rapides : tourner doucement (les scans trop déformés sont ignorés).
- Le code RL n'a été testé qu'en simulation 2D à obstacles ronds.

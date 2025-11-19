# Simulation de robot aspirateur autonome

Ce dossier contient le modèle MATLAB demandé pour simuler le robot aspirateur
(unicycle, deux moteurs indépendants, capteur ultrason frontal et odométrie
parfaite).

## Contenu

- `robot_specification.md` : cahier des charges fourni.
- `main_simulation.m` : script à lancer pour reproduire les résultats.
- `default_simulation_params.m` : création centralisée des paramètres.
- `run_cleaning_simulation.m` : coeur de la simulation (cinématique, moteurs,
  capteur, grille d'occupation, stratégie de contrôle).
- `plot_simulation_outputs.m` : génération des figures (courbe de couverture et
  carte + trajectoire).

## Utilisation

1. Ouvrir MATLAB (ou Octave pour la partie script).
2. Se placer dans le dossier contenant les fichiers et exécuter :
   ```matlab
   >> main_simulation
   ```
3. Les figures suivantes sont générées :
   - pourcentage de surface nettoyée en fonction du temps ;
   - carte de la pièce avec obstacle, cellules visitées et trajectoire.
4. La console affiche la couverture finale et le temps pour atteindre 70 %.

Les paramètres principaux (taille de salle, durée de simulation, gains moteurs,
écart des bandes de balayage, etc.) se modifient directement via la structure
retournée par `default_simulation_params`.

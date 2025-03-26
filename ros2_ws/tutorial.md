# Tutoriel d'Installation et d'Exécution - Test de Communication ROS2

## 📋 Prérequis

- WSL2 avec Ubuntu 24.04
- ROS2 Jazzy installé
- Environnement Python 3 (recommandé: virtualenv)

## 🛠️ Configuration pas à pas

### 1. Création de l'espace de travail

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Création du package

```bash
ros2 pkg create --build-type ament_python comm_test
cd comm_test
```

### 3. Structure des fichiers

Créez cette structure :

```bash
comm_test/
├── comm_test/
│   ├── __init__.py
│   ├── talker.py
│   └── listener.py
├── package.xml
└── setup.py
```

### 4. Configuration des fichiers

**package.xml** (ajoutez ces dépendances) :

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

**setup.py** (configurez les entry_points) :

```python
entry_points={
    'console_scripts': [
        'talker = comm_test.talker:main',
        'listener = comm_test.listener:main',
    ],
},
```

### 5. Installation des dépendances

```bash
sudo apt update
sudo apt install python3-pip
pip install statistics
```

## 🚀 Exécution du projet

### 1. Compilation et installation

```bash
cd ~/ros2_ws
colcon build --packages-select comm_test
source install/setup.bash
```

### 2. Lancement des nodes

**Terminal 1 - Publisher** :

```bash
ros2 run comm_test talker
```

**Terminal 2 - Subscriber** :

```bash
ros2 run comm_test listener
```

### 3. Vérification

Ouvrez un 3ème terminal pour surveiller :

```bash
ros2 topic list
ros2 topic hz /topic
ros2 topic echo /topic
```

## 🔧 Dépannage

### Si les nodes ne communiquent pas

1. Vérifiez que les deux terminaux ont bien sourcé ROS2 :

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Vérifiez les permissions :

   ```bash
   chmod +x ~/ros2_ws/src/comm_test/comm_test/*.py
   ```

3. Recompilez proprement :

   ```bash
   cd ~/ros2_ws
   rm -rf build install log
   colcon build --packages-select comm_test
   ```

## 📈 Test de Performance

Après 1000 messages envoyés, vous devriez voir :

**Sortie du Talker** :

```bash
--- Statistics ---
Total messages sent: 1000
Total time: 99.901 seconds
Mean duration: 0.100001 sec
Frequency: 10.00 Hz
```

**Sortie du Listener** :

```bash
--- Listener Statistics ---
Total messages received: 1000
Total time: 99.902 seconds
Mean duration: 0.100002 sec
```

## 💡 Conseils

- Modifiez `timer_period` dans talker.py pour changer la fréquence
- Ajoutez `--qos-profile reliability=reliable` pour une meilleure stabilité
- Utilisez `ros2 topic bw /topic` pour mesurer la bande passante

Ce tutoriel comprend :

1. Les étapes d'installation complètes
2. La configuration détaillée
3. Les commandes d'exécution
4. Les procédures de dépannage
5. Les résultats attendus
6. Des conseils d'optimisation

Pour une utilisation optimale :

1. Copiez ce contenu dans `README.md`
2. Adaptez les chemins si nécessaire
3. Conservez-le avec votre code pour référence future

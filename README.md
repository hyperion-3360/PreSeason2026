# 🤖 FRC PreSeason2026 – Équipe Hyperion 3360

Projet de **pré-saison 2026** de l’équipe **FRC Hyperion 3360**.  
Ce dépôt sert à préparer la saison officielle de la compétition **FIRST Robotics Competition (FRC)** à travers des expérimentations, des tests et de la formation technique.

---

## 🎯 Objectifs du projet

- 🧠 Former les nouveaux membres aux bases de la programmation robotique FRC.  
- ⚙️ Tester de nouvelles approches logicielles (autonomie, vision, contrôle moteur, etc.).  
- 🧩 Développer une base de code réutilisable pour la saison officielle 2026.  
- 🔍 Expérimenter les bonnes pratiques GitHub (Pull Requests, Reviews, Branches protégées).  
- 🤝 Favoriser le travail d’équipe et la collaboration en développement.

---

## 🧰 Technologies et outils utilisés

- **WPILib** – Bibliothèque FRC pour la programmation robotique  
- **Java** – Langage principal du projet *(modifiable selon ton choix)*  
- **GradleRIO** – Outil de build et de déploiement du code sur le robot  
- **Git & GitHub** – Gestion de version, collaboration et revue de code  

---

## 🧑‍💻 Structure du projet

src/
└─ main/
   └─ java/frc/robot/
      ├─ Robot.java                    # Cycle WPILib (init/periodic/disabled/autonomous/teleop)
      ├─ RobotContainer.java           # Déclarations des commandes, bindings manette, auto chooser
      ├─ RobotConfig.java              # Switch WCP ↔ SDS MK4i (IDs/ratios/inversions)
      └─ generated/TunerConstants.java # Généré par Tuner X (patché pour lire RobotConfig)
vendordeps/                            # Dépendances (ex. Phoenix 6)
build.gradle / gradlew*                # Build & déploiement
README.md


# Guide SysId - Caractérisation du Swerve Drive

Ce guide explique comment exécuter SysId sur votre robot pour caractériser le swerve drive et optimiser les gains PID/FF.

## Table des matières
1. [Qu'est-ce que SysId?](#quest-ce-que-sysid)
2. [Préparation](#préparation)
3. [Exécution des tests](#exécution-des-tests)
4. [Analyse des résultats](#analyse-des-résultats)
5. [Application des gains](#application-des-gains)
6. [Vérification](#vérification)

---

## Qu'est-ce que SysId?

SysId (System Identification) est un outil WPILib qui mesure les caractéristiques physiques de votre robot:
- **kS** (Static friction) - Voltage minimum pour vaincre la friction statique
- **kV** (Velocity gain) - Voltage nécessaire pour maintenir une vitesse donnée
- **kA** (Acceleration gain) - Voltage nécessaire pour accélérer à un taux donné
- **kP** (Proportional gain) - Gain proportionnel pour le contrôle PID

Ces gains permettent au robot de suivre précisément les trajectoires PathPlanner et d'améliorer le contrôle.

---

## Préparation

### 1. Équipement requis
- Robot complètement assemblé et fonctionnel
- Batterie pleine (>12.0V recommandé)
- Espace dégagé d'au moins 4-5 mètres en ligne droite
- Laptop avec AdvantageKit/NetworkTables connecté au robot

### 2. Vérifications de sécurité
- ✅ Batterie bien fixée et connectée
- ✅ Bumpers installés
- ✅ Roues swerve libres de tourner (rien ne bloque)
- ✅ Espace dégagé sans obstacles
- ✅ Driver prêt avec manette en main

### 3. Choisir la routine à tester

Votre robot supporte 4 routines SysId différentes. Utilisez **Back + B** pour cycler entre elles:

| Routine | Description | Usage |
|---------|-------------|-------|
| **Translation** | Caractérise les moteurs de drive (translation X/Y) | Optimiser vitesse linéaire, trajectoires PathPlanner |
| **Steer** | Caractérise les moteurs de rotation (azimuth) | Optimiser rotation des modules swerve |
| **Rotation** | Caractérise la rotation du robot entier | Optimiser rotation sur place, heading hold |
| **TranslationCoupled** | Caractérise translation avec couplage angulaire | Avancé - trajectoires complexes |

**Recommandation pour débutants**: Commencez avec **Translation** et **Rotation**.

---

## Exécution des tests

### Étape 1: Sélectionner la routine

1. Connectez-vous au robot via WiFi/USB
2. **Désactivez le robot** (Disabled mode dans Driver Station)
3. Appuyez sur **Back + B** sur la manette pour cycler entre les routines
4. Vérifiez dans la console/logs quelle routine est active:
   ```
   [SysId] Switched to Translation routine
   ```
5. Ou vérifiez dans AdvantageKit: `SysId/CurrentRoutine`

### Étape 2: Exécuter les 4 tests

Pour chaque routine, vous devez exécuter **4 tests** dans cet ordre:

#### Test 1: Quasistatic Forward (Avant lent)
- **Commande**: **Start + Y** (maintenir enfoncé)
- **Comportement**: Le robot accélère **très lentement** vers l'avant
- **Durée**: Maintenez jusqu'à ce que le robot atteigne sa vitesse max (~3-5 secondes)
- **But**: Mesurer kS (friction statique) et kV (voltage/vitesse)

#### Test 2: Quasistatic Reverse (Arrière lent)
- **Commande**: **Start + X** (maintenir enfoncé)
- **Comportement**: Le robot accélère très lentement vers l'arrière
- **Durée**: Maintenez jusqu'à ce que le robot atteigne sa vitesse max (~3-5 secondes)
- **But**: Mesurer kS et kV en sens inverse (vérifier symétrie)

#### Test 3: Dynamic Forward (Avant rapide)
- **Commande**: **Back + Y** (maintenir enfoncé)
- **Comportement**: Le robot accélère **rapidement** vers l'avant
- **Durée**: ~1-2 secondes maximum
- **But**: Mesurer kA (voltage/accélération)

#### Test 4: Dynamic Reverse (Arrière rapide)
- **Commande**: **Back + X** (maintenir enfoncé)
- **Comportement**: Le robot accélère rapidement vers l'arrière
- **Durée**: ~1-2 secondes maximum
- **But**: Mesurer kA en sens inverse

### Résumé des commandes:

| Test | Boutons | Type | Direction |
|------|---------|------|-----------|
| Quasistatic Forward | **Start + Y** | Lent | Avant |
| Quasistatic Reverse | **Start + X** | Lent | Arrière |
| Dynamic Forward | **Back + Y** | Rapide | Avant |
| Dynamic Reverse | **Back + X** | Rapide | Arrière |

### Conseils d'exécution:
- ⚠️ **IMPORTANT**: Le robot doit être en mode **Enabled** (pas Disabled) pour exécuter les tests
- 🔋 Vérifiez la batterie entre chaque test (>11.5V minimum)
- 📏 Assurez 4-5 mètres d'espace libre dans la direction du mouvement
- 🎮 Relâchez les boutons pour arrêter le test immédiatement
- 🔁 Attendez 2-3 secondes entre chaque test pour laisser le robot s'arrêter complètement
- 📊 Les données sont automatiquement loggées dans AdvantageKit

---

## Analyse des résultats

### Étape 1: Récupérer les logs AdvantageKit

1. Après tous les tests, **désactivez le robot**
2. Les logs sont dans: `/home/lvuser/logs/` sur le roboRIO
3. Copiez le fichier `.wpilog` le plus récent sur votre laptop
4. Ou utilisez AdvantageScope pour accéder aux logs via réseau

### Étape 2: Analyser avec SysId Analyzer

1. Ouvrez **WPILib SysId** (Tool dans VS Code ou application standalone)
2. Cliquez sur **Load Data**
3. Sélectionnez votre fichier `.wpilog`
4. **IMPORTANT**: Choisissez le bon type de mécanisme:
   - **Translation routine** → "Drivetrain" (units: meters)
   - **Rotation routine** → "Drivetrain (Angular)" (units: radians)
   - **Steer routine** → "Simple Motor" (units: radians)

5. Cliquez sur **Analyze Data**

### Étape 3: Vérifier la qualité des données

SysId affiche des graphiques. Vérifiez:

✅ **Bon test**:
- Courbes lisses sans pics/sauts
- Vitesse augmente progressivement (quasistatic)
- Accélération rapide et constante (dynamic)
- R² > 0.95 (coefficient de corrélation)

❌ **Mauvais test** (refaire):
- Courbes en dents de scie
- Robot n'a pas bougé
- Batterie trop faible (<11V)
- R² < 0.90

### Étape 4: Noter les gains

SysId calcule automatiquement les gains. **Notez ces valeurs**:

**Exemple pour Translation routine**:
```
kS = 0.12345 V
kV = 2.3456 V/(m/s)
kA = 0.34567 V/(m/s²)
kP = 1.2345 V/m
```

**Exemple pour Rotation routine**:
```
kS = 0.15678 V
kV = 1.9876 V/(rad/s)
kA = 0.45678 V/(rad/s²)
kP = 2.3456 V/rad
```

---

## Application des gains

### Pour Translation et TranslationCoupled

Les gains vont dans **TunerConstants.java**:

```java
// Fichier: src/main/java/frc/robot/generated/TunerConstants.java

// Cherchez la section "Slip Current"
private static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(100).withKI(0).withKD(0.2)
    .withKS(0).withKV(1.5).withKA(0);  // ← NE PAS MODIFIER (steer)

private static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(3)        // ← Remplacer par kP de SysId
    .withKI(0)
    .withKD(0)
    .withKS(0.12345)  // ← Remplacer par kS de SysId (Translation)
    .withKV(2.3456)   // ← Remplacer par kV de SysId (Translation)
    .withKA(0.34567); // ← Remplacer par kA de SysId (Translation)
```

### Pour Steer routine

Les gains vont aussi dans **TunerConstants.java**:

```java
private static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(100)      // ← Remplacer par kP de SysId (Steer)
    .withKI(0)
    .withKD(0.2)      // ← Peut ajuster si oscillations
    .withKS(0.15678)  // ← Remplacer par kS de SysId (Steer)
    .withKV(1.9876)   // ← Remplacer par kV de SysId (Steer)
    .withKA(0.45678); // ← Remplacer par kA de SysId (Steer)

private static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(3).withKI(0).withKD(0)
    .withKS(0).withKV(1.5).withKA(0);  // ← NE PAS MODIFIER (drive)
```

### Pour Rotation routine

Les gains de rotation robot vont dans **TunerConstants.java** à un endroit différent:

```java
// Cherchez "HeadingController" ou "RotationPID"
private static final Slot0Configs rotationGains = new Slot0Configs()
    .withKP(2.3456)   // ← Remplacer par kP de SysId (Rotation)
    .withKI(0)
    .withKD(0)
    .withKS(0.15678)  // ← Remplacer par kS de SysId (Rotation)
    .withKV(1.9876)   // ← Remplacer par kV de SysId (Rotation)
    .withKA(0.45678); // ← Remplacer par kA de SysId (Rotation)
```

**Note**: Si vous ne trouvez pas `rotationGains` dans TunerConstants.java, vérifiez **Constants.java** dans la section `AutoAlignConstants` ou cherchez les gains PID pour la rotation.

### ⚠️ IMPORTANT: Procédure de modification

1. **Sauvegardez** les anciennes valeurs (commentez-les):
   ```java
   // OLD VALUES (before SysId 2025-01-15):
   // .withKS(0).withKV(1.5).withKA(0)
   // NEW VALUES (SysId 2025-01-15):
   .withKS(0.12345).withKV(2.3456).withKA(0.34567)
   ```

2. **Modifiez UNE routine à la fois** (Translation OU Steer, pas les deux en même temps)

3. **Testez** après chaque modification

4. **Re-déployez** le code sur le robot:
   ```bash
   ./gradlew deploy
   ```

---

## Vérification

### Test 1: Trajectoire simple

Créez une trajectoire PathPlanner simple (ligne droite 2m):

1. Exécutez la trajectoire en auto
2. **Vérifiez dans AdvantageScope**:
   - Pose estimée vs. trajectory desired (doivent être proches)
   - Erreur de suivi < 5cm tout au long du parcours
   - Pas d'oscillations ou de dépassements

### Test 2: Rotation sur place

Testez la rotation:

1. Mode teleop, tournez le robot avec le joystick droit
2. **Vérifiez**:
   - Rotation fluide sans à-coups
   - Pas d'oscillations autour de l'angle cible
   - Le robot s'arrête précisément où demandé

### Test 3: Conduite normale

Conduisez normalement en teleop:

✅ **Bon tuning**:
- Réponse rapide aux commandes joystick
- Pas d'oscillations ou vibrations
- Robot suit précisément les trajectoires

❌ **Mauvais tuning** (ajuster):
- Robot oscille (kP trop élevé → réduire de 20%)
- Robot lent/mou (kP trop faible → augmenter de 20%)
- Robot dépasse la cible (kD trop faible → augmenter)
- Robot ne suit pas les trajectoires (refaire SysId)

### Ajustements fins

Si les gains SysId ne sont pas parfaits:

1. **kP trop élevé** → Oscillations/vibrations
   - Réduisez kP de 10-20% jusqu'à disparition

2. **kP trop faible** → Robot lent, erreur de suivi élevée
   - Augmentez kP de 10-20% jusqu'à réponse rapide

3. **kD pour amortissement** → Dépassements/oscillations
   - Commencez avec kD = 0
   - Augmentez progressivement (0.05, 0.1, 0.2) si oscillations persistent

4. **kI pour erreur steady-state** → Erreur constante qui ne se corrige pas
   - Généralement pas nécessaire pour swerve
   - Si vraiment nécessaire, commencez TRÈS petit (0.001)

---

## Résumé - Checklist rapide

### Avant SysId:
- [ ] Batterie >12.0V
- [ ] Espace dégagé 4-5m
- [ ] Robot en mode Enabled
- [ ] Routine sélectionnée (Back + B)

### Pendant SysId:
- [ ] Quasistatic Forward (Start + Y)
- [ ] Quasistatic Reverse (Start + X)
- [ ] Dynamic Forward (Back + Y)
- [ ] Dynamic Reverse (Back + X)

### Après SysId:
- [ ] Récupérer logs AdvantageKit
- [ ] Analyser avec SysId Analyzer (R² > 0.95)
- [ ] Noter les gains (kS, kV, kA, kP)
- [ ] Modifier TunerConstants.java
- [ ] Déployer et tester
- [ ] Vérifier trajectoires et rotation

---

## Troubleshooting

### Le robot ne bouge pas pendant les tests
- Vérifiez que le robot est en mode **Enabled** (pas Disabled)
- Vérifiez la batterie (>11.5V minimum)
- Vérifiez les freins de moteur ne sont pas activés

### R² très faible (<0.90) dans SysId Analyzer
- Batterie trop faible → rechargez et refaites les tests
- Tests trop courts → maintenez les boutons plus longtemps
- Surface glissante → testez sur tapis/carpet
- Problème mécanique → vérifiez courroies, roues, moteurs

### Robot oscille après application des gains
- kP trop élevé → réduisez de 20%
- Ajoutez du kD pour amortissement (commencez à 0.1)

### Robot ne suit pas les trajectoires PathPlanner
- Vérifiez que vous avez modifié `driveGains` (pas `steerGains`)
- Vérifiez les unités (mètres pour translation, radians pour rotation)
- Re-vérifiez les valeurs copiées depuis SysId (pas d'erreur de frappe)

### Logs AdvantageKit vides ou corrompus
- Vérifiez l'espace disque sur roboRIO (`df -h`)
- Vérifiez que AdvantageKit est bien initialisé dans Robot.java
- Redémarrez le roboRIO et refaites les tests

---

## Références

- [WPILib SysId Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html)
- [CTRE Phoenix 6 Swerve Tuning](https://pro.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html)
- [AdvantageKit Logging](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/START-LOGGING.md)

---

**Dernière mise à jour**: 2025-01-15
**Testé sur**: PreSeason2026 Robot (CTRE Phoenix 6 Swerve)
# 🤖 Robot Orchestrator - Master State Machine

Guide complet pour coordonner Vision + Drivetrain + GamePiece avec inputs joueur.

---

## 🏗️ Architecture Complète

```
┌────────────────────────────────────────────────────────────┐
│                  ROBOT ORCHESTRATOR                         │
│              (Master State Machine)                         │
│                                                             │
│  Inputs:                           Outputs:                │
│  • Player buttons                  • Commands subsystems   │
│  • Vision data                     • Coordinates actions   │
│  • Sensor data                     • Safety overrides      │
│  • GamePiece state                                         │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │   Vision     │  │  Drivetrain  │  │  GamePiece   │    │
│  │  Subsystem   │  │  Subsystem   │  │   Manager    │    │
│  │              │  │              │  │              │    │
│  │ • AprilTags  │  │ • Swerve     │  │ • Intake     │    │
│  │ • Target     │  │ • Auto-align │  │ • Sensors    │    │
│  │   selection  │  │ • Field-     │  │ • State      │    │
│  │              │  │   centric    │  │   machine    │    │
│  └──────────────┘  └──────────────┘  └──────────────┘    │
└────────────────────────────────────────────────────────────┘
```

---

## 📊 Diagramme d'États Master

```
                    ┌─────────────────┐
                    │ TELEOP_MANUAL   │◄──────────┐
                    └────────┬────────┘           │
                             │                    │
                    ┌────────┴────────┐           │
                    │                 │           │
          Request   │                 │ Request   │
          Intake    ▼                 ▼ Score     │
              ┌──────────┐      ┌──────────┐     │
              │SEARCHING │      │ HOLDING  │     │
              │FOR_PIECE │─────►│  PIECE   │     │
              └────┬─────┘      └────┬─────┘     │
                   │                 │           │
        Vision     │                 │           │
        Target     ▼                 ▼           │
              ┌──────────┐      ┌──────────┐     │
              │AUTO_ALIGN│      │AUTO_ALIGN│     │
              │TO_PIECE  │      │TO_SCORE  │     │
              └────┬─────┘      └────┬─────┘     │
                   │                 │           │
        Aligned    │                 │Aligned    │
                   ▼                 ▼           │
              ┌──────────┐      ┌──────────┐     │
              │INTAKING  │      │PREPARING │     │
              │  PIECE   │      │TO_SCORE  │     │
              └────┬─────┘      └────┬─────┘     │
                   │                 │           │
        Got Piece  │                 │Ready      │
                   │                 ▼           │
                   │           ┌──────────┐      │
                   │           │READY_TO  │      │
                   └──────────►│  SCORE   │      │
                               └────┬─────┘      │
                                    │            │
                           Confirm  │            │
                                    ▼            │
                               ┌──────────┐      │
                               │ SCORING  │──────┘
                               └──────────┘
```

---

## 🎮 Intégration RobotContainer

### **Création des Subsystems:**

```java
// Dans RobotContainer.java

public class RobotContainer {
    // Subsystems individuels
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    private final GamePieceManager gamePiece = new GamePieceManager();

    // MASTER ORCHESTRATOR
    private final RobotOrchestrator orchestrator =
        new RobotOrchestrator(drivetrain, vision, gamePiece);

    // Controller
    private final CommandXboxController joystick =
        new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }
}
```

### **Configuration des Controls:**

```java
private void configureBindings() {
    // ========== INTAKE WORKFLOW ==========

    // Right Trigger = Intake (simple hold)
    joystick.rightTrigger()
        .whileTrue(Commands.run(() -> orchestrator.requestIntake(true)))
        .onFalse(Commands.runOnce(() -> orchestrator.requestIntake(false)));

    // Right Bumper = Auto-align to game piece (hold while moving)
    joystick.rightBumper()
        .whileTrue(Commands.run(() -> orchestrator.requestAutoAlignToPiece(true)))
        .onFalse(Commands.runOnce(() -> orchestrator.requestAutoAlignToPiece(false)));

    // ========== SCORING WORKFLOW ==========

    // Left Trigger = Score (2-stage: prepare → confirm)
    joystick.leftTrigger()
        .whileTrue(Commands.run(() -> orchestrator.requestScore(true)))
        .onFalse(Commands.runOnce(() -> orchestrator.requestScore(false)));

    // Left Bumper = Auto-align to scoring position
    joystick.leftBumper()
        .whileTrue(Commands.run(() -> orchestrator.requestAutoAlignToScore(true)))
        .onFalse(Commands.runOnce(() -> orchestrator.requestAutoAlignToScore(false)));

    // ========== UTILITY ==========

    // X = Eject game piece
    joystick.x()
        .whileTrue(Commands.run(() -> orchestrator.requestEject(true)))
        .onFalse(Commands.runOnce(() -> orchestrator.requestEject(false)));

    // Back = Manual override (return to teleop)
    joystick.back()
        .onTrue(Commands.runOnce(() -> orchestrator.requestManualOverride()));

    // Start = Emergency stop
    joystick.start()
        .onTrue(Commands.runOnce(() -> orchestrator.emergencyStop()));
}
```

---

## 🔄 Flux de Travail Typiques

### **Scénario 1: Intake Simple (Manuel)**

```
1. Driver voit game piece
2. Driver hold Right Trigger
   → TELEOP_MANUAL → SEARCHING_FOR_PIECE
3. Robot intake running, cherche piece
4. Beam break détecte piece
   → SEARCHING_FOR_PIECE → INTAKING_PIECE
5. Current sensor confirme acquisition
   → INTAKING_PIECE → HOLDING_PIECE
6. Driver release Right Trigger
   → HOLDING_PIECE (reste dans cet état)
```

### **Scénario 2: Intake avec Vision Auto-Align**

```
1. Driver voit game piece au loin
2. Driver hold Right Trigger + Right Bumper
   → TELEOP_MANUAL → SEARCHING_FOR_PIECE → AUTO_ALIGNING_TO_PIECE
3. Vision détecte piece, robot s'aligne auto
   - Driver contrôle translation (avant/arrière/gauche/droite)
   - Robot contrôle rotation (face la piece)
4. Une fois aligné et proche
   → AUTO_ALIGNING_TO_PIECE → INTAKING_PIECE
5. Piece acquise
   → INTAKING_PIECE → HOLDING_PIECE
```

### **Scénario 3: Scoring avec Auto-Align**

```
1. Robot has piece (HOLDING_PIECE)
2. Driver hold Left Bumper (auto-align)
   → HOLDING_PIECE → AUTO_ALIGNING_TO_SCORE
3. Vision détecte AprilTag, robot s'aligne
   - AlignToTagCommand running
   - Driver peut override si besoin
4. Aligné à 1m du tag
   → AUTO_ALIGNING_TO_SCORE → PREPARING_TO_SCORE
5. Arm/elevator montent en position
   → PREPARING_TO_SCORE → READY_TO_SCORE
6. Controller vibre, LED verte clignote
7. Driver hold Left Trigger (confirme)
   → READY_TO_SCORE → SCORING
8. Piece relâchée
   → SCORING → TELEOP_MANUAL
```

### **Scénario 4: Emergency Override**

```
Peu importe l'état actuel:
1. Driver press Back button
   → * → TELEOP_MANUAL
2. Tous les subsystems relâchés
3. Driver reprend contrôle manuel complet
```

---

## 🔌 Coordination des Subsystems

### **TELEOP_MANUAL:**
```java
// Drivetrain: Full player control
// Vision: Auto-aim peut être actif si toggled
// GamePiece: Idle
```

### **SEARCHING_FOR_PIECE:**
```java
// Drivetrain: Player control
// Vision: Looking for game pieces (pas AprilTags)
// GamePiece: Intake running
```

### **AUTO_ALIGNING_TO_PIECE:**
```java
// Drivetrain:
//   - Translation: Player control
//   - Rotation: Vision control (face piece)
// Vision: Tracking game piece
// GamePiece: Intake ready
```

### **INTAKING_PIECE:**
```java
// Drivetrain: Player should stop (but can override)
// Vision: Disabled
// GamePiece: Intake running at full power
```

### **HOLDING_PIECE:**
```java
// Drivetrain: Normal player control (maybe speed limited)
// Vision: Can be used for other tasks
// GamePiece: Holding piece, intake stopped
```

### **AUTO_ALIGNING_TO_SCORE:**
```java
// Drivetrain: AlignToTagCommand running
//   - Full auto-align to AprilTag
// Vision: Tracking AprilTag, target priority active
// GamePiece: Holding piece
```

### **PREPARING_TO_SCORE:**
```java
// Drivetrain: Should be stationary
// Vision: Maintaining lock on target
// GamePiece: Moving mechanisms to score position
```

### **READY_TO_SCORE:**
```java
// Drivetrain: Locked in position
// Vision: Locked on target
// GamePiece: Ready, waiting for confirmation
// Outputs: Rumble controller, LED green blink
```

### **SCORING:**
```java
// Drivetrain: Stationary
// Vision: Can release
// GamePiece: Releasing piece (reverse intake / open claw)
```

---

## 🎯 Avantages de cette Architecture

### ✅ **Clarté:**
- Un seul endroit décide ce que fait le robot
- Facile de voir l'état actuel
- Debugging simple

### ✅ **Sécurité:**
- Transitions validées
- Override manuel toujours possible
- Emergency stop accessible

### ✅ **Flexibilité:**
- Player peut interrompre n'importe quand
- Peut mixer auto et manuel
- Graceful degradation si vision perdue

### ✅ **Simplicité pour le Driver:**
- Hold button = tout se fait automatiquement
- Feedback clair (rumble, LED, console)
- Pas besoin de micro-gérer

---

## 🔧 Extensions Possibles

### **1. Multiple Game Piece Types:**
```java
enum PieceType { CONE, CUBE }
private PieceType m_desiredPieceType;

// Dans SEARCHING_FOR_PIECE:
// Vision filtre par type de piece
// Auto-eject si mauvais type acquis
```

### **2. Auto Sequences:**
```java
// Hold A + Left Bumper = Full auto score
RobotState.AUTO_SCORE_SEQUENCE
  → Auto-align
  → Prepare
  → Score
  → Return to intake
```

### **3. Smart Vision Switching:**
```java
// Orchestrator change target priority selon state:
// SEARCHING_FOR_PIECE → CLOSEST game piece
// AUTO_ALIGNING_TO_SCORE → ALLIANCE_ONLY tags
```

### **4. Field Awareness:**
```java
// Connaît position sur terrain
// Auto-sélectionne bon scoring node
// Évite zones hors-limites
```

---

## 📝 Checklist d'Implémentation

- [ ] Créer RobotOrchestrator dans RobotContainer
- [ ] Passer subsystems au constructeur
- [ ] Mapper tous les buttons dans configureBindings()
- [ ] Implémenter `m_isAligned` check (AlignToTagCommand status)
- [ ] Connecter rumble controller dans READY_TO_SCORE
- [ ] Tester chaque transition individuellement
- [ ] Ajouter LED feedback pour chaque état
- [ ] Logger toutes transitions pour replay
- [ ] Tester override/emergency stop
- [ ] Valider avec drivers

---

## 🎓 Ce que les Jeunes Apprennent

1. **Architecture logicielle hiérarchique**
2. **Coordination de systèmes complexes**
3. **State machines imbriquées**
4. **Priorités et interruptions**
5. **User experience design**
6. **Safety-critical programming**

---

**Créé pour:** FRC Team 3360 Hyperion
**Projet:** PreSeason 2026
**Architecture:** Master State Machine Orchestrator

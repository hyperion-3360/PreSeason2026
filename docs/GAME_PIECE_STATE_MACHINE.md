# 🎮 Game Piece State Machine

Système de gestion automatisée de la manipulation des game pieces avec state machine.

---

## 📊 Diagramme d'États

```
                    ┌─────────────┐
                    │    IDLE     │◄──────────────┐
                    └──────┬──────┘               │
                           │ Request Intake       │
                           ▼                      │
                    ┌─────────────┐               │
                    │  SEARCHING  │               │
                    └──────┬──────┘               │
                           │ Piece Detected       │
                           ▼                      │
                    ┌─────────────┐               │
                    │  ACQUIRING  │               │
                    └──────┬──────┘               │
                           │ Acquired             │
                           ▼                      │
                    ┌─────────────┐               │
              ┌────►│   HOLDING   │◄────┐         │
              │     └──────┬──────┘     │         │
              │            │ Request    │         │
     Cancel   │            │ Score      │ Cancel  │
              │            ▼            │         │
              │     ┌─────────────┐     │         │
              │     │ PREPARING   │─────┘         │
              │     └──────┬──────┘               │
              │            │ In Position          │
              │            ▼                      │
              │     ┌─────────────┐               │
              └─────│READY_TO_SCORE│              │
                    └──────┬──────┘               │
                           │ Confirm              │
                           ▼                      │
                    ┌─────────────┐               │
                    │   SCORING   │───────────────┘
                    └─────────────┘
```

---

## 🎯 États et Transitions

### **IDLE** - État de repos
- **Aucune game piece dans le robot**
- Tous les mécanismes à l'arrêt
- **Transitions:**
  - → SEARCHING si joueur demande intake
  - → HOLDING si piece détectée (récupérée sans intake actif)

### **SEARCHING** - Recherche active
- **Intake en marche, cherche game piece**
- Capteurs actifs (beam break, current)
- **Transitions:**
  - → ACQUIRING si piece détectée
  - → IDLE si annulé

### **ACQUIRING** - Acquisition en cours
- **Game piece détectée, en train d'être capturée**
- Intake à pleine puissance
- Timeout de sécurité: 2 secondes
- **Transitions:**
  - → HOLDING si piece sécurisée
  - → IDLE si timeout
  - → EJECTING si annulé

### **HOLDING** - Piece sécurisée
- **Game piece dans le robot**
- Intake arrêté
- LED allumée (vert = prêt)
- **Transitions:**
  - → PREPARING si score demandé
  - → EJECTING si éjection demandée
  - → ERROR si piece perdue

### **PREPARING_TO_SCORE** - Préparation
- **Déplace arm/elevator en position de score**
- Vérifie que tout est aligné
- **Transitions:**
  - → READY_TO_SCORE si en position
  - → HOLDING si annulé

### **READY_TO_SCORE** - Prêt à scorer
- **Tout en position, attend confirmation**
- LED clignotante (vert)
- Vibration controller
- **Transitions:**
  - → SCORING si confirmé
  - → HOLDING si annulé

### **SCORING** - Score en cours
- **Relâche la game piece**
- Reverse intake / ouvre claw
- Timeout: 1 seconde
- **Transitions:**
  - → IDLE quand piece relâchée
  - → IDLE si timeout

### **EJECTING** - Éjection
- **Éjecte piece non désirée**
- Intake en reverse
- Timeout: 0.5 secondes
- **Transitions:**
  - → IDLE quand piece éjectée

### **ERROR** - État d'erreur
- **Détection de conflit de capteurs**
- Tous mécanismes arrêtés
- Nécessite reset manuel
- **Transitions:**
  - → IDLE si reset ou capteurs normalisés

---

## 🔧 Configuration des Capteurs

### **Capteurs Recommandés:**

1. **Beam Break Sensor** (Infrarouge)
   - Position: À l'entrée de l'intake
   - Détecte présence de game piece
   - Simple et fiable

2. **Current Sensor** (Intégré TalonFX)
   - Détecte charge sur moteur intake
   - Spike de courant = piece en cours d'acquisition
   - Threshold typique: 20-30A

3. **Encoder** (Arm/Elevator)
   - Position des mécanismes
   - Vérifie position de score

4. **Limit Switches** (Optionnel)
   - Positions extrêmes
   - Sécurité mécanique

### **Exemple d'Implémentation Capteurs:**

```java
private void updateSensors() {
    // Beam break - piece présente
    m_hasGamePiece = !m_beamBreak.get(); // Normally Open

    // Current spike - piece en acquisition
    m_intakeCurrentSpike = m_intakeMotor.getOutputCurrent() > 20.0;

    // Mécanismes en position
    m_mechanismsInPosition =
        m_arm.atSetpoint() &&
        m_elevator.atSetpoint();
}
```

---

## 🎮 Mapping des Contrôles

### **Exemple de Controls:**

```java
// Dans RobotContainer.java

// Intake - Hold Right Trigger
joystick.rightTrigger()
    .whileTrue(Commands.run(() -> gamePiece.requestIntake(true)))
    .onFalse(Commands.runOnce(() -> gamePiece.requestIntake(false)));

// Score - Hold Left Trigger (2 stage)
joystick.leftTrigger()
    .whileTrue(Commands.run(() -> gamePiece.requestScore(true)))
    .onFalse(Commands.runOnce(() -> gamePiece.requestScore(false)));

// Eject - Press X
joystick.x()
    .whileTrue(Commands.run(() -> gamePiece.requestEject(true)))
    .onFalse(Commands.runOnce(() -> gamePiece.requestEject(false)));

// Emergency Reset - Back button
joystick.back()
    .onTrue(Commands.runOnce(() -> gamePiece.forceIdle()));
```

---

## 📈 Avantages de la State Machine

### ✅ **Sécurité:**
- Timeouts automatiques
- Détection d'erreurs
- Impossible d'avoir états contradictoires

### ✅ **Simplicité pour le Driver:**
- Juste hold button = tout se fait automatiquement
- Feedback visuel (LEDs) et tactile (rumble)
- 2-stage scoring (prepare → confirm)

### ✅ **Debugging Facile:**
- SmartDashboard montre état actuel
- AdvantageKit log toutes transitions
- Console prints pour debugging

### ✅ **Extensible:**
- Facile d'ajouter nouveaux états
- Peut gérer plusieurs types de game pieces
- Compatible avec vision auto-align

---

## 🚀 Extensions Possibles

### **1. Multi-Piece Support:**
```java
enum PieceType { CONE, CUBE, UNKNOWN }
private PieceType m_currentPieceType;
```

### **2. Vision-Guided Intake:**
- SEARCHING → détecte piece avec vision
- Auto-align vers piece
- Transitions auto quand aligné

### **3. Auto-Score Positions:**
- SmartDashboard chooser: LOW, MID, HIGH
- PREPARING utilise la position choisie
- PID auto vers bonne hauteur

### **4. Smart Eject:**
- Détecte si mauvais type de piece
- Auto-eject si CUBE détecté mais CONE voulu

---

## 📝 TODO pour Implémentation Complète

- [ ] Connecter vrais capteurs (beam break, current)
- [ ] Implémenter contrôle moteur intake
- [ ] Ajouter contrôle LED pour feedback visuel
- [ ] Tester timeouts avec hardware réel
- [ ] Calibrer threshold de current spike
- [ ] Ajouter vibration controller pour READY_TO_SCORE
- [ ] Logger toutes transitions pour replay

---

**Créé pour:** FRC Team 3360 Hyperion
**Projet:** PreSeason 2026
**Auteur:** Équipe programmation

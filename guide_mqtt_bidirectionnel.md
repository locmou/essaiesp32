# Guide : Communication MQTT Bidirectionnelle ESP32 ↔ Home Assistant

## 📋 Ce que fait ce programme

### ESP32 → Home Assistant
- Publie des données toutes les 5 secondes
- Envoie `testValue`, `lastCommand`, `ledState`, `uptime`

### Home Assistant → ESP32
- Reçoit des commandes pour modifier `testValue`
- Affiche les commandes reçues sur le Serial Monitor
- Répond aux commandes en temps réel

## 🚀 Installation

### 1. Téléverser le programme
```
1. Ouvrir le fichier .ino dans Arduino IDE
2. Sélectionner votre carte ESP32
3. Téléverser
4. Ouvrir le Serial Monitor (115200 baud)
```

### 2. Observer le Serial Monitor
Vous devriez voir :
```
╔════════════════════════════════════════╗
║  ESP32 ↔ HOME ASSISTANT EXPÉRIMENTAL  ║
╚════════════════════════════════════════╝

=== Connexion WiFi ===
✓ WiFi connecté !
IP : 192.168.1.XX

Tentative MQTT... ✓ OK !

=== SOUSCRIPTION AUX TOPICS ===
✓ Abonné à : stationair/command
✓ Abonné à : stationair/set_value
```

## 🏠 Configuration dans Home Assistant

### Entités créées automatiquement

L'ESP32 crée automatiquement 5 entités dans HA :

1. **sensor.test_value** (lecture seule)
   - Affiche la valeur actuelle de `testValue`
   - Icône : 🔢 (compteur)

2. **number.set_test_value** (slider 0-100)
   - Permet de modifier `testValue` avec un slider
   - Icône : 🎚️ (tune)

3. **button.reset_value**
   - Remet `testValue` à 0
   - Icône : 🔄 (restore)

4. **button.increment_value**
   - Incrémente `testValue` de +1
   - Icône : ➕

5. **button.decrement_value**
   - Décrémente `testValue` de -1
   - Icône : ➖

### Vérifier les entités

1. Aller dans **Paramètres** → **Appareils et services**
2. Chercher "Station Air Experimental"
3. Vous devriez voir l'appareil avec 5 entités

## 🧪 Tests à effectuer

### Test 1 : Modifier avec le slider

**Dans Home Assistant :**
1. Aller dans l'appareil "Station Air Experimental"
2. Utiliser le slider "Set Test Value"
3. Déplacer vers 42

**Sur le Serial Monitor :**
```
┌─────────────────────────────────
│ MESSAGE REÇU sur topic : stationair/set_value
│ Longueur : 2
│ Contenu : 42
└─────────────────────────────────
➤ MODIFICATION : testValue = 42

╔═══════════════════════════════╗
║ testValue = 42
║ lastCommand = aucune
║ ledState = OFF
╚═══════════════════════════════╝
```

### Test 2 : Bouton Reset

**Dans Home Assistant :**
1. Cliquer sur "Reset Value"

**Sur le Serial Monitor :**
```
┌─────────────────────────────────
│ MESSAGE REÇU sur topic : stationair/command
│ Contenu : reset
└─────────────────────────────────
➤ COMMANDE : Reset de la variable test

╔═══════════════════════════════╗
║ testValue = 0
║ lastCommand = reset
║ ledState = OFF
╚═══════════════════════════════╝
```

### Test 3 : Increment/Decrement

**Dans Home Assistant :**
1. Cliquer plusieurs fois sur "Increment Value"
2. Observer la valeur augmenter

**Sur le Serial Monitor :**
```
➤ COMMANDE : Incrément +1
╔═══════════════════════════════╗
║ testValue = 1
╚═══════════════════════════════╝

➤ COMMANDE : Incrément +1
╔═══════════════════════════════╗
║ testValue = 2
╚═══════════════════════════════╝
```

### Test 4 : Commandes MQTT manuelles

**Via MQTT Explorer ou terminal :**

```bash
# Envoyer une commande
mosquitto_pub -h 192.168.1.11 -u "loic.mounier@laposte.net" -P "vgo:?2258H" \
  -t "stationair/command" -m "increment"

# Définir une valeur
mosquitto_pub -h 192.168.1.11 -u "loic.mounier@laposte.net" -P "vgo:?2258H" \
  -t "stationair/set_value" -m "77"
```

## 📊 Créer un Dashboard dans Home Assistant

**Exemple de carte YAML :**

```yaml
type: vertical-stack
cards:
  - type: entities
    title: Station Air Experimental
    entities:
      - entity: sensor.test_value
        name: Valeur actuelle
      - entity: number.set_test_value
        name: Modifier la valeur
  - type: horizontal-stack
    cards:
      - type: button
        entity: button.reset_value
        name: Reset
        icon: mdi:restore
      - type: button
        entity: button.increment_value
        name: +1
        icon: mdi:plus
      - type: button
        entity: button.decrement_value
        name: -1
        icon: mdi:minus
```

## 🔧 Personnalisation

### Ajouter une nouvelle commande

**1. Dans le code ESP32 :**

```cpp
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // ... code existant ...
  
  if (strcmp(topic, TOPIC_COMMAND) == 0) {
    // Ajouter votre commande
    if (message == "ma_commande") {
      Serial.println("➤ MA COMMANDE personnalisée");
      // Votre code ici
    }
  }
}
```

**2. Créer un bouton dans HA :**

Dans `publish_discovery()`, ajouter :

```cpp
const char* discovery_ma_commande = R"({
"name":"Ma Commande",
"uniq_id":"stationair_ma_commande",
"cmd_t":"stationair/command",
"payload_press":"ma_commande",
"icon":"mdi:flash",
"device":{"ids":["stationair"],"name":"Station Air Experimental","mf":"DIY","mdl":"ESP32"}
})";

client.publish("homeassistant/button/stationair_ma_commande/config", 
               discovery_ma_commande, true);
```

### Ajouter un nouveau topic

```cpp
// 1. Définir le topic
const char* TOPIC_MON_TOPIC = "stationair/mon_topic";

// 2. S'abonner dans reconnect_mqtt()
client.subscribe(TOPIC_MON_TOPIC);

// 3. Traiter dans mqtt_callback()
if (strcmp(topic, TOPIC_MON_TOPIC) == 0) {
  // Votre traitement
}
```

## 🐛 Dépannage

### Aucune entité n'apparaît dans HA

1. Vérifier que MQTT est bien configuré dans HA
2. Redémarrer l'ESP32
3. Vérifier les logs MQTT :
   ```bash
   mosquitto_sub -h 192.168.1.11 -u "user" -P "pass" -t "homeassistant/#" -v
   ```

### Les commandes ne sont pas reçues

1. Vérifier que `client.loop()` est bien appelé dans `loop()`
2. Vérifier les topics dans le Serial Monitor
3. Tester avec mosquitto_pub manuellement

### ESP32 se déconnecte

1. Augmenter `client.setKeepAlive(120)`
2. Vérifier la stabilité WiFi
3. Ajouter des logs dans `reconnect_mqtt()`

## 📈 Prochaines étapes

Une fois ce test fonctionnel, vous pourrez :

1. **Intégrer au programme principal**
   - Ajouter des commandes pour changer le mode d'affichage LCD
   - Activer/désactiver le rétroéclairage
   - Forcer une publication de données
   - Régler les seuils d'alerte CO

2. **Créer des automatisations HA**
   - Allumer le rétroéclairage quand quelqu'un entre
   - Changer l'affichage selon l'heure
   - Alertes sur CO élevé

3. **Contrôler l'affichage**
   - Switch entre température/météo
   - Activer/désactiver les icônes
   - Changer la luminosité LCD

## 📝 Notes

- Le callback `mqtt_callback()` est appelé **automatiquement** quand un message arrive
- `client.loop()` **DOIT** être appelé régulièrement dans `loop()` pour traiter les messages
- Les messages Discovery sont publiés **une seule fois** à la connexion
- Les données sont publiées **toutes les 5 secondes** (modifiable)

Bon test ! 🚀

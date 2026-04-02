# 📊 Guide Mode InfoHA - Station Air ESP32

## 🎯 Vue d'ensemble

Le mode **InfoHA** permet d'afficher sur le LCD 20×4 de votre station **jusqu'à 4 variables** provenant de Home Assistant, comme :
- ⚡ Puissance soutirée
- ☀️ Production photovoltaïque
- 🌡️ Température extérieure
- 📊 Consommation journalière
- 💰 Prix de l'électricité
- 🔋 Niveau de batterie

## 📱 Interface Home Assistant

### Entités créées automatiquement

Après le téléversement du code, vous aurez **9 entités** dans HA :

#### **Capteurs de la station (4)**
1. `sensor.temperature` - Température intérieure
2. `sensor.humidite` - Humidité
3. `sensor.pression` - Pression atmosphérique
4. `sensor.co` - Monoxyde de carbone

#### **Contrôles (5)**
5. `select.mode_affichage` - Choix du mode (Temperature / Pression / **InfoHA**)
6. `select.info_slot_1` - Configuration ligne 1 du LCD
7. `select.info_slot_2` - Configuration ligne 2 du LCD
8. `select.info_slot_3` - Configuration ligne 3 du LCD
9. `select.info_slot_4` - Configuration ligne 4 du LCD

## 🎛️ Configuration étape par étape

### **Étape 1 : Activer le mode InfoHA**

Dans Home Assistant :
```
1. Aller dans l'appareil "Station Air"
2. Cliquer sur "Mode Affichage"
3. Sélectionner "infoha"
```

L'écran LCD affiche maintenant :
```
Slot 1: ---
Slot 2: ---
Slot 3: ---
Slot 4: ---
```

### **Étape 2 : Configurer les slots**

Pour chaque slot (ligne du LCD), choisissez ce que vous voulez afficher :

**Slot 1 - Ligne 1 du LCD :**
```
1. Cliquer sur "Info Slot 1"
2. Choisir par exemple "Puissance Soutirée"
```

**Slot 2 - Ligne 2 du LCD :**
```
1. Cliquer sur "Info Slot 2"
2. Choisir par exemple "Production PV"
```

**Slot 3 - Ligne 3 du LCD :**
```
1. Cliquer sur "Info Slot 3"
2. Choisir par exemple "Température Ext"
```

**Slot 4 - Ligne 4 du LCD :**
```
1. Cliquer sur "Info Slot 4"  
2. Choisir "Aucun" (ou une autre variable)
```

### **Étape 3 : Créer les automations dans HA**

Pour que les valeurs s'affichent, vous devez **envoyer les données** depuis HA vers l'ESP32.

**Fichier `automations.yaml` :**

```yaml
# Envoyer la puissance soutirée
- alias: "ESP32 - Puissance"
  trigger:
    - platform: state
      entity_id: sensor.power_consumption
  action:
    - service: mqtt.publish
      data:
        topic: "stationair/info/puissance"
        payload: "{{ states('sensor.power_consumption') }}"

# Envoyer la production PV
- alias: "ESP32 - Production PV"
  trigger:
    - platform: state
      entity_id: sensor.solar_production
  action:
    - service: mqtt.publish
      data:
        topic: "stationair/info/pv"
        payload: "{{ states('sensor.solar_production') }}"

# Envoyer température extérieure
- alias: "ESP32 - Temp Ext"
  trigger:
    - platform: state
      entity_id: sensor.outdoor_temperature
  action:
    - service: mqtt.publish
      data:
        topic: "stationair/info/temp_ext"
        payload: "{{ states('sensor.outdoor_temperature') }}"
```

**OU** utiliser une automation périodique (toutes les 30s) :

```yaml
- alias: "ESP32 - Rafraîchir InfoHA"
  trigger:
    - platform: time_pattern
      seconds: "/30"
  action:
    - service: mqtt.publish
      data:
        topic: "stationair/info/puissance"
        payload: "{{ states('sensor.power_consumption') | float(0) }}"
    
    - service: mqtt.publish
      data:
        topic: "stationair/info/pv"
        payload: "{{ states('sensor.solar_production') | float(0) }}"
    
    - service: mqtt.publish
      data:
        topic: "stationair/info/temp_ext"
        payload: "{{ states('sensor.outdoor_temperature') | float(0) }}"
```

## 📺 Affichage sur le LCD

### Format d'affichage

Chaque ligne affiche :
```
Label: Value Unit
```

**Exemples :**
```
Puissanc: 2.3k W      ← Puissance > 1000W affichée en kW
Prod PV: 1500 W       ← Valeur entre 100-1000
Temp Ext: 18.5 °C     ← Température avec 1 décimale
Conso Jo: 12.34 kWh   ← 2 décimales si < 100
```

### Formatage automatique

Le code adapte l'affichage selon la valeur :
- **≥ 1000** : Affiche en kilo (ex: `2.5k W`)
- **≥ 100** : Pas de décimale (ex: `150 W`)
- **≥ 10** : 1 décimale (ex: `12.5 °C`)
- **< 10** : 2 décimales (ex: `0.18 €/kWh`)

Les labels sont tronqués à **8 caractères** pour laisser de la place aux valeurs.

## 🔌 Topics MQTT

### Topics de configuration (HA → ESP32)

| Topic | Valeurs possibles |
|-------|-------------------|
| `stationair/mode/set` | `temperature`, `pression`, `infoha` |
| `stationair/slot1/set` | `Aucun`, `Puissance Soutirée`, `Production PV`, etc. |
| `stationair/slot2/set` | idem |
| `stationair/slot3/set` | idem |
| `stationair/slot4/set` | idem |

### Topics de données (HA → ESP32)

| Topic | Description | Unité affichée |
|-------|-------------|----------------|
| `stationair/info/puissance` | Puissance soutirée | W |
| `stationair/info/pv` | Production solaire | W |
| `stationair/info/temp_ext` | Température extérieure | °C |
| `stationair/info/conso_jour` | Consommation journalière | kWh |
| `stationair/info/prix_elec` | Prix électricité | €/kWh |
| `stationair/info/batterie_soc` | Niveau batterie | % |

## 🎨 Exemple de carte Lovelace

```yaml
type: vertical-stack
cards:
  # Sélection du mode
  - type: entities
    title: 📺 Affichage LCD
    entities:
      - entity: select.mode_affichage
        name: Mode
  
  # Configuration InfoHA
  - type: entities
    title: 📊 Configuration InfoHA
    entities:
      - entity: select.info_slot_1
        name: "📍 Ligne 1"
      - entity: select.info_slot_2
        name: "📍 Ligne 2"
      - entity: select.info_slot_3
        name: "📍 Ligne 3"
      - entity: select.info_slot_4
        name: "📍 Ligne 4"
  
  # Aperçu des capteurs
  - type: entities
    title: 🌡️ Capteurs Station
    entities:
      - sensor.temperature
      - sensor.humidite
      - sensor.pression
      - sensor.co
```

## 🧪 Test manuel avec MQTT Explorer

### 1. Configurer un slot

```
Topic: stationair/slot1/set
Payload: Puissance Soutirée
```

### 2. Envoyer une valeur

```
Topic: stationair/info/puissance
Payload: 2345.6
```

L'écran affiche :
```
Puissanc: 2.3k W
```

### 3. Changer de mode

```
Topic: stationair/mode/set
Payload: infoha
```

## 🐛 Dépannage

### Les slots affichent "Slot X: ---"

**Cause** : Aucune donnée n'a été reçue
**Solution** :
1. Vérifier que les automations HA sont actives
2. Vérifier les topics MQTT avec MQTT Explorer
3. Vérifier le Serial Monitor de l'ESP32

### Les valeurs ne se rafraîchissent pas

**Cause** : Automations non déclenchées
**Solution** :
- Utiliser une automation **time_pattern** pour envoyer périodiquement
- Vérifier que les sensors HA existent et ont des valeurs

### L'affichage est tronqué

**Cause** : Label + valeur + unité > 20 caractères
**Solution** :
- Le code tronque automatiquement à 20 caractères
- Les labels sont limités à 8 caractères
- Utiliser des noms courts dans les options du select

## 🚀 Extensions possibles

### Ajouter d'autres variables

**1. Dans le code ESP32**, ajouter un nouveau topic :

```cpp
else if (strcmp(topic, "stationair/info/ma_variable") == 0) {
  for (int i = 0; i < 4; i++) {
    if (infoSlots[i].label == "Ma Variable") {
      infoSlots[i].value = message.toFloat();
      infoSlots[i].unit = "Unité";
    }
  }
}
```

**2. Mettre à jour les discovery select** pour ajouter l'option :

```cpp
"options":["Aucun","Puissance Soutirée",...,"Ma Variable"],
```

**3. Créer l'automation dans HA** :

```yaml
- service: mqtt.publish
  data:
    topic: "stationair/info/ma_variable"
    payload: "{{ states('sensor.mon_sensor') }}"
```

## 📊 Exemple complet d'utilisation

**Configuration :**
- Slot 1 : Puissance Soutirée
- Slot 2 : Production PV
- Slot 3 : Prix Elec
- Slot 4 : Batterie SOC

**Affichage LCD :**
```
┌────────────────────┐
│Puissanc: 2.3k W    │
│Prod PV: 1850 W     │
│Prix Ele: 0.18 €/kWh│
│Batteri: 85 %       │
└────────────────────┘
```

**Avantages :**
- ✅ Vue instantanée de vos données énergie
- ✅ Pas besoin d'ouvrir HA
- ✅ Configurable à la volée depuis HA
- ✅ Jusqu'à 4 variables simultanées

Profitez bien de votre station connectée ! 🎉

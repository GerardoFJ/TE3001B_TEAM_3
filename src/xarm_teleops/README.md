# xarm_teleops — Teleoperación con Sensado de Fuerza

**TE3001B Fundamentación de Robótica** — Tecnológico de Monterrey 2026
Teleoperación maestro-esclavo entre dos xArm Lite 6 con retroalimentación háptica.

## Arquitectura del sistema

```
PC Maestro                              PC Esclavo
┌─────────────────────┐    UDP 5005    ┌─────────────────────┐
│  master_control.py  │ ─────────────► │  slave_control.py   │
│                     │ ◄────────────  │                     │
│  q_master →         │    UDP 5006    │  → ForceEstimator   │
│  ← F_ext, τ_meas    │  (F_ext, τ)   │  ← τ_meas           │
└──────────┬──────────┘                └──────────┬──────────┘
           │ Ethernet                             │ Ethernet
     xArm Lite 6 (Maestro)               xArm Lite 6 (Esclavo)
```

## Archivos

| Archivo | Descripción |
|---------|-------------|
| `src/master_control.py` | Lazo principal PC Maestro |
| `src/slave_control.py` | Lazo principal PC Esclavo |
| `src/force_estimator.py` | F̂_ext = (Jᵀ)⁺·Δτ con filtro pasa-bajas |
| `src/jacobian_utils.py` | Jacobiano geométrico xArm Lite 6 |
| `src/network_comm.py` | Comunicación UDP + medición RTT |
| `src/xarm_setup.py` | Inicialización y seguridad del robot |
| `src/net_test.py` | Test de latencia de red |

## Requisitos

```bash
pip install numpy scipy
# xArm Python SDK (ya incluido en el workspace)
```

## Configuración

### 1. Asignar IPs de los robots

Editar en `slave_control.py`:
```python
ARM_SLAVE_IP = '192.168.1.XXX'   # IP del xArm Esclavo
MASTER_IP    = '192.168.1.YYY'   # IP de la PC Maestro
```

Editar en `master_control.py`:
```python
ARM_MASTER_IP = '192.168.1.ZZZ'  # IP del xArm Maestro
SLAVE_IP      = '192.168.1.WWW'  # IP de la PC Esclava
```

### 2. Test de red (Pregunta A2)

```bash
# En PC Esclavo:
python3 src/net_test.py server

# En PC Maestro:
python3 src/net_test.py client 192.168.1.WWW
```

### 3. Ejecutar teleoperación

```bash
# En PC Esclavo (primero):
python3 src/slave_control.py

# En PC Maestro:
python3 src/master_control.py
```

## Parámetros ajustables

| Parámetro | Archivo | Descripción |
|-----------|---------|-------------|
| `ALPHA` | `slave_control.py` | Escalado posición maestro→esclavo |
| `BETA` | `master_control.py` | Ganancia reflexión de fuerza |
| `KH` | `master_control.py` | Ganancia corrección háptica [rad/N] |
| `cutoff_hz` | `force_estimator.py` | Frecuencia de corte filtro [Hz] |
| `CONTROL_HZ` | ambos | Frecuencia del lazo de control |

## Ecuaciones clave

**Estimación de fuerza:**
$$\hat{F}_{ext} = (J^\top)^+ \cdot (\tau_{meas} - \tau_{gravity})$$

**Feedback háptico (posición):**
$$\delta q = -K_h \cdot J^\top \cdot \beta \cdot \hat{F}_{ext}$$

## Estructura de entrega

```
xarm_teleops/
├── src/
│   ├── master_control.py
│   ├── slave_control.py
│   ├── force_estimator.py
│   ├── jacobian_utils.py
│   ├── network_comm.py
│   ├── xarm_setup.py
│   └── net_test.py
├── data/           # CSV/NPY de experimentos
├── figures/        # Imágenes para el reporte
└── resources/      # PDF y .tex del reporte
```
